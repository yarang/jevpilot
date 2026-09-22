import test from "node:test";
import assert from "node:assert/strict";
import { evaluate, questions, validState } from "../server/jev.js";
import { Simulation } from "../src/simulation.js";
import {
  candidateChoices,
  vectorWeights,
  decisionControls,
  decisionSelection,
  physics,
  maneuverSteering,
} from "../src/planning.js";
import { prepareJevRequest, expandJevAnswers } from "../src/jev-request.js";

// Jev answers with the anonymised ids it was offered (v0, v1, ...), not the
// internal batch ids, and only for questions that were actually asked. A motion
// question with a single option is resolved locally and never reaches the API.
function apiResponse(state, choice) {
  const prepared = prepareJevRequest(state);
  const ids = Object.keys(prepared.request.questions.vector?.criteria ?? {});
  choice ??= ids[0];
  const answers = {
    vector: {
      choice,
      probabilities: Object.fromEntries(
        ids.map((id) => [id, id === choice ? 0.8 : 0.2 / (ids.length - 1)]),
      ),
    },
  };
  if (prepared.request.questions.motion)
    answers.motion = {
      choice: "drive",
      probabilities: { drive: 0.8, stop: 0.2 },
    };
  return {
    model: "jev-latest",
    answers,
    usage: { input_tokens: 2000, output_tokens: 80 },
  };
}

// The same answers once the server has restored the real candidate ids and
// merged the locally resolved questions back in. This is what the selection and
// weighting helpers consume.
function localAnswers(state, choice) {
  const prepared = prepareJevRequest(state);
  const alias = Object.keys(prepared.aliases).find(
    (id) => prepared.aliases[id] === choice,
  );
  return expandJevAnswers(prepared, apiResponse(state, alias).answers);
}

test("Jev chooses a complete maneuver from the submitted random batch", async () => {
  const sample = new Simulation(42).decisionState();
  const original = globalThis.fetch;
  let request;
  globalThis.fetch = async (url, options) => {
    request = { url, ...options };
    return Response.json(apiResponse(sample));
  };
  try {
    const result = await evaluate(sample, {
      TYPESAFE_API_KEY: "test-only-key",
    });
    assert.equal(request.headers.Authorization, "Bearer test-only-key");
    const sent = JSON.parse(request.body);
    assert.deepEqual(Object.keys(sent.questions), ["vector"]);
    // Jev is offered anonymised ids; candidate_ids maps them back to the batch.
    assert.deepEqual(
      Object.keys(sent.questions.vector.criteria),
      Object.keys(result.candidate_ids),
    );
    assert.deepEqual(
      Object.values(result.candidate_ids),
      Object.keys(candidateChoices(sample)),
    );
    const selected = sample.vectors[result.answers.vector.choice];
    assert.deepEqual(result.controls, {
      steering: selected.steering,
      velocity: selected.velocity_mps,
    });
    assert.equal(result.batch_id, sample.batch_id);
    assert.equal(result.cost_usd, 0.000084);
    assert(!JSON.stringify(result).includes("test-only-key"));
    assert.deepEqual(decisionControls(sample, result), result.controls);
  } finally {
    globalThis.fetch = original;
  }
});

test("unknown choices, invalid probabilities and API errors cannot produce controls", async () => {
  const sample = new Simulation(42).decisionState(),
    original = globalThis.fetch;
  try {
    globalThis.fetch = async () => new Response("", { status: 429 });
    await assert.rejects(() => evaluate(sample, {}), /rate limit/);
    globalThis.fetch = async () =>
      Response.json(apiResponse(sample, "old_batch_v0"));
    await assert.rejects(() => evaluate(sample, {}), /incomplete decision/);
    const bad = apiResponse(sample);
    delete bad.answers.vector.probabilities[
      Object.keys(bad.answers.vector.probabilities)[1]
    ];
    globalThis.fetch = async () => Response.json(bad);
    await assert.rejects(() => evaluate(sample, {}), /incomplete decision/);
    // Eligibility keys off collision_imminent, so an imminent path is never
    // offered at all and a choice naming one cannot produce controls.
    const hit = structuredClone(sample);
    const excluded = Object.keys(candidateChoices(hit))[0];
    hit.vectors[excluded].collision_imminent = true;
    assert(!Object.hasOwn(candidateChoices(hit), excluded));
    assert(!Object.values(prepareJevRequest(hit).aliases).includes(excluded));
    globalThis.fetch = async () => Response.json(apiResponse(hit, "stale_v0"));
    await assert.rejects(() => evaluate(hit, {}), /incomplete decision/);
  } finally {
    globalThis.fetch = original;
  }
});

test("control validation rejects another batch and changed steering or speed", () => {
  const sim = new Simulation(42),
    first = sim.decisionState();
  const answers = localAnswers(first);
  const selection = decisionSelection(first, answers);
  const selected = first.vectors[selection.choice];
  const result = {
    batch_id: first.batch_id,
    answers,
    selection,
    controls: { steering: selected.steering, velocity: selected.velocity_mps },
  };
  assert(decisionControls(first, result));
  assert.equal(decisionControls(sim.decisionState(), result), null);
  assert.equal(
    decisionControls(first, {
      ...result,
      controls: { ...result.controls, steering: 0.85 },
    }),
    null,
  );
  assert.equal(
    decisionControls(first, {
      ...result,
      controls: { ...result.controls, velocity: 200 },
    }),
    null,
  );
});

test("probabilities describe only the corresponding candidate batch and expire", () => {
  const state = new Simulation(42).decisionState(),
    answer = localAnswers(state).vector;
  assert.equal(
    vectorWeights(answer, candidateChoices(state))[answer.choice].probability,
    0.8,
  );
  assert.equal(vectorWeights(answer, candidateChoices(state), 1801), null);
  assert.equal(vectorWeights(answer, { other: {} }), null);
});

test("every displayed path exactly integrates its submitted steering and speed", () => {
  for (const initial of [0, 14, -2]) {
    const sim = new Simulation(42);
    sim.player.speed = initial;
    sim.decisionState();
    for (const [id, candidate] of Object.entries(sim.lastPlan.vectors)) {
      const ghost = { ...sim.player };
      const projection = sim.lastPlan.projections[id];
      for (let i = 1; i <= 60; i++) {
        physics(
          ghost,
          maneuverSteering(ghost, candidate),
          candidate.velocity_mps,
          0.05,
        );
        assert(Math.abs(ghost.x - projection.points[i].x) < 1e-9);
        assert(Math.abs(ghost.z - projection.points[i].z) < 1e-9);
      }
    }
  }
});

test("malformed candidate tables and unsafe speed ranges are rejected before API use", () => {
  const state = new Simulation(42).decisionState();
  assert(validState(state));
  const id = Object.keys(state.vectors)[0];
  for (const changes of [
    { steering: NaN },
    { steering: 0.9 },
    { velocity_mps: 100 },
    { velocity_mps: -1 },
  ]) {
    const bad = structuredClone(state);
    Object.assign(bad.vectors[id], changes);
    assert.equal(validState(bad), false);
  }
  assert.equal(validState({ ...state, batch_id: "other" }), false);
  assert.equal(questions(state).vector.type, "choice");
});

test("eligible choices exclude collisions and fall back to braking when blocked", () => {
  const state = new Simulation(42).decisionState();
  const choices = candidateChoices(state);
  assert(
    Object.values(choices).every(
      (v) => v.velocity_mps > 0 && v.stays_on_road && !v.collision_imminent,
    ),
  );
  // decisionState drops the stop vector while nothing requires a stop, so the
  // blocked fallback has to be tested against a batch that still carries one.
  const stopId = `${state.batch_id}_stop`;
  const blocked = {
    ...state,
    vectors: {
      ...Object.fromEntries(
        Object.entries(state.vectors).map(([id, v]) => [
          id,
          { ...v, collision_imminent: true },
        ]),
      ),
      [stopId]: {
        ...Object.values(state.vectors)[0],
        velocity_mps: 0,
        steering: 0,
      },
    },
  };
  assert.deepEqual(Object.keys(candidateChoices(blocked)), [stopId]);
  delete blocked.vectors[stopId];
  assert.equal(validState(blocked), false);
});
