import test from "node:test";
import assert from "node:assert/strict";
import { generateWorld } from "../src/world.js";
import { roadGeometry, roadOccupancy } from "../src/road-geometry.js";
import {
  convexity,
  fromManifest,
  packSurface,
  unpackSurface,
  toManifest,
  validateManifest,
} from "../src/stage-format.js";
import { Simulation } from "../src/simulation.js";

const manifestFor = (seed, type) => {
  const world = generateWorld(seed, type);
  return toManifest(world, roadGeometry(world));
};

test("every procedural stage satisfies the manifest contract", () => {
  for (const type of ["town", "city", "highway"])
    for (const seed of [1, 7, 42]) {
      const problems = validateManifest(manifestFor(seed, type));
      assert.deepEqual(problems, [], `${type} seed ${seed}: ${problems[0]}`);
    }
});

// roadOccupancy clips the car's footprint against each surface edge in turn.
// That decomposition silently reports a car sitting inside a concave polygon as
// fully off-road, so the guard has to be a build-time check, not a runtime one.
test("concavity is caught by the validator, not by the occupancy test", () => {
  const lShape = [
    { x: 0, z: 0 },
    { x: 10, z: 0 },
    { x: 10, z: 10 },
    { x: 6, z: 10 },
    { x: 6, z: 4 },
    { x: 0, z: 4 },
  ];
  assert(convexity(lShape) < 0);
  assert(convexity(lShape.slice(0, 4)) >= 0);

  // The car sits well inside the L, and occupancy still calls it off-road.
  const car = { x: 2, z: 2, heading: 0, width: 2, depth: 3 };
  const surface = {
    points: lShape,
    minX: 0,
    maxX: 10,
    minZ: 0,
    maxZ: 10,
  };
  assert.equal(roadOccupancy(car, [surface]).on_road, false);

  const manifest = manifestFor(42, "town");
  manifest.surfaces.push(packSurface(lShape));
  assert(
    validateManifest(manifest).some((p) => p.includes("concave")),
    "the validator has to reject what occupancy cannot",
  );
});

test("the validator rejects each way a stage can be wrong", () => {
  const cases = {
    "neighbor ghost": (m) => m.nodes[0].neighbors.push("ghost"),
    "asymmetric link": (m) => {
      const first = m.nodes.find((n) => n.neighbors.length);
      m.nodes.find((o) => o.id === first.neighbors[0]).neighbors = [];
    },
    "unknown control": (m) => (m.nodes[0].control = "roundabout"),
    "zero width": (m) => (m.edges[0].width = 0),
    "route gap": (m) => m.route.ids.splice(1, 0, "ghost"),
    "short route": (m) => m.route.ids.pop(),
    "version drift": (m) => (m.version = 99),
    "self-intersecting surface": (m) =>
      m.surfaces.push(
        packSurface([
          { x: 0, z: 0 },
          { x: 10, z: 10 },
          { x: 10, z: 0 },
          { x: 0, z: 10 },
        ]),
      ),
  };
  for (const [label, mutate] of Object.entries(cases)) {
    const manifest = structuredClone(manifestFor(42, "town"));
    mutate(manifest);
    assert(validateManifest(manifest).length > 0, `${label} went unreported`);
  }
});

// Surfaces ride to the planner worker as centimetre integers so a manifest stays
// small enough to import statically. A centimetre is far inside the tolerance
// roadOccupancy works at, but the round trip still has to be lossless enough
// that on-road stays on-road.
test("centimetre packing preserves occupancy decisions", () => {
  const world = generateWorld(42, "town");
  const surfaces = roadGeometry(world);
  for (const surface of surfaces) {
    const back = unpackSurface(packSurface(surface.points));
    assert.equal(back.length, surface.points.length);
    for (const [i, p] of back.entries()) {
      assert(Math.abs(p.x - surface.points[i].x) <= 0.005);
      assert(Math.abs(p.z - surface.points[i].z) <= 0.005);
    }
  }
  const car = { ...world.route.points[0], heading: 0, width: 1.9, depth: 4.75 };
  const repacked = surfaces.map((s) => {
    const points = unpackSurface(packSurface(s.points));
    return {
      points,
      minX: Math.min(...points.map((p) => p.x)),
      maxX: Math.max(...points.map((p) => p.x)),
      minZ: Math.min(...points.map((p) => p.z)),
      maxZ: Math.max(...points.map((p) => p.z)),
    };
  });
  assert.equal(
    roadOccupancy(car, repacked).on_road,
    roadOccupancy(car, surfaces).on_road,
  );
});

// A manifest is the whole input an authored stage gets, so it has to be enough
// to drive on. Packing to centimetres is lossy on purpose; what has to survive
// is every decision, not every digit.
test("a stage rebuilt from its manifest drives the same", () => {
  for (const type of ["town", "city", "highway"]) {
    const original = generateWorld(42, type);
    const manifest = structuredClone(
      toManifest(original, roadGeometry(original)),
    );
    assert.deepEqual(validateManifest(manifest), []);
    const rebuilt = fromManifest(manifest);

    // Stop lines and section boundaries decide where the car has to halt, so
    // they round-trip exactly rather than within a tolerance.
    assert.equal(
      rebuilt.route.crossings.length,
      (original.route.crossings ?? []).length,
    );
    for (const [i, c] of rebuilt.route.crossings.entries())
      assert.equal(
        c.stopS,
        Math.round(original.route.crossings[i].stopS * 100) / 100,
      );
    for (const [i, s] of (rebuilt.route.sections ?? []).entries()) {
      assert.equal(s.kind, original.route.sections[i].kind);
      assert.equal(
        s.startS,
        Math.round(original.route.sections[i].startS * 100) / 100,
      );
    }

    assert.equal(roadGeometry(rebuilt).length, roadGeometry(original).length);

    const from = (world) => {
      const sim = new Simulation(42, type);
      sim.world = world;
      sim.world.route = world.route;
      sim.player.route = world.route;
      return sim.decisionState();
    };
    const before = from(original),
      after = from(rebuilt);

    assert.deepEqual(Object.keys(after.vectors), Object.keys(before.vectors));
    assert.equal(after.speed_ceiling_mps, before.speed_ceiling_mps);
    for (const id of Object.keys(before.vectors)) {
      const a = before.vectors[id],
        b = after.vectors[id];
      for (const key of [
        "stays_on_road",
        "stays_in_lane",
        "collision_imminent",
      ])
        assert.equal(b[key], a[key], `${type} ${id}.${key}`);
      assert(
        Math.abs(b.steering - a.steering) < 0.005,
        `${type} ${id}.steering`,
      );
      assert(Math.abs(b.route_error_m - a.route_error_m) < 0.05);
      assert(Math.abs(b.lane_error_m - a.lane_error_m) < 0.05);
    }
  }
});
