// A stage manifest is the planning layer's source of truth: the road graph, the
// drivable surface, and the semantics the planner reads. The procedural
// generators and the Blender pipeline both emit this one shape, so
// roadGeometry, makeRoute and the section rules never learn where a stage came
// from.
//
// Surfaces are stored as flat centimetre integers. At stage scale that is well
// inside the tolerance the occupancy test works at, and it keeps a manifest
// small enough to import statically into the planner worker, which rebuilds its
// world synchronously and cannot await a fetch.
export const STAGE_FORMAT_VERSION = 1;
const CM = 100;

// roadOccupancy subtracts the car's footprint from each surface by clipping it
// against every edge in turn. That decomposition is only valid for a convex
// clip polygon: against a concave one it reports a car sitting inside the
// polygon as fully off-road, with no error raised. Every authored surface has
// to be convex, so this is the check the pipeline turns on that silence.
//
// Returns the worst corner cross product. Zero or above is convex; a negative
// value is a reflex corner, and its magnitude says how sharp.
export function convexity(points) {
  let worst = Infinity;
  for (let i = 0; i < points.length; i++) {
    const a = points[i],
      b = points[(i + 1) % points.length],
      c = points[(i + 2) % points.length];
    worst = Math.min(
      worst,
      (b.x - a.x) * (c.z - b.z) - (b.z - a.z) * (c.x - b.x),
    );
  }
  return points.length < 3 ? -Infinity : worst;
}

export const packSurface = (points) =>
  points.flatMap((p) => [Math.round(p.x * CM), Math.round(p.z * CM)]);

export const unpackSurface = (flat) =>
  Array.from({ length: flat.length / 2 }, (_, i) => ({
    x: flat[i * 2] / CM,
    z: flat[i * 2 + 1] / CM,
  }));

// The planner reads node control and section kind directly, so a manifest that
// invents new values would fail silently rather than loudly. Keep the accepted
// sets here, next to the format they belong to.
export const CONTROLS = ["none", "stop", "signal"];
export const SECTION_KINDS = [
  "local",
  "ramp_turn",
  "onramp",
  "merge",
  "interstate",
  "exit",
  "offramp",
  "town",
];

// The driving line is authored, not derived. makeRoute builds it from the grid
// for the procedural towns and from the ramp script for the Interstate, and
// neither rule describes a road someone drew, so the manifest carries the
// polyline itself.
export function routeToManifest(route) {
  const points = route.points;
  return {
    ids: route.ids,
    points: packSurface(points),
    // Arc length travels with the polyline. Recomputing it from packed points
    // accumulates the rounding of every chord, which over the Interstate's
    // 1,320 samples moved the far end of the road by more than a centimetre.
    // Stop lines and section boundaries are stored the same way, in
    // centimetres: snapping them to the nearest sample instead put an
    // Interstate stop line nine centimetres off, because the ramp script does
    // not place them on a sample.
    stations: points.map((p) => Math.round(p.s * CM)),
    crossings: (route.crossings ?? []).map((c) => ({
      nodeId: c.nodeId,
      x: c.x,
      z: c.z,
      approach: c.approach,
      exit: c.exit,
      stopS: Math.round(c.stopS * CM),
    })),
    ...(route.sections
      ? {
          sections: route.sections.map((s) => ({
            kind: s.kind,
            name: s.name,
            startS: Math.round(s.startS * CM),
            endS: Math.round(s.endS * CM),
            speedLimit: s.speedLimit,
            ...(s.laneHalfWidth ? { laneHalfWidth: s.laneHalfWidth } : {}),
          })),
        }
      : {}),
  };
}

export function routeFromManifest(manifest) {
  const points = unpackSurface(manifest.points);
  points.forEach((p, i) => (p.s = manifest.stations[i] / CM));
  return {
    ids: manifest.ids,
    points,
    length: points.at(-1).s,
    crossings: manifest.crossings.map((c) => ({
      nodeId: c.nodeId,
      x: c.x,
      z: c.z,
      approach: c.approach,
      exit: c.exit,
      stopS: c.stopS / CM,
    })),
    ...(manifest.sections
      ? {
          sections: manifest.sections.map((v) => ({
            kind: v.kind,
            name: v.name,
            startS: v.startS / CM,
            endS: v.endS / CM,
            speedLimit: v.speedLimit,
            ...(v.laneHalfWidth ? { laneHalfWidth: v.laneHalfWidth } : {}),
          })),
        }
      : {}),
  };
}

// A world the simulator can drive, carrying its surfaces so roadGeometry uses
// them instead of rebuilding from generator-specific fields the manifest does
// not describe.
export function fromManifest(manifest) {
  const nodes = manifest.nodes.map((n) => ({ ...n }));
  const world = {
    seed: manifest.id,
    type: manifest.type,
    theme: manifest.theme,
    nodes,
    byId: Object.fromEntries(nodes.map((n) => [n.id, n])),
    edges: manifest.edges.map((e) => ({ ...e })),
    objects: manifest.objects,
    bounds: manifest.bounds,
    startNode: manifest.start,
    nextNode: manifest.next,
    destination: manifest.destination,
    surfaces: manifest.surfaces.map(unpackSurface),
    route: routeFromManifest(manifest.route),
  };
  return world;
}

export function toManifest(world, surfaces) {
  return {
    version: STAGE_FORMAT_VERSION,
    id: `${world.type}-${world.seed}`,
    type: world.type,
    theme: world.theme,
    bounds: world.bounds,
    start: world.startNode,
    next: world.nextNode,
    destination: world.destination,
    nodes: world.nodes.map((n) => ({
      id: n.id,
      x: n.x,
      z: n.z,
      control: n.control,
      offset: n.offset,
      neighbors: n.neighbors,
      ...(n.townJunction ? { townJunction: true } : {}),
    })),
    edges: world.edges.map((e) => ({
      id: e.id,
      a: e.a,
      b: e.b,
      width: e.width,
      length: e.length,
      speedLimit: e.speedLimit,
      kind: e.kind ?? "street",
      name: e.name,
      ...(e.oneWay ? { oneWay: true } : {}),
    })),
    surfaces: surfaces.map((s) => packSurface(s.points)),
    objects: world.objects,
    route: routeToManifest(world.route),
  };
}

// Everything a stage can get wrong in a way the simulator would not report.
// Returns one string per problem; an empty array means the stage is usable.
export function validateManifest(manifest) {
  const problems = [];
  const say = (m) => problems.push(m);

  if (manifest.version !== STAGE_FORMAT_VERSION)
    say(`version ${manifest.version}, expected ${STAGE_FORMAT_VERSION}`);

  const byId = new Map(manifest.nodes.map((n) => [n.id, n]));
  if (byId.size !== manifest.nodes.length) say("duplicate node ids");

  // Ramps are deliberately one-way and say so on their edge. An asymmetric
  // link with no such edge is an authoring slip that would quietly strand
  // traffic, so the edge has to carry the intent.
  const pair = (a, b) => [a, b].sort().join("\u0000");
  const edges = new Map(manifest.edges.map((e) => [pair(e.a, e.b), e]));

  for (const n of manifest.nodes) {
    if (!CONTROLS.includes(n.control))
      say(`node ${n.id}: unknown control ${JSON.stringify(n.control)}`);
    if (![n.x, n.z, n.offset].every(Number.isFinite))
      say(`node ${n.id}: non-finite position or signal offset`);
    for (const id of n.neighbors) {
      if (!byId.has(id)) {
        say(`node ${n.id}: neighbor ${id} does not exist`);
        continue;
      }
      const edge = edges.get(pair(n.id, id));
      if (!edge) say(`node ${n.id} -> ${id}: link has no edge`);
      else if (!byId.get(id).neighbors.includes(n.id) && !edge.oneWay)
        say(`node ${n.id} -> ${id}: asymmetric link on a two-way edge`);
    }
  }

  for (const e of manifest.edges) {
    if (!byId.has(e.a) || !byId.has(e.b))
      say(`edge ${e.id}: endpoint missing from the graph`);
    if (!(e.width > 0)) say(`edge ${e.id}: width ${e.width}`);
    if (!(e.speedLimit > 0)) say(`edge ${e.id}: speed limit ${e.speedLimit}`);
  }

  // Reachability, so a stage cannot ship with a destination no route can get to.
  const seen = new Set([manifest.start]);
  for (const queue = [manifest.start]; queue.length;) {
    for (const id of byId.get(queue.shift())?.neighbors ?? [])
      if (!seen.has(id)) {
        seen.add(id);
        queue.push(id);
      }
  }
  for (const n of manifest.nodes)
    if (!seen.has(n.id)) say(`node ${n.id} is unreachable from the start`);
  if (!seen.has(manifest.destination))
    say("destination is unreachable from the start");

  manifest.surfaces.forEach((flat, i) => {
    if (flat.length < 6 || flat.length % 2)
      return say(`surface ${i}: ${flat.length / 2} points`);
    const points = unpackSurface(flat);
    // Winding is normalised on load, so check the polygon as given and as
    // reversed; only a shape that is concave either way is a real problem.
    const worst = Math.max(convexity(points), convexity([...points].reverse()));
    if (worst < 0)
      say(`surface ${i}: concave corner (cross ${worst.toFixed(3)})`);
  });

  const stations = manifest.route.stations ?? [];
  if (stations.length !== manifest.route.points.length / 2)
    say("route: one arc length per point is required");
  for (let i = 1; i < stations.length; i++)
    if (stations[i] < stations[i - 1])
      say(`route: arc length goes backwards at point ${i}`);

  for (const s of manifest.route.sections ?? []) {
    if (!SECTION_KINDS.includes(s.kind))
      say(`route section: unknown kind ${JSON.stringify(s.kind)}`);
    if (!(s.endS > s.startS))
      say(`route section ${s.kind}: endS is not past startS`);
    if (s.endS > stations.at(-1))
      say(`route section ${s.kind}: endS is past the end of the polyline`);
  }
  for (const c of manifest.route.crossings ?? [])
    if (!(c.stopS >= 0 && c.stopS <= stations.at(-1)))
      say(`crossing ${c.nodeId}: stop line is off the polyline`);

  for (let i = 1; i < manifest.route.ids.length; i++) {
    const from = byId.get(manifest.route.ids[i - 1]);
    if (!from?.neighbors.includes(manifest.route.ids[i]))
      say(
        `route step ${manifest.route.ids[i - 1]} -> ${manifest.route.ids[i]} is not an edge`,
      );
  }
  if (manifest.route.ids.at(-1) !== manifest.destination)
    say("route does not end at the destination");

  return problems;
}
