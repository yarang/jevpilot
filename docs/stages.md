# Authoring a stage

A stage is two files that travel together:

- `src/stages/<id>.json` — the **manifest**: the road graph, the drivable
  surface, and the semantics the planner reads. Imported statically, because
  the planner worker rebuilds its world synchronously and cannot await a fetch.
- `public/stages/<id>.glb` — the **visual mesh**, loaded only by the renderer.

The planner never reads the `.glb`, and the renderer never reads the manifest's
surfaces. Keeping them apart is what lets a stage be authored in Blender
without the collision layer inheriting whatever geometry came out of the
exporter.

Validate before shipping:

```sh
node scripts/validate-stage.mjs src/stages/<id>.json
node scripts/validate-stage.mjs          # the procedural stages, as a baseline
```

## The three constraints

**Surfaces must be convex.** `roadOccupancy` subtracts the car's footprint from
each surface by clipping it against every edge in turn. That decomposition is
only valid for a convex clip polygon. Against a concave one it reports a car
sitting _inside_ the polygon as fully off-road — no error, no warning, the
planner simply discards every path. Build road surfaces as strips of
quadrilaterals, and junctions as their own convex patches. The existing
Interstate does exactly this: 3,495 convex quads rather than one ribbon.

**The manifest is a graph, not a mesh.** Routing needs `nodes[].neighbors` and
`edges[].length`; the stop and signal rules need `nodes[].control`; the speed
profile and candidate sampling need `route.sections[].kind`. A triangle soup
carries none of it. Author roads as curves with properties attached, and let the
converter derive the mesh — not the other way round.

**The world is rebuilt from the manifest on both threads.** The main thread and
the planner worker each construct their own copy and never exchange it. If the
two disagree, the planner evaluates candidates against asphalt the renderer did
not draw. This is why the manifest is data rather than a procedure: identical
input, identical world.

## Blender conventions

Author roads as **curves**, not meshes. A curve keeps the centreline and width
as parameters, which is what `makeRoute` and the surface builder both need. A
mesh forces the converter to recover that, which is fragile.

Attach semantics as **custom properties**; the glTF exporter carries them
through as `extras`.

| Object         | Property          | Values                                                                           |
| -------------- | ----------------- | -------------------------------------------------------------------------------- |
| Road curve     | `kind`            | `local`, `ramp_turn`, `onramp`, `merge`, `interstate`, `exit`, `offramp`, `town` |
| Road curve     | `width`           | metres of drivable asphalt                                                       |
| Road curve     | `speed_limit`     | m/s                                                                              |
| Road curve     | `lane_half_width` | metres; `2.25` on the interstate, `3` elsewhere                                  |
| Road curve     | `one_way`         | `true` for ramps                                                                 |
| Junction empty | `control`         | `none`, `stop`, `signal`                                                         |
| Junction empty | `offset`          | signal phase offset, 0–23                                                        |

Export as glTF 2.0 binary (`.glb`) with **Export Custom Properties** enabled and
Draco compression on — the decoder already ships in `public/draco/`.

Units are metres, +x east, +z south, heading 0 north. Blender's default axes
differ; export with Y up so the converter's mapping holds.

## What the converter does

`scripts/build-stage.mjs` reads the `.glb`, pulls the curves and their `extras`,
and writes the manifest. It is the only place that turns authored geometry into
the planner's convex surfaces, so it is also where convexity is guaranteed
rather than hoped for: roads become quad strips along the curve, junctions
become their own patches, and the result goes through `validateManifest` before
it is written.

A stage that fails validation is not written. The error names the surface index
or node id, so it can be found in the `.blend` file.
