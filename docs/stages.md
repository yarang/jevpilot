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

glTF has no curve primitive, and the Blender manual is explicit: _"curves and
other non-mesh data are not preserved, and must be converted to meshes prior to
export."_ Empty objects are not listed as exportable either. What survives an
export is triangles, node transforms, and `extras`.

So the `.glb` is **not** how a stage's data travels. Blender writes the manifest
itself, through a Python script that reads the curves from `bpy.data` where they
are still curves, and the `.glb` carries only what the renderer draws. Nothing
has to be recovered from triangles.

Author roads as **curves**: a curve keeps the centreline and width as
parameters, which is what the driving line and the surface builder both need.
Attach semantics as **custom properties**, which the script reads directly.

| Object         | Property          | Values                                                                           |
| -------------- | ----------------- | -------------------------------------------------------------------------------- |
| Road curve     | `kind`            | `local`, `ramp_turn`, `onramp`, `merge`, `interstate`, `exit`, `offramp`, `town` |
| Road curve     | `width`           | metres of drivable asphalt                                                       |
| Road curve     | `speed_limit`     | m/s                                                                              |
| Road curve     | `lane_half_width` | metres; `2.25` on the interstate, `3` elsewhere                                  |
| Road curve     | `one_way`         | `true` for ramps                                                                 |
| Junction empty | `control`         | `none`, `stop`, `signal`                                                         |
| Junction empty | `offset`          | signal phase offset, 0-23                                                        |

Export the visual mesh as glTF 2.0 binary (`.glb`) with Draco compression on;
the decoder already ships in `public/draco/`. Custom properties do not need to
be included in that export, since the manifest carries them.

Units are metres, +x east, +z south, heading 0 north. Blender exports +Y up, and
the script maps its axes when it writes the manifest.

## What the exporter does

The Blender script turns each road curve into a strip of quadrilaterals along
its centreline and each junction into its own patch. Building the surface this
way is what makes it convex by construction rather than by luck, which is the
one property `roadOccupancy` cannot check for itself.

It runs `validateManifest` equivalent checks before writing, and a stage that
fails is not written. The error names the surface index or node id, so it can be
found back in the `.blend` file.

Whatever writes a manifest, run the validator on the result:

```sh
npm run validate:stage -- src/stages/<id>.json
```
