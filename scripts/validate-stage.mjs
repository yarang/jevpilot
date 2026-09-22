// Checks a stage the way the simulator will read it, so an authored stage fails
// here rather than silently misbehaving at runtime. With no arguments it runs
// over the procedural worlds, which is the baseline a Blender stage has to meet.
import { generateWorld } from "../src/world.js";
import { roadGeometry } from "../src/road-geometry.js";
import { toManifest, validateManifest } from "../src/stage-format.js";
import { readFile } from "node:fs/promises";

const files = process.argv.slice(2);
let failed = 0;

async function check(label, manifest) {
  const problems = validateManifest(manifest);
  const size = Buffer.byteLength(JSON.stringify(manifest));
  console.log(
    `${problems.length ? "FAIL" : "ok  "}  ${label.padEnd(22)} ` +
      `${String(manifest.nodes.length).padStart(4)} nodes  ` +
      `${String(manifest.edges.length).padStart(4)} edges  ` +
      `${String(manifest.surfaces.length).padStart(5)} surfaces  ` +
      `${(size / 1024).toFixed(0).padStart(4)} KB`,
  );
  for (const p of problems.slice(0, 8)) console.log(`        - ${p}`);
  if (problems.length > 8)
    console.log(`        ... and ${problems.length - 8} more`);
  if (problems.length) failed++;
}

if (files.length) {
  for (const file of files)
    await check(file, JSON.parse(await readFile(file, "utf8")));
} else {
  const seeds = Number(process.env.SEEDS || 20);
  for (const type of ["town", "city", "highway"])
    for (let seed = 1; seed <= seeds; seed++) {
      const world = generateWorld(seed, type);
      await check(
        `${type} seed ${seed}`,
        toManifest(world, roadGeometry(world)),
      );
    }
}
console.log(failed ? `\n${failed} stage(s) failed` : "\nall stages valid");
process.exitCode = failed ? 1 : 0;
