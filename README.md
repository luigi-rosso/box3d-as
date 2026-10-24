# box3d-as

Box3D, Erin Catto's 3D physics engine, ported to AnimaScript as a Rive
scripting library. Bit exact against native Box3D, step for step.

Upstream: https://github.com/erincatto/box3d at
`f36d061ba26998c87cb7dc593a4065aaa03fb7eb` (v0.1.0 plus fixes). Box3D is MIT
licensed; the port keeps that licence and its attribution, see `LICENSE`.

## Use

`lib/box3d.as` re-exports the public API with Box3D's C names. Ids are handles
(`b3WorldId`, `b3BodyId`, ...), plain data are structs, and callbacks are
closures. `CONVENTIONS.md` says how the C maps to AnimaScript.

```as
import { b3DefaultWorldDef, b3CreateWorld, b3DefaultBodyDef, b3CreateBody, b3_dynamicBody,
         b3DefaultShapeDef, b3MakeBoxHull, b3CreateHullShape, b3World_Step } from "./lib/box3d";

let world = b3CreateWorld(b3DefaultWorldDef());
let bodyDef = b3DefaultBodyDef();
bodyDef.type = b3_dynamicBody;
let body = b3CreateBody(world, bodyDef);
let box = b3MakeBoxHull(0.5, 0.5, 0.5);
b3CreateHullShape(body, b3DefaultShapeDef(), box.base);
b3World_Step(world, <f32>(1.0 / 60.0), 4);
```

The demo at the repo root is a rive-cli project:
`RIVE_WAMRC=~/Projects/tools/wamrc rive . --aot`. Its seven scenes are a
pyramid, a card house, ragdolls with a wrecking ball, rain on a mesh, a height
field, bullets and a kinematic capsule character. Interactions are orbit,
pick, shoot, and overlays for contacts, joints and the profile. See
`notes/demo.md`.

## What was ported

Everything in the library, about 41,000 lines of C, into about 54,500 lines of
AnimaScript across 56 files in `lib/`:

- **Collision shapes:** spheres, capsules, hulls with Box3D's quickhull,
  triangle meshes with their BVH, height fields and baked compounds.
- **Broad phase:** the dynamic tree broad phase.
- **Narrow phase:** GJK, shape casts, time of impact, SAT with its caches,
  convex, triangle and mesh manifolds, and contact recycling.
- **Solver:** the soft step solver with graph coloring and the 4 wide contact
  solver.
- **Joints:** all of them: distance, motor, parallel, prismatic, revolute,
  spherical, weld, wheel and filter.
- **Islands, sleeping, continuous collision and bullets.**
- **Sensors and events.**
- **Queries:** ray casts, shape casts and overlaps.
- **Character mover:** `b3World_CastMover`, `b3World_CollideMover`,
  `b3SolvePlanes`, `b3ClipVector`, plus the per body versions.

Skipped: debug draw, recording and replay, world snapshots, the built in
scheduler, and the x86 and NEON SIMD backends. The 4 lane SIMD layer is ported
on scalar lanes and on wasm `v128`.

## Parity

Native reference: the scalar Box3D build with `-ffp-contract=off`. Native
Box3D gives identical results for the scalar and NEON builds and for any
worker count (`notes/scenes.md`), so the port is held to bit exactness, not a
tolerance.

- **Module checks:** `verify/run.sh` runs 29 modules. Each compares every
  output value bit for bit against C over fixed and seeded random inputs,
  about 69 million values in total, all matching.
- **Trajectories:** `verify/traj.sh --large` runs 13 scenes. Every step's
  bodies, the mover state and the world counters match the native recording
  bit for bit. The scenes are pyramid, large pyramid, card house, ragdolls and
  chains, capsules on a mesh, height field, bullets, mover, many bodies, plank
  pile, mesh drop, village and queries.
- **Builds:** both gates pass on the scalar artifact and on the SIMD artifact,
  on AOT and on the interpreter.
- **Threaded runs:** `verify/threads.sh` checks the threaded port at 1, 2, 4
  and 8 lanes against the same recordings.
- **Upstream bugs found:** a clip buffer overflow, a hull builder hang, a
  stale move index write and a few API edge bugs, in `notes/upstream.md`. The
  port reproduces each one so it stays faithful.

## Performance

Single threaded, AOT on WAMR, SIMD artifact, against native Box3D SIMD (NEON)
on one worker, Apple M5 Max (`notes/perf_3.md`; the machine was shared, so
differences under about 20% are noise):

| scene | native SIMD ms/step | AnimaScript ms/step | ratio |
|---|---|---|---|
| large pyramid, 5051 boxes | 4.95 | 5.28 | 1.07x |
| plank pile | 0.98 | 1.02 | 1.04x |
| many bodies, 10781 boxes | 8.71 | 11.53 | 1.32x |
| village, compounds | 2.86 | 4.08 | 1.43x |
| mesh drop | 6.13 | 10.17 | 1.66x |
| small pyramid | 0.034 | 0.061 | 1.8x |
| chains, joints | 0.044 | 0.090 | 2.0x |

The first AOT build ran large pyramid at 6x native scalar. Three passes closed
that gap:

1. **Solver:** step scratch reused instead of allocated, the 4 lane kernels on
   `v128`, flat body state gathers, and no body sim copies
   (`notes/perf_solver.md`).
2. **Narrow phase and queries:** nothing allocated per contact or per query,
   field reads instead of struct copies, and a call free GJK
   (`notes/perf_collide.md`).
3. **Data layout:** prepare rows loaded as `v128`, contacts reuse their slot
   objects, and continuous collision works through addresses
   (`notes/perf_3.md`).

The interpreter runs the small pyramid at about 1 ms a step, which is 30x
native, so the live demo needs `--aot`.

Most of what remains is outside the library's source:

- WAMR's software bounds checks cost 6 to 8%.
- WAMR reloads the memory base after stores in functions that call, 8 to 10%
  of instructions in the collide paths.
- WAMR spills v128 registers and pays a stub on every call.
- rasc lacks the SIMD lane loads and stores.
- `clock()` calls `getenv` on every call.

## Threading

Box3D's tasks run on script lanes (`lib/lanes.as`, `lib/lane_collide.as`,
`lib/lane_tree.as`, enabled with `b3World_EnableLanes`). They cover the narrow
phase, every solver stage and the tree rebuild. Results stay bit exact at any
lane count.

On today's share-nothing lanes the step does not get faster, while native
gains 3.0 to 3.7x from 1 to 8 workers:

- Moving the solver's data costs 60 to 85% of the solver's own work.
- Each of the 84 colored solver stage syncs per step costs about as much as
  the work it separates.

`notes/threading.md` has the tables, the analysis, and a ranked list of
changes to AnimaScript threading that would fix this, led by shared linear
memory between lanes. The lane hooks do nothing unless enabled.

## Notes

- `notes/findings.md`: rasc and runtime issues hit along the way, each with a
  minimal repro.
- `notes/upstream.md`: upstream Box3D issues.
- `notes/<module>.md`: per module porting notes.
- `notes/bench.md` and `bench/`: benches and profiling recipes.

## Tooling

- rasc and rive-cli from rive master.
- `verify/` needs the native Box3D checkout at `~/Projects/box3d`, built in
  `build/scalar` and `build/release`.
- The native recordings in `verify/scenes/out` are not committed;
  `NATIVE=1 verify/traj.sh` records them.
