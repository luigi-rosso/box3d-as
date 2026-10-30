# box3d-as

Box3D, Erin Catto's 3D physics engine, ported to AnimaScript as a Rive scripting library. It keeps Box3D's C API
names, and every result matches native Box3D bit for bit, step for step.

**Live demo: [box3d-demo-wlepqf.rive.to](https://box3d-demo-wlepqf.rive.to).** Eight interactive scenes running in the
browser: drag to orbit, grab and throw bodies, Tab to shoot.

Upstream: [erincatto/box3d](https://github.com/erincatto/box3d) at `f36d061ba26998c87cb7dc593a4065aaa03fb7eb` (v0.1.0
plus fixes). Box3D is MIT licensed. The port keeps that licence and Erin Catto's copyright on every file (`LICENSE`).

## What it is

- **All of Box3D's simulation.** Upstream `src/` is ported to about 55,700 lines of AnimaScript in 56 files under
  `lib/`, one file per C source, with functions in C's order:
  - shapes: spheres, capsules, hulls with Box3D's quickhull, triangle meshes, height fields and baked compounds;
  - the dynamic tree broad phase;
  - the narrow phase: GJK, shape casts, time of impact, SAT with its caches, and convex, triangle and mesh manifolds;
  - the soft step solver, with graph coloring and the 4 wide contact solver on wasm SIMD;
  - all nine joint types;
  - islands and sleeping, continuous collision and bullets, sensors and events;
  - ray casts, shape casts and overlaps, and the character mover.
- **Box3D's API, under the same names.** `lib/box3d.as` re-exports 670 of the 716 public functions. Ids are handles
  (`b3BodyId`), plain C structs are AnimaScript structs, and callbacks are closures. `CONVENTIONS.md` has the full
  C to AnimaScript mapping.
- **Bit exact.** The same float operations in the same order, checked against native Box3D on every module and on
  every step of 13 scenes (see Parity below).
- **Threaded.** Parallel calls run the pair finding, the narrow phase, the solver stages and finalize over the world's
  own memory and scale like native, natively and in the browser; lanes run the same tasks on copies. Results are the
  same at any part or lane count (see Threading below).
- **Left out:** debug draw; recording, replay and the world snapshots they use; file IO; the built in thread
  scheduler, whose place lanes take; and the SSE2, AVX2 and NEON backends. The 4 lane SIMD layer runs on wasm
  `v128`, or on scalar lanes when the build has no SIMD.

## Quick start

```as
import { b3Body_GetPosition, b3CreateBody, b3CreateHullShape, b3CreateWorld, b3DefaultBodyDef, b3DefaultShapeDef,
	b3DefaultWorldDef, b3DestroyWorld, b3MakeBoxHull, b3Vec3, b3World_Step, b3_dynamicBody } from "./lib/box3d";

let worldId = b3CreateWorld(b3DefaultWorldDef());

let groundDef = b3DefaultBodyDef();
groundDef.position = b3Vec3(0.0, -1.0, 0.0);
let groundId = b3CreateBody(worldId, groundDef);
b3CreateHullShape(groundId, b3DefaultShapeDef(), b3MakeBoxHull(10.0, 1.0, 10.0).base);

let bodyDef = b3DefaultBodyDef();
bodyDef.type = b3_dynamicBody;
bodyDef.position = b3Vec3(0.0, 4.0, 0.0);
let bodyId = b3CreateBody(worldId, bodyDef);
b3CreateHullShape(bodyId, b3DefaultShapeDef(), b3MakeBoxHull(0.5, 0.5, 0.5).base);

let timeStep: f32 = <f32>(1.0 / 60.0);
for (let i = 0; i < 180; ++i) {
	b3World_Step(worldId, timeStep, 4);
}
let y = b3Body_GetPosition(bodyId).y; // 0.5: the box rests on the ground
b3DestroyWorld(worldId);
```

rive-cli resolves imports only inside a project, so copy or sync `lib/` into yours. As in C, write float constants
as `f32` (`<f32>(1.0 / 60.0)`; a constant expression of float literals is `f64`).

- **`examples/`** has one short file per area of the API: [hello world](examples/hello_world.as),
  [shapes](examples/shapes.as) (sphere, capsule, hull, mesh, height field, compound),
  [joints](examples/joints.as) (a revolute chain, a prismatic elevator), [queries](examples/queries.as) (ray casts,
  a shape cast, overlaps), [events](examples/events.as) (contact, sensor and hit events),
  [the character mover](examples/mover.as) and [threading](examples/threading.as).
  `examples/check.sh` runs each one under `rive --test` and checks what it shows (`examples/tests.as`), such as the
  box coming to rest, or four lanes and parallel calls ending bit for bit where the serial path does.
- **The demo** at the repo root is a rive-cli project with eight scenes: pyramid, card house, ragdolls with a wrecking
  ball, rain on a mesh, height field, bullets, a capsule character, and threads. Orbit, pick and shoot, with overlays
  for contacts, joints and the profile (`notes/demo.md`). The solids draw instanced on the GPU with a depth buffer,
  through AnimaScript's GPU API and WGSL (`demo/gpu_draw.as`, `demo/scene3d.wgsl`), one draw per geometry: the
  script's draw takes 0.01 to 0.1 ms where the 2D painter took 0.6 to 1.8 ms, and `--define=VECTOR_DRAW` keeps the
  painter. A sun casts cascaded shadow maps from every solid (`demo/shadows.as`); `H` steps through high, soft, low
  and off. The threads scene stirs 2048 balls and capsules and draws each step's tasks on each thread as it runs, with
  the step time over the last five seconds, on serial, lanes or parallel calls. A runtime without threads offers
  serial only, and one with threads but without parallel calls (`@rive-app/webgl2-threads` 2.48.0 and older) offers
  lanes. It needs AOT for interactive rates:

  ```sh
  RIVE_WAMRC=<wamrc> rive . --aot
  ```

## API availability

Generated by `docs/api.py` from the upstream public headers (`box3d.h`, `collision.h`, `math_functions.h`,
`types.h`, `base.h` and `id.h`: every `B3_API` function plus the inline ones), against what `lib/box3d.as` exports.
[`docs/api.md`](docs/api.md) lists every function with the module that defines it. Box3D has no chain shapes; meshes,
height fields and compounds have a row of their own.

| area | ported | not ported, and why |
|---|---|---|
| world | 46 / 47 | `b3World_DumpShapeBounds` (other: file IO) |
| bodies | 77 / 77 |  |
| shapes | 33 / 33 |  |
| mesh, height field and compound shapes | 10 / 10 |  |
| joints, common | 26 / 26 |  |
| distance joint | 25 / 25 |  |
| filter joint | 2 / 2 |  |
| motor joint | 22 / 22 |  |
| parallel joint | 8 / 8 |  |
| prismatic joint | 24 / 24 |  |
| revolute joint | 23 / 23 |  |
| spherical joint | 28 / 28 |  |
| weld joint | 10 / 10 |  |
| wheel joint | 38 / 38 |  |
| queries | 7 / 7 |  |
| events | 21 / 21 |  |
| character mover | 6 / 6 |  |
| geometry and collision | 98 / 101 | `b3DumpHeightData` (other: file IO), `b3LoadHeightField` (other: file IO), `b3ValidateCompound` (other: validates serialized bytes) |
| dynamic tree | 22 / 24 | `b3DynamicTree_Load` (other: file IO), `b3DynamicTree_Save` (other: file IO) |
| math | 112 / 112 |  |
| ids and defs | 17 / 17 |  |
| base | 15 / 15 |  |
| debug draw | 0 / 4 | all: debug draw |
| recording and replay | 0 / 36 | all: recording and replay |
| **all** | **670 / 716** | |

Exported but with nothing to do in a runtime that owns memory and threads: `b3SetAllocator`, `b3GetByteCount` (0),
`b3IsAVX2Available` (false), `b3Yield`, `b3Sleep`, `b3World_DumpMemoryStats`, and the `b3Destroy*` functions for
hulls, meshes, height fields and compounds (the collector frees them).

## Parity

The reference is native Box3D's scalar build (`BOX3D_DISABLE_SIMD`, `-ffp-contract=off`). Native Box3D gives the
same bits for its scalar and NEON builds and for any worker count (`notes/scenes.md`), so the port is held to bit
exactness, with no tolerance.

| gate | what it checks | covers |
|---|---|---|
| `verify/run.sh` | each module's outputs against a C driver over fixed and seeded random inputs, every value's bits | 29 modules, about 69 million values |
| `verify/traj.sh --large` | every body's state, the mover and the world counters, every step, against a native recording | 13 scenes |
| `verify/threads.sh --large` | the per step state hash under rive-cli on parallel calls at 1, 2, 4 and 8 parts and the default, and on lanes at 1, 2, 4 and 8 on workers and on jobs | 195 runs |
| `bench/web/verify.sh` | the state hash of every step on V8, for the port and for C Box3D on wasm | 13 scenes, 6 builds |
| `examples/check.sh` | the examples run and show what they say, interpreted and on AOT | 8 cases |

The 13 scenes are pyramid, large pyramid, card house, ragdolls and chains, capsules on a mesh, height field,
bullets, mover, many bodies, plank pile, mesh drop, village (compounds) and queries. Both module and trajectory
gates pass on the scalar and the SIMD artifact. Every bench run below also checks its hash against the recording.
The port found four upstream bugs (a clip buffer overflow, a hull builder hang, a stale move index write, API edge
cases; `notes/upstream.md`) and reproduces each, to stay faithful.

## Performance compared to C

ms per step. The single threaded tables take each run's median step and the min over interleaved rounds. Machine:
Apple M5 Max (6 performance and 12 efficiency cores), macOS 26.5, shared with other work, so compare ratios within a
table, not milliseconds across tables.

### V8: the port against C Box3D compiled to wasm

The same scenes in node 26.3.1 (V8 14.6): C Box3D at f36d061 through emscripten 6.0.13 (`-O3 -ffp-contract=off`;
SIMD is Box3D's SSE2 path on `-msimd128`, scalar is `BOX3D_DISABLE_SIMD`), and the port's release bake (rasc `-O2`,
binaryen 133, as rive-cli bakes a release). Native ARM64 Box3D for scale. main at the master toolchain, 5 rounds,
320 runs, load 5.8 to 8.6, 2026-10-26; every run's hash matched the recording on every step.

| scene | native NEON | C wasm SIMD | AS SIMD | AS / C SIMD | C wasm scalar | AS scalar | AS / C scalar |
|---|---|---|---|---|---|---|---|
| large_pyramid | 3.59 | 4.28 | 4.08 | 0.95 | 10.31 | 8.75 | 0.85 |
| many_bodies | 7.59 | 8.83 | 9.73 | 1.10 | 21.14 | 18.56 | 0.88 |
| mesh_drop | 5.49 | 6.53 | 7.00 | 1.07 | 11.33 | 11.99 | 1.06 |
| village | 2.63 | 3.04 | 3.70 | 1.21 | 4.91 | 5.07 | 1.03 |
| plank_pile | 0.670 | 0.740 | 0.726 | 0.98 | 0.830 | 0.801 | 0.96 |
| chains | 0.0175 | 0.0239 | 0.0246 | 1.03 | 0.0236 | 0.0242 | 1.03 |
| pyramid | 0.0291 | 0.0374 | 0.0362 | 0.97 | 0.0928 | 0.0740 | 0.80 |
| queries | 0.116 | 0.142 | 0.154 | 1.08 | 0.149 | 0.161 | 1.08 |

- With SIMD the port runs at 0.95 to 1.21x C on the same engine; C on wasm is itself 1.1 to 1.4x native.
- Without SIMD it beats C's scalar wasm on the solver heavy scenes: C's scalar build gets no autovectorization on
  wasm, while the port's scalar artifact runs one lane at a time and skips empty lanes.
- The same session ran the port's builds from the parallel-calls toolchain, where `notes/perf_4.md` measured 0.99
  to 1.17x: within 5% of main's on every scene, either way. Headless Chrome 155 agrees with node (`notes/web.md`).
- With parallel calls in lib, on master's rasc at 4a861ff147 against C on emsdk 4.0.23, the AOT session below put
  the SIMD artifact at 1.00 to 1.12x C on V8, 3 rounds: the main lane runs the same code as before.

Reproduce: `EMSDK=<emsdk> WASM_OPT=<binaryen 133 wasm-opt> bench/web/build.sh`, `bench/web/build_native.sh`, then
`ROUNDS=5 node bench/web/bench.mjs` (`notes/web.md` has the build details).

### WAMR AOT: the port against native Box3D

rive-cli's own runtime: the release bake of `bench/as` on WAMR AOT (`--aot`, `RIVE_WASM_AOT_SYNC=o3`, the SIMD
artifact, software bounds checks), interleaved with native NEON Box3D on one worker and with the same module on V8.
rive master at 4a861ff147 (CLI, rasc and its wamrc), `bench/web/aot.mjs`, 3 rounds, 96 runs, load 7.8 to 20.8,
2026-10-28; medians from step 1, every hash matched.

| scene | native NEON | AS WAMR AOT | AOT / native | AS on V8 | V8 / native |
|---|---|---|---|---|---|
| large_pyramid | 3.33 | 3.71 | 1.11 | 3.85 | 1.16 |
| many_bodies | 7.15 | 8.46 | 1.18 | 8.98 | 1.26 |
| mesh_drop | 4.43 | 5.08 | 1.15 | 5.44 | 1.23 |
| village | 2.50 | 2.91 | 1.16 | 3.18 | 1.27 |
| plank_pile | 0.661 | 0.647 | 0.98 | 0.710 | 1.07 |
| chains | 0.0179 | 0.0269 | 1.50 | 0.0269 | 1.50 |
| pyramid | 0.0291 | 0.0346 | 1.19 | 0.0352 | 1.21 |
| queries | 0.113 | 0.142 | 1.26 | 0.148 | 1.31 |

- On the big scenes WAMR AOT runs the port at 1.11 to 1.18x native, ahead of V8 now; plank_pile is at native speed.
  On 2026-10-26, before master's WAMR changes (#15091 onward, such as wide calls on AOT), the same scenes ran at 1.30
  to 1.45x, and chains at 2.25x.
- The small scenes vary most between sessions; `notes/perf_4.md` measured 1.0 to 1.6x across all eight.
- These keep the software bounds checks of the `o3` tier, for comparison with the earlier sessions. Master's CLI now
  ships hardware bounds on macOS and Linux (#15153), its default tier, which `notes/perf_4.md` measured about 5%
  faster on the big scenes.
- Counted in cycles on the script thread of a quiet machine, master's hardware bounds tier runs large_pyramid at
  1.04x native and pyramid at 1.12x (`notes/perf_5.md`). Under cache contention our collide slowed twice as much as
  C's, from misses the core could not overlap, so it now loads the next contact ahead of the current one.

Reproduce: `ROUNDS=3 RIVE_WAMRC=<wamrc> node bench/web/aot.mjs`, then `node bench/web/bench.mjs --table=<log>`, or
`bench/as/run.sh` for the AOT tier alone against bench/native (`notes/bench.md`).

What is left, from the profiles (`notes/perf_4.md`): prepare and store impulses (1.2 to 1.5x C on V8), where each
lane's manifold lives in its contact's own array; collide and pairs (1.1 to 1.3x), where contacts and shapes are
objects reached through references; on WAMR, software bounds checks and the per call cost of AOT code, which is why
the small, call heavy scenes (chains, queries) trail most there.

### Threaded

ms per step at 1 / 2 / 4 / 8 workers, lanes or parts, min over rounds (`notes/threading.md`, `notes/web.md`). C Box3D
uses its own scheduler: natively on threads, on the web on emscripten pthreads (8 pooled workers) in headless Chrome.
Parallel calls ran on rive master at 4a861ff147, natively on WAMR AOT (`aot -O3`, software bounds) and in Chrome 156 on
librive's threads build, 2026-10-28. The native columns are means from step 1, 3 rounds interleaved, load 10 to 14; the
Chrome columns are medians from step 1, 3 rounds interleaved, load 7 to 8, with the ratio to C's pthreads build in the
same session. Lanes are the 2026-10-26 session in Chrome 155, the solver stages on workers, with the serial step in
parentheses.

| scene | native NEON | AS parallel calls, native AOT | C wasm pthreads, Chrome | AS parallel calls, Chrome | AS lanes, Chrome |
|---|---|---|---|---|---|
| large_pyramid | 3.44 / 1.81 / 1.06 / 1.00 | 3.98 / 2.16 / 1.31 / 1.11 | 3.70 / 1.97 / 1.12 / 0.98 | 3.81 / 2.05 / 1.25 / 1.10 (1.03 to 1.13x C) | 4.65 / 4.81 / 4.81 / 5.34 (4.64) |
| many_bodies | 7.49 / 3.79 / 2.13 / 1.61 | 9.49 / 4.70 / 2.65 / 1.86 | 7.90 / 3.93 / 2.15 / 1.60 | 8.39 / 4.24 / 2.45 / 1.83 (1.06 to 1.14x) | 9.81 / 10.69 / 8.92 / 9.97 (10.11) |
| mesh_drop | 4.91 / 2.62 / 1.56 / 1.18 | 5.91 / 3.22 / 1.96 / 1.49 | 4.82 / 2.59 / 1.51 / 1.23 | 5.72 / 3.20 / 1.88 / 1.56 (1.19 to 1.27x) | 8.80 / 8.15 / 8.24 / 8.30 (8.77) |
| village | 2.12 / 1.34 / 0.94 / 0.92 | 2.55 / 1.62 / 1.15 / 1.16 | 2.83 / 1.72 / 1.13 / 1.09 | 3.16 / 2.01 / 1.36 / 1.38 (1.12 to 1.26x) | 3.16 / 3.72 / 3.84 / 4.82 (3.22) |

Parallel calls scale like C's threads, natively and on the web: natively at 1.11 to 1.27x native at every count, and
in Chrome at 1.03 to 1.14x C's pthreads build on large_pyramid and many_bodies (3.5 and 4.6x at 8 parts), where lanes
left the port 4.4 to 4.6x behind. mesh_drop and village stay 1.2 to 1.3x behind in both places, in the port's own
parallel step. Every run matched native on every step it checked, in Chrome, Firefox and WebKit too, and without cross
origin isolation the same bake runs serially and stays bit exact.

Reproduce: `THREADS=1 bench/web/build.sh`, then `bench/web/parallel.mjs` (the commands and the web harness builds are
in `notes/web.md`), and `bench/threads/run.sh` natively.

## Threading

- **Parallel calls:** `b3World_EnableParallel(worldId, tasks)` from `lib/parallel_step.as` runs the pair finding,
  the narrow phase with the tree rebuild, the solver stages and finalize as `@parallel.inplace` calls over the world's
  own memory, with no serialization ([example](examples/threading.as)). It is bit exact at every part count and
  scales like native's workers, natively on WAMR and in Chrome and Firefox on librive's threads build.
  - It needs rive master at 4a861ff147 or later (#15105), where parallel calls landed.
  - The part count is `Parallel.size()`: the job pool plus the main lane natively (`--job-threads=N` gives N + 1, the
    default keeps to the fast cores), one per core up to 8 on an isolated web page with the threads build, and 1
    anywhere without threads, where the calls run on the main lane with the same results (`notes/web.md`).
    `b3SetParallelLimit(parts)` caps it at run time.
  - Parts run the same functions as the main lane. `Parallel.inPart()` keeps callbacks and allocations out of the
    parts' copies, and the per worker scratch is marked `@parallel.shared`, since each part uses its own worker's.
  - A runtime from before parallel calls traps on the first call, and gives a script no way to ask whether it has
    them. The demo's Threads scene tells such runtimes apart before offering Parallel (`notes/web.md`).
- **Lanes:** `b3World_EnableLanes(worldId, lanes, backend, tasks)` from `lib/lanes.as` puts the narrow phase, the
  solver stages and the tree rebuild on workers or jobs. Lanes share no memory, so each step serializes the solver's
  data to the lanes and back and syncs 84 colored stages: the whole step gains at most 1.1 to 1.5x, and village gets
  slower with every lane. `notes/threading.md` measures why; it is what parallel calls replaced.

## Branches

`main` is the only branch: the port, the examples, the gates and every bench. It builds and passes every gate on rive
master at 4a861ff147 or later. The older branches (`single-core`, `parallel`, `web-bench`, `threads`) are merged in and
kept for history.

## Layout and tooling

| path | what |
|---|---|
| `lib/` | the port; `box3d.as` is the public API, `parallel_step.as` the parallel calls backend, `lanes.as` lanes |
| `examples/` | one example per API area, `tests.as` and `check.sh` |
| `main.as`, `demo/`, `rive.yaml` | the demo |
| `verify/` | parity gates: `cref/` C drivers, `as/` AnimaScript drivers, `scenes/` the 13 scenes in C and AnimaScript |
| `bench/native`, `bench/as`, `bench/threads`, `bench/web` | native timings, WAMR AOT timings, threaded timings, V8 against C on wasm |
| `docs/` | `api.md` and the script that generates it |
| `notes/` | porting notes, findings, perf passes |

Toolchain, from a rive master checkout (`R` below):

```sh
cd $R/packages/rasc/animascript && RIVE_OUT=out/release RIVE_PREMAKE_ARGS= build_rive.sh release -- rasc
cd $R/packages/rive-cli && ./build.sh release
# wamrc for --aot, from the patched WAMR the CLI build downloads, against Homebrew llvm@18
cmake -S $R/packages/rive-cli/dependencies/bytecodealliance_wasm-micro-runtime_WAMR-2.4.5/wamr-compiler \
	-B wamrc-build -G Ninja -DCMAKE_BUILD_TYPE=Release -DWAMR_BUILD_WITH_CUSTOM_LLVM=1 \
	-DLLVM_DIR=/opt/homebrew/opt/llvm@18/lib/cmake/llvm && cmake --build wamrc-build
```

`build_rive.sh` is in `$R/packages/runtime/build`. A dependency's patches apply only when it is first downloaded, so
delete `packages/*/dependencies/bytecodealliance_wasm-micro-runtime_*` when rive's WAMR patches change. The scripts
default to `~/Projects/rive-master`; `RASC`, `AS_STD`, `RIVE` and `RIVE_WAMRC` override.

The gates need upstream Box3D at `~/Projects/box3d` (`BOX3D`), built in `build/scalar` (`BOX3D_DISABLE_SIMD=ON`,
`-ffp-contract=off`) and `build/release`. The native recordings in `verify/scenes/out` are not committed:
`NATIVE=1 verify/traj.sh --large` records them.

```sh
KEEP_GOING=1 RASC_FLAGS="-O" verify/run.sh
KEEP_GOING=1 RASC_FLAGS="-O --enable-simd" verify/run.sh
RASC_FLAGS="-O" verify/traj.sh --large
RASC_FLAGS="-O --enable-simd" verify/traj.sh --large
RIVE_WAMRC=<wamrc> verify/threads.sh --large
RIVE_WASM_AOT_SYNC=interp examples/check.sh && RIVE_WAMRC=<wamrc> examples/check.sh --aot
python3 verify/exports.py && python3 docs/api.py
```

`verify/threads.sh` runs parallel calls, workers and jobs (`BACKENDS="3 2 1"`); `BACKENDS=3 PARTS="1 4"` keeps it
short. A CLI built from source is a dev build, which `bench/web/lanes_bake.sh` needs for `--unsigned-riv`: release
CLIs write a `.riv` only through `--publish`.

Benches: `bench/native/run.sh`, `bench/as/run.sh` (WAMR AOT, `notes/bench.md`), `bench/threads/run.sh`, and
`bench/web/` (`notes/web.md`: `build.sh` builds C Box3D with emscripten and the port's release bake, `bench.mjs`
runs the V8 rounds, `aot.mjs` WAMR AOT against V8 and native).

## Notes

| note | what |
|---|---|
| `CONVENTIONS.md` | how C maps to AnimaScript: files, names, types, structs and classes, pointers, callbacks |
| `notes/findings.md` | rasc and runtime issues hit along the way, each with a minimal repro |
| `notes/upstream.md` | upstream Box3D bugs the port found |
| `notes/perf_solver.md`, `perf_collide.md`, `perf_3.md`, `perf_4.md`, `perf_5.md` | the five performance passes |
| `notes/web.md` | Box3D on the web: the port against C on V8, threaded, in Chrome |
| `notes/threading.md` | lanes and parallel calls: design, results, what AnimaScript threading should change |
| `notes/bench.md`, `notes/scenes.md`, `notes/parity.md` | bench recipes, the scenes and the parity method |
| `notes/demo.md` | the demo's structure |
| `notes/<module>.md` | per module porting notes: deviations, skipped functions |
