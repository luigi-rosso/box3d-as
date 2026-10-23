# bench: step timings and the AOT profile

## Profiling timers

`b3GetTicks`, `b3GetMilliseconds` and `b3GetMillisecondsAndReset` (src/timer.c, POSIX branch) are in
`lib/core.as` on `clock()` from `rive/os`: ticks are nanoseconds, `<u64>(clock() * 1e9)`, and the
millisecond conversions are C's `(float)((now - ticks) / 1000000.0)`. Every `b3Profile` field is filled
where C fills it:

| C | port |
|---|---|
| physics_world.c `b3World_Step`: step, pairs, collide, solve, sensors | already in place in `lib/physics_world.as` |
| solver.c `b3SolverTask` (main lane): prepareConstraints, integrateVelocities, warmStart, solveImpulses, integratePositions, relaxImpulses, restitution, storeImpulses | `lib/solver.as` |
| solver.c `b3Solve`: solverSetup, constraints, transforms, jointEvents, hitEvents, refit, bullets, sensorHits, sleepIslands | `lib/solver.as` |
| island.c `b3SplitIslandTask`: splitIslands | `lib/island.as` |

broad_phase.c and sensor.c read no timers at this pin. rasc refuses a `ref` to a primitive local
(notes/findings.md), so `b3GetMillisecondsAndReset(&ticks)` callers keep the ticks in a one field struct,
`b3Ticks` in core.as, and pass `ticks.value`.

Results stay bit exact: `KEEP_GOING=1 verify/run.sh` passes all 28 modules with `-O`, and solver, world,
bodyshape and joints pass with `RASC_FLAGS=`. Under node the clock import is a stub that returns 0; solver
and world were also run once with `rive_rt_v1.now` bound to `performance.now`, 0 mismatches.

## The AnimaScript bench

`bench/as/` is a rive-cli project (`rive.yaml`, `main.as`, wasm scripting, `dangerouslyFast: true`).
rive-cli resolves imports only inside the project directory (`../../lib` reaches `/lib/box3d.as` in its
virtual tree, and it does not follow a symlinked `lib`), so `run.sh` and `profile.sh` rsync `lib/` into
`bench/as/lib/` (ignored) before each run; `main.as` imports `./lib/box3d`. `main.as` ports the two
builders it needs from `verify/scenes/scenes.c` (`CreatePyramid`, `CreateSmallPyramid`,
`CreateManyPyramids`), rather than importing the parity agent's `scenes.as`, which is still moving.

- `--define=SCENE=0` pyramid base 10, `1` large_pyramid base 100, `2` many_bodies; `--define=STEPS=n`
  (defaults 200, 200, 100).
- The world is built in `init` with `workerCount = 1`; `advance` takes one `b3World_Step(1/60, 4)` per
  frame, timed with `clock()`; the first step is untimed (as native). After the last step it logs
  `BENCH RESULT` with mean, min, median and max ms per step over the timed steps and the mean per step of
  every `b3Profile` field.
- `--bench` bakes release scripts (no fuel). Frame collections run at frame boundaries, outside the timed
  step.

Commands:

```sh
bench/as/run.sh                                  # ROUNDS=3 NATIVE_REPEATS=3; writes bench/as/out/session_*.log, prints the tables
bench/as/profile.sh 1 300 15                     # scene, steps, seconds of Time Profiler; OUT= keeps the trace
# one run by hand, after an rsync of lib/ into bench/as/lib/
RIVE=~/Projects/rive-master/packages/rive-cli/out/release/rive
RIVE_WAMRC=~/Projects/tools/wamrc RIVE_WASM_AOT_SYNC=o3 $RIVE bench/as --aot --bench=210 --define=SCENE=1   # AOT
$RIVE bench/as --bench=210 --define=SCENE=0                                                                 # interpreter
```

Each run's log has `script tier: aot -O3` or `script tier: interp`; every run below was checked.

## Results

Session `bench/as/out/session_20261023_133219.log`, 2026-10-23, Apple M5 Max, 1 worker. Three rounds;
each round runs `bench/native/run.sh` (SIMD then scalar, 3 repeats) and then the AnimaScript runs, scene by
scene, so native and AnimaScript alternate. The machine was shared with other agents: the 1 minute load
was 16 to 34 throughout (the column in parentheses), against about 12 for the native numbers in
notes/scenes.md (large_pyramid SIMD 6.64, scalar 10.54 ms/step there), so absolute numbers here are
inflated by 1.6 to 1.8x on both sides; the ratios within a round are the usable facts.

Native ms/step is the mean over a repeat's timed steps, min over repeats; AnimaScript's is the mean over
the run's timed steps. Both then take the min over rounds. `x scalar` divides by native scalar of the
same scene. The last column is the AnimaScript run's per step min and median.

| scene | impl | best ms/step | x scalar | per round ms/step (load) | AS step min / median of best run |
|---|---|---|---|---|---|
| large_pyramid | native simd | **11.902** | 0.7 | 13.90 (27.6), 11.90 (26.9), 12.76 (20.1) |  |
| large_pyramid | native scalar | **16.833** | 1.0 | 17.30 (27.2), 17.58 (25.6), 16.83 (19.5) |  |
| large_pyramid | AS aot | **103.480** | 6.1 | 107.47 (24.9), 103.48 (19.3), 109.96 (18.7) | 84.95 / 96.31 |
| many_bodies | native simd | **22.811** | 0.7 | 24.39 (22.7), 22.81 (19.8), 25.44 (18.7) |  |
| many_bodies | native scalar | **34.582** | 1.0 | 36.59 (21.1), 34.58 (19.1), 36.39 (18.3) |  |
| many_bodies | AS aot | **194.703** | 5.6 | 196.10 (28.6), 194.70 (16.9), 198.20 (16.1) | 162.37 / 181.41 |
| pyramid | native simd | **0.076** | 0.6 | 0.08 (34.2), 0.10 (18.8), 0.09 (16.4) |  |
| pyramid | native scalar | **0.126** | 1.0 | 0.13 (34.2), 0.15 (18.8), 0.15 (16.4) |  |
| pyramid | AS aot | **0.868** | 6.9 | 0.87 (28.2), 0.99 (18.2), 0.99 (16.6) | 0.81 / 0.82 |
| pyramid | AS interp | **8.038** | 63.6 | 8.04 (26.4), 8.56 (19.5), 8.71 (20.4) | 6.74 / 7.97 |

Stage breakdown, mean ms per step, min over rounds (restitution, splitIslands, bullets, events, sensors
and sleepIslands are 0 or near 0 in these scenes and left out):

| scene | impl | step | pairs | collide | solve | solverSetup | constraints | prepareConstraints | integrateVelocities | warmStart | solveImpulses | integratePositions | relaxImpulses | storeImpulses | transforms | refit |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| large_pyramid | native simd | 11.901 | 0.436 | 2.716 | 8.748 | 0.002 | 8.250 | 2.085 | 0.158 | 0.730 | 1.732 | 0.273 | 2.572 | 0.698 | 0.476 | 0.020 |
| large_pyramid | native scalar | 16.831 | 0.411 | 2.700 | 13.639 | 0.002 | 13.200 | 2.704 | 0.140 | 1.661 | 2.943 | 0.245 | 4.736 | 0.707 | 0.419 | 0.016 |
| large_pyramid | AS aot | 103.478 | 0.724 | 8.919 | 93.832 | 8.078 | 84.529 | 9.678 | 0.252 | 13.768 | 24.558 | 0.416 | 33.350 | 2.503 | 1.180 | 0.041 |
| many_bodies | native simd | 22.810 | 0.000 | 5.831 | 16.978 | 0.002 | 16.014 | 4.413 | 0.371 | 1.312 | 3.112 | 0.558 | 4.736 | 1.443 | 0.961 | 0.000 |
| many_bodies | native scalar | 34.581 | 0.000 | 5.950 | 28.631 | 0.002 | 27.666 | 6.152 | 0.388 | 3.425 | 6.010 | 0.575 | 9.698 | 1.394 | 0.961 | 0.001 |
| many_bodies | AS aot | 194.702 | 0.001 | 15.791 | 178.404 | 14.535 | 161.287 | 18.047 | 0.568 | 26.364 | 46.542 | 0.883 | 63.444 | 4.872 | 2.406 | 0.002 |
| pyramid | native simd | 0.076 | 0.000 | 0.011 | 0.065 | 0.000 | 0.060 | 0.007 | 0.002 | 0.007 | 0.016 | 0.003 | 0.023 | 0.002 | 0.004 | 0.000 |
| pyramid | native scalar | 0.126 | 0.000 | 0.011 | 0.115 | 0.000 | 0.110 | 0.010 | 0.002 | 0.017 | 0.030 | 0.003 | 0.047 | 0.002 | 0.004 | 0.000 |
| pyramid | AS aot | 0.867 | 0.000 | 0.056 | 0.809 | 0.005 | 0.788 | 0.073 | 0.004 | 0.135 | 0.239 | 0.005 | 0.323 | 0.008 | 0.013 | 0.000 |
| pyramid | AS interp | 8.037 | 0.001 | 0.335 | 7.695 | 0.045 | 7.482 | 0.759 | 0.067 | 1.214 | 2.220 | 0.081 | 3.038 | 0.096 | 0.160 | 0.001 |

What the tables say:

- AOT runs 6.1x native scalar on large_pyramid, 5.6x on many_bodies and 6.9x on pyramid; against native
  SIMD it is 8.5 to 11.4x. The interpreter is 9.3x slower than AOT on pyramid (64x native scalar).
- The solver is 91% of the AOT step (large_pyramid): relax 33.4, solve 24.6, warm start 13.8, prepare 9.7
  ms. The iterations are 7.0 to 8.3x native scalar and prepare 3.6x, matching the profile (spills,
  multi-value calls, bounds checks).
- `solverSetup` is 8.1 ms per step in AnimaScript and 0.002 ms in C: the port allocates and zero fills
  every per step array (`wideConstraints` and `wideManifolds`, about 6.5 MB for large_pyramid, the stages,
  blocks and spans) where C bumps its stack allocator. It is 8% of the step and the biggest single gap
  ratio.
- collide is 3.3x native scalar (8.9 against 2.7 ms), so the narrow phase ports better than the solver;
  `b3ResolveContactBodySim`'s sim copies are its largest piece in the profile.
- Per step medians sit 6 to 7% below the AOT means: a few steps run several times longer (max 337 ms
  in a 96 ms mean run, 1268 ms once in the profile run). The heap grows about 6 MB per step (20315 wasm
  pages, 1.3 GB, after 200 large_pyramid steps; 30042 pages after 300) and rive warns "script heap grew
  480MB over 120 frames ... something is accumulating references", so the per step arrays above are not
  reclaimed, and the long steps line up with memory growth.

## AOT profile

large_pyramid, 300 steps, AOT -O3, 1 worker; Time Profiler attached for 15 s of stepping (6622 samples
at 1 ms, 99.3% with a script frame). Self time by wasm function, from `bench/as/profile.sh 1 300 15`.
`[entry]` is WAMR's exported entry stub `aot_func#N` that every call between AOT functions goes
through: it checks the native stack bound, reloads and stores back every stack passed argument
(the 9th float argument on) and every multi-value result pointer, then tail calls the body
`aot_func_internal#N`. The stub's share is listed next to the body's. Instruction counts are the body's
arm64 code from `wamrc --format=object`.

| # | function | self % | body + stub % | why it costs |
|---|---|---|---|---|
| 1 | `b3SolveContactsW` | 19.3 | 19.3 | The relax and solve loop over 4 lane constraints: 5741 instructions per body, 2260 loads and 1482 stores against 605 float ops. Each `b3FloatW4` is four f32 locals and each `b3Vec3W4` twelve, so the live set far exceeds 32 FP registers and LLVM spills; every constraint field load also pays WAMR's software bounds check (305 conditional branches). C's scalar build autovectorizes the same lanes into `.4s` NEON, so native does one instruction where we do four plus spills. |
| 2 | `b3PushContactsW` | 12.3 | 12.3 | Same shape as solve (the biased iterations): 2736 instructions, 1000 loads, 743 stores, 217 float ops, 14 out of line helper calls per block. |
| 3 | `b3CrossW4` | 4.1 | 8.1 | An `@inline` 4 lane cross product (36 float ops) that stays a call: rasc -O never inlines struct returning functions and binaryen's inliner keeps it out of line (over the 100 node flexible limit, called from 27 sites). 24 float arguments (16 on the stack) and 12 results (11 written through result pointers) make the call cost as much as the math, and the entry stub copies the 20 stack slots again. |
| 4 | `b3GatherBodies` | 4.4 | 8.1 | Copies four whole 64 B body states out of `Array<b3BodyState>` (each index bounds checked by rasc and by WAMR), then returns 52 floats as a multi-value result, 51 of them through memory, so each gather writes and rereads the lanes twice. C gathers with direct loads into registers. |
| 5 | `b3PrepareConstraintW` | 8.0 | 8.0 | 7004 instructions, 612 conditional branches (rasc index checks plus software bounds checks on the 296 stores into the wide constraint and manifold), 4 `__linkSlot` write barriers for the lane reference scratch arrays, 9 `b3CrossW4` and 7 `b3MulMVW` calls. |
| 6 | `b3WarmStartContactsW` | 7.1 | 7.1 | Gather, apply impulses, scatter; 738 loads and 688 stores against 121 float ops, so it is spill and copy bound like solve. |
| 7 | `b3MulMVW` | 2.3 | 5.1 | 4 lane matrix times vector (60 float ops) out of line: 36 float arguments and 12 results through memory; the stub is more than half of it. |
| 8 | `b3ResolveContactBodySim` | 2.7 | 4.6 | Returns a whole `b3BodySim` (252 B, 63 leaves) by value per contact side in the collide task: 615 loads and 419 stores, no float math. C returns a pointer. notes/world.md already flags it. |
| 9 | `b3MulAddSVW4` | 2.0 | 4.4 | 24 float ops behind a 28 argument, 12 result call; stub and body cost the same. |
| 10 | `b3InvRotateVectorW` | 2.8 | 3.5 | Calls `b3CrossW4` twice, so it pays two nested multi-value round trips; 75 stores for 40 float ops. |
| 11 | `b3ScatterBodies` | 2.5 | 3.2 | Writes 24 floats back with `states[index].field = ...` per field, so each store re-reads the array data pointer and re-checks the index (28 trap checks in 290 instructions); 29 float arguments arrive mostly on the stack. |
| 12 | `<lambda>_1087` (collide task) | 2.8 | 2.8 | The `b3CollideTask` closure with the contact update inlined: 421 conditional branches in 3287 instructions (bounds and null checks), 8 `__link` write barrier and 2 `__new` call sites, plus the sim copies of row 8. |
| 13 | `b3MulSubSVW4` | 1.1 | 2.5 | As `b3MulAddSVW4`. |
| 14 | `b3MulSubMVW` | 0.9 | 2.2 | 72 float ops behind 48 float arguments; stub and spills dominate. |
| 15 | `b3MulAddMVW` | 0.7 | 1.9 | As `b3MulSubMVW`. |
| 16 | `b3StoreImpulsesW` | 1.5 | 1.5 | Scalar lane extracts written into `Array<b3Manifold>` elements through places: 129 conditional branches in 772 instructions, almost all bounds checks. |
| 17 | `<lambda>_1070` (finalize bodies) | 1.3 | 1.3 | The `b3FinalizeBodiesTask` closure: per body copies of state and sim in and out (notes/solver.md hazard 2), 380 conditional branches; cheap per body but 5050 bodies. |
| 18 | `b3ExecuteBlock` | 1.2 | 1.2 (77.8 incl.) | Stage dispatcher with integrate velocities and positions inlined; its self time is those integrations plus the per block switch. |
| 19 | `b3GatherInvInertiaW` | 0.3 | 0.5 | Gathers 24 floats from sims as a multi-value return through memory, as row 4. |
| 20 | `b3PlaceLeaf` | 0.3 | 0.3 | Tree rebuild in the pair update; scalar node copies in `StaticArray<b3TreeNode>` with 106 bounds check branches. |

Taken together: the seven small 4 lane helpers (rows 3, 7, 9, 10, 13 to 15) with their entry stubs are 28%
of the samples, the gathers, scatters and sim copies (rows 4, 8, 11, 19) another 16%, and the four wide
solver bodies 47%, which are themselves mostly spill traffic. The allocator and the collector barely show
during steps (`__linkSlot` 0.09%, `__riveFrameMinor` 0.05%, `__new` below the list); the "script tier: aot
-O3" log line and the absence of interpreter symbols confirm the tier.

## Findings for the optimization pass (not acted on)

- The `@inline` 4 lane helpers of `lib/simd.as` that return structs (`b3CrossW4`, `b3MulMVW`,
  `b3MulAddSVW4`, `b3InvRotateVectorW`, ...) stay calls in the release bake: rasc -O skips struct returning
  functions and binaryen's inliner keeps them over its size limit. Inlining them, or writing the hot
  bodies with lanes as plain locals, goes after the 28% of samples they and their stubs hold, before any SIMD.
- `b3GatherBodies` returns 52 floats and `b3ResolveContactBodySim` a 63 leaf struct by value; WAMR AOT
  returns every result past the first through memory.
- WAMR's exported entry stub runs on every AOT to AOT call and copies the stack passed arguments again;
  wide signatures make it as expensive as the callee.
- The ladder compiles with `--bounds-checks=1` (software bounds checks), so every load and store in the
  solver loops carries a compare and branch on top of rasc's own index checks.
- Native scalar is not scalar code: clang autovectorizes `simd_scalar.h` lanes (notes/scenes.md), so the
  fair long run target for the port is wasm SIMD behind `ASC_FEATURE_SIMD` (`wasmSimd` defaults to on).
- Per step allocation of the solver arrays costs 8 ms (solverSetup) and leaks or retains about 6 MB per
  step under the frame collector; reusing them across steps, as C's stack allocator does, fixes both.

## Profiling recipe

`bench/as/profile.sh` attaches xctrace's Time Profiler to the rive process once stepping starts and maps
samples back to wasm functions:

1. WAMR loads AOT code into an anonymous `rwx` mapping, so Instruments shows bare addresses. `vmmap` of the
   same process gives the mapping; code starts 4 bytes in (the text section's literal size word, 0 here).
2. `RIVE_RASC_NAMES=1` keeps the name section through the release bake (`as_pool.cpp`); the AOT cache
   (`RIVE_AOT_CACHE_DIR`) keeps the exact `.wasm` next to each `.aot`.
3. `wamrc --bounds-checks=1 --opt-level=3 --cpu=generic --cpu-features=+reserve-x18 --tune-cpu=apple-m1`
   (the ladder's flags, `module_tier_ladder.cpp`; no `--rive-interrupt` under `dangerouslyFast`)
   reproduces the cached `.aot` byte for byte, and `--format=object` with the same flags gives an ELF with
   `aot_func#N` / `aot_func_internal#N` symbols at the same text offsets.
4. `tools/xctrace_samples.py` flattens the xctrace export; `tools/aotsym.py` maps each leaf PC through the
   symbols and the name section (`--split` separates entry stubs). Backtraces are only two frames deep in
   AOT code, so self time is reliable and inclusive time is not.

The trace bundle records the whole environment of the shell, secrets included; `profile.sh` writes it to
a temp dir (or `OUT`), never into the repo.
