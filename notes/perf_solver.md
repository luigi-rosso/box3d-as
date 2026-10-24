# perf: the solve stage

Scope: `lib/simd.as`, `lib/contact_solver_wide.as`, `lib/solver.as` (step setup), with no change to
`contact_solver.as`, `solver_set.as`, `constraint_graph.as`, `island.as` or `body.as` in the end. The
collide task (`physics_world.as`, including `b3ResolveContactBodySim`), contacts, shapes and the tree
belong to the collide pass. `bench/as/run.sh` and `profile.sh` gained `WASM_SIMD=off`.

All of it keeps `KEEP_GOING=1 verify/run.sh` (28 modules) and `verify/traj.sh --large` (9 scenes) bit exact
with `RASC_FLAGS=-O` and with `RASC_FLAGS="-O --enable-simd"`, checked before each commit on a snapshot of
the worktree. The verify harness needs nothing new for SIMD: `RASC_FLAGS` reaches rasc in both scripts.

## Results

`bench/as/run.sh`, 3 rounds, Apple M5 Max, one worker. Before is notes/bench.md's session
`session_20261023_133219.log` (load 16 to 34), whose bake had no SIMD code, so it stands for both
artifacts. After are `session_20261023_151510.log` (SIMD artifact, the default `wasmSimd: on`) and
`session_20261023_152750.log` (`WASM_SIMD=off`), both on a machine loaded 40 to 290 by other agents, so
absolute numbers are inflated and noisy; `x scalar` divides by native scalar of the same session and is the
number to compare.

| scene | before AOT | x scalar | after AOT, SIMD | x scalar | after AOT, scalar artifact | x scalar |
|---|---|---|---|---|---|---|
| large_pyramid | 103.48 | 6.1 | 27.17 | 1.3 | 36.16 | 1.9 |
| many_bodies | 194.70 | 5.6 | 46.73 | 1.1 | 68.25 | 2.4 (1.7 to 2.4 per round) |
| pyramid | 0.868 | 6.9 | 0.234 | 1.6 | 0.271 | 2.6 |
| pyramid, interpreter | 8.04 | 63.6 | 4.74 | 31.9 | 3.12 | 30.5 |

The interpreter rows after run the scalar artifact (finding below). Native SIMD in the SIMD session:
large_pyramid 12.83, many_bodies 27.98, pyramid 0.089 ms, so the SIMD artifact is 2.1x, 1.7x and 2.6x
native SIMD.

Per stage, large_pyramid, ms per step (native columns from the same session as the AS column):

| stage | before AS | native scalar | after AS SIMD | native SIMD | native scalar | after AS scalar artifact | native scalar |
|---|---|---|---|---|---|---|---|
| step | 103.478 | 16.831 | 27.169 | 12.825 | 21.623 | 36.157 | 19.488 |
| collide (not this pass) | 8.919 | 2.700 | 10.659 | 3.351 | 3.692 | 9.071 | 3.144 |
| solve | 93.832 | 13.639 | 15.619 | 9.040 | 17.305 | 26.401 | 15.494 |
| solverSetup | 8.078 | 0.002 | 0.013 | 0.002 | 0.002 | 0.012 | 0.002 |
| prepareConstraints | 9.678 | 2.704 | 3.724 | 2.386 | 3.660 | 5.232 | 3.120 |
| integrateVelocities | 0.252 | 0.140 | 0.268 | 0.164 | 0.184 | 0.211 | 0.154 |
| warmStart | 13.768 | 1.661 | 1.327 | 0.740 | 2.101 | 2.699 | 1.859 |
| solveImpulses (push) | 24.558 | 2.943 | 2.398 | 1.665 | 3.669 | 6.173 | 3.224 |
| integratePositions | 0.416 | 0.245 | 0.446 | 0.283 | 0.301 | 0.388 | 0.276 |
| relaxImpulses (solve) | 33.350 | 4.736 | 3.537 | 2.497 | 5.929 | 8.606 | 5.268 |
| storeImpulses | 2.503 | 0.707 | 2.168 | 0.823 | 0.870 | 1.997 | 0.782 |
| transforms (finalize) | 1.180 | 0.419 | 1.321 | 0.455 | 0.542 | 1.042 | 0.470 |

The solve stage went from 6.9x native scalar to 0.9x with SIMD (1.7x native SIMD) and 1.7x on the scalar
artifact. In a quieter moment (`profile.sh`, load about 90) the SIMD large_pyramid ran 18.9 ms per step
with solve 9.7 ms, against native SIMD's 8.7 to 9.0 ms solve. many_bodies' SIMD solve is 28.6 ms against
native scalar 35.3 and native SIMD 20.8.

What the solve stage costs now, by kernel (the profile at load about 90, large_pyramid SIMD, share of all
samples): prepare 15.2%, solve 13.0%, push 8.6%, warm start 5.6%, finalize bodies closure 6.4%, the block
dispatcher with the integrations 4.6%, store impulses 3.9%. The rest of the top is the collide task:
`b3ResolveContactBodySim` alone is 23.6% and the collide closure 10.8%.

## Changes, in commit order

### 1. Grow only step buffers (solverSetup 8 ms to 0.01 ms, heap growth gone)

`b3Solve` allocated every per step array (`wideConstraints` and `wideManifolds`, about 6.5 MB for
large_pyramid, the mesh arrays, the overflow constraints, stages, blocks, spans, worker contexts, the
bullet list) where C bumps its stack allocator. They live one step on the colors and the step context, so
they survive a frame boundary, get promoted, and the old generation grew about 6 MB per step (20315 pages
after 200 large_pyramid steps). A module `b3SolverScratch` in solver.as now owns them, grow only
(`count + count / 4 + 16`), shared by every world since only one `b3Solve` runs at a time. C's stack
memory is not zeroed and every reader stays inside the counts the step writes, so stale contents past the
counts or from an earlier step are never read: the parity checks confirm it, including the four world
solver driver, which steps worlds of different sizes through the same scratch. The scratch keeps its high
water mark after a world is destroyed. The mesh prepare span keeps
the lane order array plus a base index instead of a `subarray` view. `b3StepContext`'s default arrays are
shared sentinels.

large_pyramid, load 15 to 17, one interleaved pair: solverSetup 8.18 to 0.011 ms, step 81.8 to 74.9 ms;
1079 wasm pages after 110 frames instead of 10588. After the rest of the changes the heap is 1071 pages
after 20 steps and 1071 after 200: no growth per step, no accumulating references warning, and the
multi hundred ms outlier steps are gone (max is now about twice the median, the machine noise).

### 2. v128 kernels for prepare, warm start, push and solve, read in place

Under `ASC_FEATURE_SIMD` each `b3FloatW4` held in registers is one `v128`; the wide constraints stay the
same structs in memory, and the kernels read and write their fields with `v128.load` / `v128.store` at
`offsetof` addresses, as C's SSE2 build reads them through pointers. Body states gather as C's SSE2
`b3GatherBodies`: the 64 byte state is four rows, loaded from the raw array data and transposed with
shuffles; the scatter transposes v and w back and stores two rows per dynamic body (the zero w lane lands
in the padding, which nothing reads). Every lane op is the exact form of notes/containers.md: min/max/abs
and blends are compare plus bitselect, masks stay all ones lanes inside a kernel and are stored as
`and(mask, 1.0)` where the scalar path stores 1.0, and read back with `ne(mask, 0)`. Vector helpers that
would return three values (cross, matrix times vector, inverse rotate) are written per component
(`b3CrossL(a1, a2, b1, b2)`, `b3MulRowL`), so every helper returns one value and rasc -O inlines it.

large_pyramid, load 80 to 150 (so only the ratios mean anything): warm start 21.6 to 2.3, push 38.8 to 3.5,
relax 48.9 to 5.3 ms in the first pair; prepare 15.0 to 6.3 ms once it went wide too. The solve kernel's
body went from 5741 to 1625 arm64 instructions, loads from 2260 to 238 and stores from 1482 to 76.

### 3. Store impulses through raw lane loads

`b3StoreImpulsesW` reads `( (const float*)&w )[lane]` as C does, a `load<f32>` at the field plus 4 lane,
and writes the manifold through its address, instead of `b3LaneW`'s branch chain and a bounds checked
`Array` place per field. The same code serves both artifacts. Measured 5.1 to 3.5 ms under heavy load.

### 4. One kernel source for both artifacts (scalar artifact 2.9x faster)

v128 cannot appear in a declaration of the scalar artifact, and a struct cannot be a generic argument, so
the kernels became generic over a lane type `T` that is `v128` under SIMD and `f32` without it. The `L`
helpers in `lib/simd.as` pick their body with `isFloat<T>()`, which rasc folds per instance and whose dead
branch never compiles. An `f32` kernel runs once per lane with every wide field address offset by 4 lane
bytes (the `FixedArray<i32, 4>` index lanes line up the same way), and reads only its lane's contact,
sims and body states. Lanes of one wide constraint touch distinct dynamic bodies (graph coloring; static
and kinematic bodies are never written), so running them one after the other gives the bits C's scalar
build gives, and the parity checks agree. Decisions C takes across lanes (`b3AllZeroW` of the rolling
resistance or restitution, the restitution `b3AnyTrueW`) read all four lanes from memory or the contacts
before the lane loop, so every lane takes the same branch C takes. The old struct based kernels and their
multi value helpers (`b3GatherBodies`, `b3MulMVW`, `b3InvRotateVectorW`, ...) are gone; the restitution
kernel went generic too. The dispatch is an `if (ASC_FEATURE_SIMD)`, not a ternary, since rasc compiles
both arms of a constant ternary.

In the scalar artifact this removes every out of line helper call, entry stub and multi value return
through memory, and keeps one lane's 26 body values in registers. large_pyramid scalar artifact, load 85
to 140: 202.6 to 69.3 ms per step (relax 59.5 to 17.6, push 45.3 to 12.0, warm 25.7 to 5.7, prepare 19.0 to
10.8). The interpreter (pyramid, scalar artifact) went from 9.7 to 4.1 ms per step.

### 5. Inertia loaded once per constraint, empty f32 lanes skipped

The six inverse inertia lanes per body load once per wide constraint instead of at each use. An f32 kernel
skips a lane whose contact is null: such a lane writes only its own constraint lanes, which nothing reads
(store impulses skips null contacts, and the shared decisions read values prepare writes for every lane).
Prepare never skips, since those decisions read its stores. No measurable change on the SIMD artifact; kept
because it is less code per use.

### 6. Prepare reads contacts and manifolds through raw addresses

C keeps `b3Contact*` and `b3Manifold*` lane arrays. The port stored contact and manifold array references
into module scratch arrays, each store a `__linkSlot` write barrier, and indexed `StaticArray<b3Manifold>`
per lane, each index check a divide by 268. They are now `FixedArray<usize, 4>` of object and manifold
addresses, read with `load<f32>` at `offsetof` fields; nothing collects or moves during a step. Prepare
many_bodies 9.0 to 8.1 ms (min of 3, load 60 to 140).

## Raw memory and why it is in range

- Body state and sim indices come from `b3PrepareConstraintL`, which takes them from the contacts' encoded
  awake indices; 0 (or `B3_NULL_INDEX` for sims) selects `b3_identityStates` / `b3_zeroSims`.
- Constraint `ci` and manifold `mi + k` addresses stay below the counts `b3Solve` sized the scratch with.
- Lane offsets are below 16 bytes within a 16 byte wide field.
- Manifold `k < contact.manifoldCount` of `contact.manifolds`, points below `B3_MAX_MANIFOLD_POINTS`.
- `Array<V>` data is read as `load<usize>(array + offsetof<Array<i32>>("dataStart"))`, since rasc refuses
  `dataStart` on value class arrays (notes/findings.md).

## What did not help

- Hoisting the inverse inertia (change 5) did not move the SIMD numbers; LLVM already kept most of them.
- Instruction level profiles of prepare, finalize and store impulses are flat (no instruction above about
  1% of the function's samples): what is left there is the per access bounds checks and the copies, not
  one hot spot.

## Findings (in notes/findings.md)

- WAMR 2.4.5's fast interpreter traps (`invalid local type`) on `local.set` / `local.tee` of a v128 local
  past the first 256 frame cells, so the SIMD artifact cannot run on the interpreter tier: the bench now
  runs its interpreter scene on the scalar artifact, and a project that may fall back to the interpreter
  needs `wasmSimd: off` until WAMR is fixed. This is the one decision to make before shipping the default
  `wasmSimd: on` bake of this library.
- `@inline` does not reach binaryen (no `metadata.code.inline` hint).
- `dataStart` is refused on value class arrays.
- `isFloat<T>()` folds per instance but is not in the docs; a constant ternary compiles both arms.

## What is left

- The collide task is now most of the gap: `b3ResolveContactBodySim` (physics_world.as) copies a
  252 B sim twice per contact and is 23.6% of the AOT samples by itself; it belongs to the collide pass.
- Store impulses (2.6x native SIMD), finalize bodies (2.9x) and prepare (1.6x) in this scope. Their
  profiles are flat: bounds checks on every access (the ladder's `--bounds-checks=1`), the per body copies
  of the 252 B sim and 64 B state in `b3FinalizeBodiesTask` and the integrations (C uses pointers), and
  the `Array<b3Contact>` / `contact.manifolds` hops per lane. Raw addresses as in prepare would remove most
  of the copies at some cost in readability.
- `b3MulMM` and the other struct returning math helpers in finalize stay calls (math_functions.as).
- The iteration kernels are 1.4x to 1.8x native SIMD; what remains there is the software bounds check on
  every `v128.load` (one compare and branch per 16 bytes) and some spills around the 26 live body lanes.
- Joints and the overflow color do not run in these scenes and were not touched; their prepare and solve
  paths still copy joint sims and states by value.
- Before shipping `wasmSimd: on` for this library, WAMR's interpreter needs the v128 local fix, or every
  host that may run the interpreter tier needs `wasmSimd: off`.

# Round 2

Commits 08300d9 to b4898b6. Gates as in round 1, now 10 trajectory scenes (`plank_pile` added) in both
artifacts, plus the bench's state hash (below) on every tier: SIMD AOT, SIMD interpreter, scalar AOT and
scalar interpreter all print the native recording's hash for pyramid (200 steps), large_pyramid (200; 20 on
the interpreter), many_bodies (100), chains (300) and plank_pile (300; 50 on the interpreter).

## Measurement

The machine was quiet for the final sessions (load 8 to 30), unlike round 1. `bench/as/out/session_*`:
SIMD artifact with native interleaved, 3 rounds; scalar artifact, 2 rounds. The bench now also prints the
per stage minimum over the timed steps (`min_<stage>`), which a loaded machine disturbs least, and
`hash=`: FNV-1a over every body's pose and velocity bits, which `verify/scenes/hash.mjs <recording>
<steps>` computes from a native recording.

| scene | native SIMD | native scalar | AS SIMD artifact | x native SIMD | AS scalar artifact | x native scalar |
|---|---|---|---|---|---|---|
| large_pyramid | 4.371 | 7.169 | 11.477 | 2.6 | 22.70 | 2.2 |
| many_bodies | 12.974 | 19.092 | 20.859 | 1.6 | 47.62 | 1.8 |
| pyramid | 0.042 | 0.077 | 0.100 | 2.4 | 0.255 | 2.9 |
| chains | 0.071 | 0.073 | 0.101 | 1.4 | 0.180 | 1.8 |
| plank_pile | 0.892 | 0.962 | 1.698 | 1.9 | 2.486 | 1.9 |
| pyramid, interpreter | | 0.077 | 1.489 | | 3.312 | |

(The scalar artifact columns come from the second session, whose native scalar was 10.35 and 27.11 ms on
the two big scenes; the ratio uses that session.)

large_pyramid by stage, SIMD session, ms per step:

| stage | native SIMD | native scalar | AS SIMD | x native SIMD |
|---|---|---|---|---|
| collide (not this pass) | 0.666 | 0.741 | 4.503 | 6.8 |
| solve | 3.474 | 6.207 | 6.551 | 1.9 |
| prepareConstraints | 0.559 | 0.862 | 1.667 | 3.0 |
| integrateVelocities | 0.068 | 0.068 | 0.118 | 1.7 |
| warmStart | 0.359 | 0.705 | 0.534 | 1.5 |
| solveImpulses (push) | 0.753 | 1.589 | 1.154 | 1.5 |
| integratePositions | 0.107 | 0.102 | 0.170 | 1.6 |
| relaxImpulses (solve) | 1.096 | 2.319 | 1.673 | 1.5 |
| storeImpulses | 0.303 | 0.343 | 0.864 | 2.9 |
| transforms (finalize) | 0.207 | 0.206 | 0.340 | 1.6 |

On a quiet machine the gaps are wider than round 1's loaded sessions showed: native's caches and memory
bandwidth were the contended resource there. The SIMD solve stage is 1.9x native SIMD and about native
scalar; collide is now 54% of the AS step's excess over native SIMD.

## 1. The SIMD artifact on WAMR's fast interpreter

`bench/as/tools/v128cells.py <bake.wasm>` lists each function's frame offset of its last v128 local (binaryen
writes v128 locals after the scalar ones; WAMR 2.4.5 traps at 256, notes/findings.md). Before and after, in
the release bake:

| kernel | before | after |
|---|---|---|
| b3SolveContactsL<v128> | 310 (73 v128) | 238 (53) |
| b3PushContactsL<v128> | 306 (72) | 237 (53) |
| b3ApplyRestitutionL<v128> | 283 (66) | 196 (42) |
| b3PrepareConstraintL<v128> | 273 (54) | 225 (43) |
| b3WarmStartContactsL<v128> | 139 (30) | 139 (30) |

What moved them: the inverse inertia lanes load at each use again (undoing round 1's change 5, which had not
helped); delta position and rotation gather inside the manifold loop instead of living across it (convex
constraints have one manifold, so the loads are the same count); splatted constants are made at their use
(LLVM hoists them under AOT); in prepare `invIA + invIB` is recomputed where used and the tangent velocity
lanes are set per manifold. No function was split. `bench/as/run.sh` runs the cell check after a SIMD session
and warns from 240.

AOT before and after, large_pyramid, 3 interleaved rounds (load 30 to 100): step 23.54 and 23.66 ms (min of
means); stage minima within 5% (prepare 3.44 and 3.61, relax 2.82 and 2.98, push 2.10 and 2.08), which is
the noise. The interpreter now runs the SIMD artifact bit exact: pyramid 1.49 ms per step against the scalar
artifact's 3.31 on the interpreter.

## 2. Pointer walks for the remaining stages

- `b3FinalizeBodiesTask`, `b3IntegrateVelocitiesTask` and `b3IntegratePositionsTask` reach states, sims,
  bodies and move events through addresses (`b3ArrayData`, body.as) and the `FS_` / `FSIM_` / `FB_` field
  offsets, reading only the fields C reads and writing through as C's pointers do, so `b3SolveContinuous`
  sees the stores without the store and reload around it. `invInertiaWorld` is `R I Rᵀ` written out per
  element in `b3MulMM`'s order (`b3StoreWorldInertia`), so no `b3MulMM` / `b3Transpose` calls remain.
- Store impulses and prepare read lane contacts through `world.contacts`' data and `offsetof<b3Contact>`
  fields; store impulses loads the four lanes' contact and manifold addresses up front, as C does.
- Joints (all eight types) and the overflow contact kernels take states through `b3StateAddress` (the
  identity state for a static end) and store velocities with `b3StoreStateVelocities`, instead of a struct
  ternary over `b3_identityBodyState` and checked `Array` places.

Per stage minima, large_pyramid, before (3b0092d) and after (d639beb), 3 rounds: transforms 0.86 to 0.49,
store impulses 1.29 to 1.01, prepare 2.32 to 2.16, integrate velocities 0.18 to 0.17, positions 0.28 to
0.24, solve stage 10.17 to 9.63. Joints and overflow (e54c7d9), plank_pile: solve minimum 0.707 to 0.681,
relax 0.312 to 0.294; chains within noise.

## 3. Joints and the overflow color: chains and plank_pile

`plank_pile` (verify/scenes/scenes.c and the scenes.as mirror) is two layers of 16 x 16 boxes, each under a
dynamic plank touching all 256, plus a hub with 40 distance joints: 748 constraints in the overflow color
every step. Its native recording comes from `NATIVE=1 verify/traj.sh plank_pile` (recordings are not
committed). The AS bench builds chains (`SCENE=3`) and plank_pile (`SCENE=4`) from the same scene mirror,
which `run.sh` copies into `bench/as/scenes/`. They are 1.4x and 1.9x native SIMD (table above).
plank_pile's AS solve is 1.39 ms against 0.85 native: overflow push and relax 0.41 and 0.55 against 0.29 and
0.40, prepare 0.14 against 0.06. In the profile the overflow kernels are inlined into `b3SolverTask` (26%
of plank_pile's samples, scalar code, 442 index checks in 6584 instructions) and the joint solves into
`b3SolveJoint` (12297 instructions, all helpers inlined); what is left there is per access checking and
scalar math, not calls or copies. Collide is plank_pile's biggest gap (0.30 against 0.04 ms).

## 4. What if: WAMR hardware bounds checks

`RIVE_OUT=out/release_hw ./build.sh release --wasm_hw_bounds` in packages/rive-cli of ~/Projects/rive-master
is a build flag only (it defines `RIVE_WASM_HW_BOUNDS`; the ladder then compiles a `.hw.aot` with
`--bounds-checks=0 --stack-bounds-checks=1` and runs it under the guard page handler). The binary is at
`packages/rive-cli/out/release_hw/rive`, from the same source as `out/release`. large_pyramid, same lib, bit
exact (hash checked):

| run | sw step | hw step | note |
|---|---|---|---|
| 3 rounds, load 95 to 110 | 19.56 (min step) | 15.27 | hw 22% faster, every stage faster (prepare 36%) |
| 3 rounds, load 18 to 25 | 13.43 (min of means) | 13.57 | equal |
| 4 rounds, load 16 to 20 | 9.23 (min step), 11.57 mean | 10.47, 11.84 | sw no slower; medians 12.84 and 11.07 |

So on a quiet M5 Max the software bounds checks cost nothing measurable in this workload (predicted
branches beside memory bound loops), while under heavy contention removing them bought about 20%. The
iteration kernels' 1.5x against native SIMD is not the bounds checks.

## What is left after round 2

- Collide: 4.5 against 0.67 ms on large_pyramid and 0.30 against 0.04 on plank_pile; `b3ResolveContactBodySim`
  and the collide closure top the profiles. The collide pass's.
- Prepare (3.0x) and store impulses (2.9x): both walk four scattered contacts and their manifold arrays per
  wide constraint; the port's contacts are separate objects with separately allocated manifold arrays, where
  C keeps them in one array and block allocator. A data layout change in contact.as would be the next step.
- Iteration kernels 1.5x native SIMD with bounds checks ruled out: spills around the 26 live body lanes and
  the per lane gather pointer selects are what remains.
- Joints and overflow: about 1.5x native per kernel; scalar math as C, through checked joint sim fields.
