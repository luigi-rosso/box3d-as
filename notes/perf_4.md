# perf 4: single core against C on V8, and against native on WAMR

Scope: the solver kernels, the tree, the joints and the queries, moved onto the `parallel-calls` toolchain
(~/Projects/rive-parallel-calls at a4fa1239d4, master plus parallel calls after #15091). Commits ca737e1 to 4306b8e on
`single-core`, from 95ea784. Every change kept `KEEP_GOING=1 verify/run.sh` (29 modules) and `verify/traj.sh --large`
(13 scenes) bit exact with `RASC_FLAGS=-O` and `RASC_FLAGS="-O --enable-simd"`, and `verify/threads.sh --large` with
`BACKENDS=3 PARTS="1 4"` (26 runs) bit exact, gated on a snapshot of the tree before each commit. Every bench run below
matched the native recording's state hash.

Toolchain: rasc and std from the worktree, rive-cli `out/release/rive` and its wamrc
(`~/Projects/rive-parallel-calls/wamrc-build/wamrc`), binaryen 133 for the web bake. The web bake is now a release bake
(`--release`, bench/web/build.sh), as rive-cli's `--bench` is. "Before" below is 95ea784 built with the same toolchain in
the same session, so the tables isolate this pass's source changes; the old toolchain's numbers are in notes/web.md.

## Results

Machine: Apple M5 Max (6 + 12 cores), shared with other agents all session; the one minute load is given per table.

### V8 against C wasm

node 26.3.1 (V8 14.6), bench/web/bench.mjs, 5 rounds interleaved over 8 builds (320 runs, load 7.5 to 11.1), min over
rounds of each run's median ms per step (log `bench/web/out/bench_20261026T214959.jsonl`):

| scene | native NEON | C wasm SIMD | AS SIMD before | AS SIMD after | AS / C before | AS / C after | C scalar | AS scalar before | AS scalar after | AS / C scalar before | after |
|---|---|---|---|---|---|---|---|---|---|---|---|
| large_pyramid | 3.653 | 4.149 | 4.997 | 4.375 | 1.20 | 1.05 | 10.239 | 9.407 | 9.103 | 0.92 | 0.89 |
| many_bodies | 7.802 | 9.310 | 11.107 | 9.854 | 1.19 | 1.06 | 21.121 | 19.881 | 18.976 | 0.94 | 0.90 |
| mesh_drop | 6.011 | 6.543 | 8.147 | 7.682 | 1.25 | 1.17 | 11.550 | 12.943 | 12.634 | 1.12 | 1.09 |
| village | 2.755 | 3.278 | 4.040 | 3.755 | 1.23 | 1.15 | 5.124 | 5.514 | 5.251 | 1.08 | 1.02 |
| plank_pile | 0.669 | 0.737 | 0.759 | 0.727 | 1.03 | 0.99 | 0.815 | 0.822 | 0.816 | 1.01 | 1.00 |
| chains | 0.0183 | 0.0251 | 0.0274 | 0.0278 | 1.09 | 1.11 | 0.0235 | 0.0283 | 0.0274 | 1.21 | 1.17 |
| pyramid | 0.0299 | 0.0371 | 0.0459 | 0.0385 | 1.24 | 1.04 | 0.101 | 0.0851 | 0.0811 | 0.84 | 0.80 |
| queries | 0.122 | 0.151 | 0.175 | 0.168 | 1.16 | 1.11 | 0.155 | 0.178 | 0.169 | 1.14 | 1.09 |

Per stage, AS / C SIMD, before to after (stage means of each build's best run):

| stage | large_pyramid | many_bodies | mesh_drop | village |
|---|---|---|---|---|
| prepare | 1.40 to 1.17 | 1.35 to 1.17 | 1.50 to 1.37 | 1.46 to 1.25 |
| warm start | 1.15 to 1.02 | 1.08 to 0.97 | 1.11 to 0.99 | 1.27 to 1.12 |
| solve (biased) | 1.29 to 1.00 | 1.29 to 1.00 | 1.23 to 1.07 | 1.31 to 1.03 |
| relax | 1.16 to 0.94 | 1.15 to 0.94 | 1.06 to 0.96 | 1.19 to 0.96 |
| store impulses | 1.42 to 1.29 | 1.41 to 1.29 | 1.61 to 1.46 | 1.53 to 1.24 |
| integrate velocities | 1.20 to 1.20 | 1.15 to 1.16 | 1.07 to 1.09 | 1.10 to 1.11 |
| pairs | 1.17 to 1.17 | 0.88 to 0.86 | 1.24 to 1.24 | 1.22 to 1.21 |
| collide | 1.07 to 1.07 | 1.11 to 1.11 | 1.23 to 1.23 | 1.29 to 1.29 |
| finalize and continuous | 0.97 to 0.97 | 1.02 to 1.01 | 1.09 to 1.10 | 1.05 to 1.00 |

- **The iteration kernels now run at C's speed or faster on V8**: solve 1.00 to 1.07x, relax 0.94 to 0.96x, warm start
  0.97 to 1.12x. The solver heavy scenes went from 1.19 to 1.25x C to 1.04 to 1.06x.
- **Prepare and store impulses** closed by about half of their gap; collide and pairs did not move (below).
- The scalar artifact gained 3 to 5% on the solver scenes and stays ahead of C's scalar build there.

V8's own machine code (`node --print-wasm-code --no-liftoff`, arm64 instructions per kernel; prepare and store with
`--no-wasm-inlining` so they stay separate):

| kernel | C | AS before | AS after |
|---|---|---|---|
| solve | 1120 | 1598 | 1350 |
| push | 780 | 1205 | 967 |
| warm start | 620 | 940 | 790 |
| prepare (C's includes its block loop) | 1838 | 2362 | 1831 |
| store impulses (after: the four lanes unrolled) | 502 | 670 | 1556 |

### WAMR AOT against native

rive-cli `--bench` (release), `--aot`, RIVE_WASM_AOT_SYNC=o3, the SIMD artifact, interleaved with native NEON
(bench/web/run.mjs native_simd, the same driver), 3 rounds, min over rounds of the median (scratchpad `aotab.mjs`).
Software bounds checks (the shipping build), load 7.2 to 14.4:

| scene | native | AS before | AS after | before / native | after / native |
|---|---|---|---|---|---|
| large_pyramid | 3.583 | 4.806 | 4.700 | 1.34 | 1.31 |
| many_bodies | 8.280 | 10.623 | 10.661 | 1.28 | 1.29 |
| mesh_drop | 5.034 | 6.172 | 6.137 | 1.23 | 1.22 |
| village | 2.634 | 3.970 | 3.465 | 1.51 | 1.32 |
| plank_pile | 0.670 | 0.851 | 0.672 | 1.27 | 1.00 |
| chains | 0.0238 | 0.0515 | 0.0382 | 2.17 | 1.61 |
| pyramid | 0.0292 | 0.0501 | 0.0396 | 1.72 | 1.36 |
| queries | 0.116 | 0.207 | 0.161 | 1.78 | 1.39 |

The small scenes' before column ran in the loudest part of the session (an earlier session measured base chains at
0.0418, pyramid 0.0406, queries 0.164 at load 6 to 11), so read the small scenes' gains as about half of what the table
shows. The big scenes moved by 0 to 3%: on WAMR the kernels were not where the gap was (below).

Hardware bounds (a CLI built with `--wasm_hw_bounds` into `out/release_hw`, no source change; the guard page fix of
patch 0026 is on this branch, so out of bounds accesses trap), same method, load 7.0 to 10.3:

| scene | native | AS before | AS after | before / native | after / native |
|---|---|---|---|---|---|
| large_pyramid | 3.921 | 4.975 | 4.696 | 1.27 | 1.20 |
| many_bodies | 8.317 | 10.571 | 10.143 | 1.27 | 1.22 |
| mesh_drop | 5.111 | 5.966 | 6.163 | 1.17 | 1.21 |
| village | 2.673 | 3.447 | 3.415 | 1.29 | 1.28 |
| plank_pile | 0.671 | 0.701 | 0.807 | 1.05 | 1.20 |
| chains | 0.0179 | 0.0541 | 0.0389 | 3.02 | 2.17 |
| pyramid | 0.0295 | 0.0380 | 0.0363 | 1.29 | 1.23 |
| queries | 0.116 | 0.151 | 0.144 | 1.30 | 1.24 |

plank_pile and mesh_drop moved the wrong way between rounds of this session by more than any change could explain
(plank_pile's after was 0.672 in the software bounds session); the rest agree with the static counts. Hardware bounds
are still not shippable as a default (the Pre-release Study's list: signal chaining with crash reporters, the two
failing test groups, Windows, iOS).

Static WAMR AOT instructions per kernel (`wamrc --format=object`, the ladder's flags, software bounds):

| kernel | before | after |
|---|---|---|
| solve | 1595 | 1540 |
| push | 1068 | 1011 |
| warm start | 938 | 871 |
| prepare | 2397 | 2292 |
| store impulses (after: unrolled) | 474 | 1041 |
| tree partition | 432 | 287 |
| mesh manifolds | 12673 | 12052 |
| joint solve | 10585, every joint type inlined | 71 plus a call to the type's function (spherical 1624) |

## Changes, in commit order

### 1. Transposes as C's unpack zips (ca737e1)

Each gathered column was its own transpose, `shuffle(shuffle(r0, r1, 0, 4, 0, 4), shuffle(r2, r3, 0, 4, 0, 4), 0, 1,
4, 5)`: three shuffles per column, nine for a vector, and V8's arm64 backend has no instruction for `0, 4, 0, 4`, so it
moves lanes one at a time (296 lane moves in solve against C's 50). The body gathers and scatters now transpose a whole
row at once as SSE2's `b3TransposeW4` does, unpacklo and unpackhi of row pairs (`b3TransposeL`, `b3GatherRowL`,
`b3GatherRow3L` in lib/contact_solver_wide.as and lib/simd.as), so every shuffle is one zip: solve, push and warm start
have 90, 90 and 60 shuffles, C's counts exactly, and the lane moves are gone. The single column helpers use the same
zips.

V8, 3 interleaved rounds (load 6 to 8): large_pyramid 4.52 to 4.12 ms per step, solve 0.894 to 0.692, relax 1.193 to
0.982 (C 1.056), warm start 0.357 to 0.316 (C 0.314). WAMR: no change in the kernels (LLVM already picks good code for
the old shuffles).

### 2. Contact addresses in the constraint, lanes unrolled (5f32181)

The wide constraint held contact ids, so prepare and store impulses looked each one up in `world.contacts`, and prepare
kept its per lane contacts, manifolds, sims, counts and point counts in `FixedArray` locals indexed by the lane loop.
rasc compiles a variable index into a fixed array local as a range check and a `br_table` over the elements
(findings.md): prepare had 164 conditional branches in V8's code. Now:

- the constraint stores each lane's contact address (0 for C's NULL), which holds for the step, as C stores the
  pointer; the callers set them before prepare;
- prepare reads its lanes back from the constraint (contacts, base 1 body indices) with raw loads and picks a lane's
  manifold with selects (`b3LaneManifold`, `b3PickLane`), with no lane arrays;
- store impulses runs the four lanes unrolled through an `@inline` helper (`b3StoreLaneImpulses`), so their contacts
  and manifold pointers stay in locals.

V8 (load 6 to 8): prepare 0.529 to 0.456 ms on large_pyramid, store impulses 0.295 to 0.257; prepare 2362 to 1862
instructions, conditional branches 164 to 49.

### 3. Prepare as C gathers (04099ae)

Lane loads (`b3SetLoadL`: one `v128.load32_lane` per lane) for the floats C gathers with `b3SetW` of loads (inverse
masses, the inertia's czz, the rolling impulse's y and z); the second inertia row read from cy.y as C's transpose reads
it (four shuffles instead of six); friction, restitution and rolling resistance as one row; the inverse inertia sum
computed once; the tangents kept live instead of reloaded (the reload kept the frame under the interpreter's old cell
limit). On V8 the lane loads are neutral (1862 to 1860 instructions: a lane load needs its address in a register); the
scalar artifact loses its dead per lane loads (prepare 132 to 108 loads).

### 4. Workarounds the toolchain made unnecessary (89a1d36)

- `b3PrefilterTriangles` has one call site and `@noinline` instead of a second call site.
- `b3BarycentricCoordsTri` is a helper again, called at its five sites as in C; the release bake inlines it and
  `b3ShapeDistance` comes out identical (7885 ops, no calls).
- The triangle feature tables are array literals again, which parallel code may now read (b89a090a2f).
- The collide task computes `b3InvMulWorldTransforms` whole, as C, at both sites (the closure is identical).
- bench/as/run.sh no longer runs the v128 cell check: the interpreter limit is fixed (patch 0030), checked with the
  80 local repro of findings.md on this CLI.

### 5. Solver stage times on the parallel path (8ac26a3)

`b3ParallelSolverTask` fills prepareConstraints to storeImpulses on part 0, at the points C's main thread reads its
timer, now that parallel code may read the clock (dc5083bc5a). A BACKEND=3 run at 4 parts reports every stage.

### 6. Joints as C splits them (0e72a7e)

`b3SolvePointConstraint` is `@inline` (C's `static inline`); every joint type's prepare, warm start and solve is
`@noinline`, as C keeps them in their own functions, where the release bake had inlined all of them into one 21765 op
`b3SolveJoint` with 537 locals. On V8 chains is unchanged (0.0276 to 0.0277 ms, C 0.0266); on WAMR the joint solve went
from 10585 to 1624 instructions for the spherical joint chains use, and chains is faster in both WAMR sessions.

### 7. Nested tree query stacks (8ca7f48)

A tree query nested in a query callback (a ray reaching a mesh's own tree, an overlap on a compound) allocated a new
4 KB stack per call: `__alloc` and `__link` were 6% of the queries scene's V8 profile. Each nesting depth now keeps one
stack. Overlap queries 0.0126 to 0.0117 ms per step on V8.

### 8. Unchecked where proven (28147a3, 4306b8e)

- `b3ComputeMeshManifoldsAt` runs unchecked again. perf_collide.md change 7 proved its indices in range and put
  `@unchecked` on the function; the parallel split (57c281f) left the decorator on the thin wrapper, and a decorator does
  not reach a function it calls. 406 to 311 trap sites (the rest are the deliberately checked helpers for shape supplied
  data and the resizable `Array` struct accesses, which keep their checks by design).
- `b3PartitionMid` and its only helper `b3SwapLeaves`: every index is in [base, base + count), a range `b3BuildTree`
  keeps inside the rebuild's leaf arrays (which `b3DynamicTree_ReserveRebuild` sizes to the proxy count). 35 to 0
  traps, 1095 to 658 ops, WAMR 432 to 287 instructions.

### 9. Vector all zero and any positive tests (f1cef97)

`b3AllZeroAt` and `b3AnyPositiveAt` were four scalar loads and a chain of branches; under SIMD they are one load, a
compare and `all_true` / `any_true`. Warm start 803 to 790 instructions on V8.

## What did not help

- **`--low-memory-unused`** (plan item 2): the toolchain change it needs is a finding (findings.md). Prototyped
  outside the worktree (rasc with static data at 1024, the flag on the external post pass): bit exact, address adds
  190 to 32 in solve, and no change on V8 (every stage within 2%). WAMR's static counts drop 10 to 20% per kernel and
  19% in `b3ShapeDistance`, mostly merged bounds checks; the run time effect was not measurable without a rive-cli
  change. Restructuring the kernels to put offsets in the load immediate by hand is not possible through the shared
  helpers (a `load` offset must be a compile time constant, and an inlined parameter is not one), and V8 showed the adds
  cost nothing.
- **Gathering the delta position and rotation before the manifold loop**, as C's `b3GatherBodies` does: V8's v128 spills
  in solve went up (126 to 171) and the kernel grew (1363 to 1372 instructions). Reverted.
- **Lane loads on V8**: neutral (change 3); kept for the scalar artifact and for parity with C's code.
- **Index checks in the kernels** (plan item 3): there were almost none. The 15 to 35 `if`s per kernel the web notes
  counted are `?:` selects (null body pointers, fixed manifold counts) and short circuit lane tests, not index checks;
  each kernel has one checked read (`manifoldStarts[block.startIndex]`).

## What is left, with evidence

- **Store impulses, 1.24 to 1.46x C on V8, 1.5 to 2.1x native on WAMR**: the code matches C's line for line now; what
  differs is memory, each lane's manifold sits in its contact's own `StaticArray` (perf_3 measured the layout and found
  both memory bound to the same degree), and on WAMR each scalar store carries a software bounds check (hardware bounds:
  large_pyramid 0.334 ms against native 0.257, 1.3x).
- **Prepare, 1.17 to 1.37x C on V8**: V8's code is now the size of C's (1831 against 1838), so the rest is in the per
  lane scalar setup (sims, manifolds, point counts read per lane) and the hit event velocity pass, which C does the same
  way but clang unrolls.
- **Collide, 1.07 to 1.29x C on V8, 1.5 to 1.7x native on WAMR**: unchanged by this pass. A village profile has the
  collide closure itself at C's cost (961 against 958 ms of samples with `b3UpdateContact`), `b3UpdateConvexContact` at
  1.55x and the tree query in pairs at 1.23x. Contacts, manifolds and shapes are objects reached through references
  (perf_collide.md); `b3UpdateConvexContact` has 147 trap sites, most of them reads of hull data indexed by hull supplied
  values, which perf_collide.md keeps checked on purpose.
- **Pairs, 1.17 to 1.24x C on V8**: the tree rebuild (`b3DynamicTree_RebuildAt` 1.6x C's samples on mesh_drop) and the
  query callbacks; the partition is now unchecked.
- **Chains on WAMR, 1.6x native (2.2x with hardware bounds), against 1.1x C on V8**: 38% of chains' script samples are
  in `b3SolveSphericalJoint` (1624 instructions, 39 v128 spills); the rest is per step fixed cost spread over many small
  functions (dispatch, task callbacks, each AOT call through WAMR's entry stub). V8 runs the same module at C's speed,
  so this is WAMR's code generation and call overhead, not ours.
- **Queries, 1.11x C on V8, 1.4x native on WAMR**: the tree traversals (`b3RayCastTreeView` 1.18x and
  `b3BoxCastTreeView` 1.5x C's samples, with the visitor callback a closure call per candidate) and `b3ShapeDistance` at
  C's cost.
- **Integrate velocities, 1.1 to 1.2x C**: small (0.08 ms) and unchanged; its row stores build each vector with three
  lane replaces.

## How it was measured

- V8: bench/web/bench.mjs with `IMPLS` naming per change builds (`bench/web/out/as_simd_<tag>.wasm`, built with
  `--release` like the final ones), interleaved rounds, min of medians; stage tables from the same logs.
- V8 machine code: `node --print-wasm-code --no-liftoff run.mjs <impl> pyramid 5 --prewarm=0`, instructions counted per
  function by opcode class (scratchpad `v8code.py`); C from the named build `c_simd_split`.
- wasm ops: binaryen's text print per function (scratchpad `kops.py`).
- WAMR: per label project copies of bench/as with a snapshot of lib/, each with its own AOT cache, interleaved with
  native through bench/web/run.mjs (scratchpad `aotab.mjs`); static counts with `wamrc --format=object` and the ladder's
  flags (`--cpu-features=+reserve-x18,+lse` now; bench/as/profile.sh is updated to match and to symbolize the main
  module's code region, which is no longer the first rwx mapping when lane modules load).
