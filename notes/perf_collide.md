# perf: collide, pairs, queries

Everything in the step except the solve stage, plus the queries. Files: `lib/physics_world.as`
(collide task, contact state processing, update pairs, queries), `contact.as`, `broad_phase.as`,
`dynamic_tree.as`, `sensor.as`, `shape.as`, `compound.as`, `mesh_contact.as`, `convex_manifold.as`,
`triangle_manifold.as`, `manifold.as`, `hull.as`, `distance.as`, `sphere.as`, `capsule.as`,
`mesh.as`, `height_field.as`. Outside them: `verify/scenes/*` (three new scenes), a new verify module
(`scene_queries`), `verify/traj.sh` and `traj.mjs` (scene list), the bench (`bench/as/main.as`,
`run.sh`, `profile.sh`, `table.py`, `.gitignore`, `bench/native/bench.c`), and one finding appended to
`notes/findings.md`. `lib/simd.as` and the solver files are untouched.

## Results

Two measurements, both on the merge of this branch with main (3b0092d, the solver agent's SIMD
kernels), so the whole step is current. AOT -O3, release bakes, one worker, Apple M5 Max. The merged
tree passes `KEEP_GOING=1 verify/run.sh` (29 modules) and `verify/traj.sh --large` (12 scenes) bit
exact with `RASC_FLAGS="-O"` and with `RASC_FLAGS="-O --enable-simd"`.

**Stage times, before and after, run side by side** (`cmp.sh`: base c000c34, main 3b0092d and this
branch merged run at the same time each round, three rounds, min per field; load 14 to 40). ms per
step, mean over the timed steps:

| scene (steps) | collide base | collide main | collide merged | pairs main / merged | queries main / merged |
|---|---|---|---|---|---|
| large_pyramid (200) | 5.884 | 5.404 | **2.483** | 0.477 / 0.462 | |
| many_bodies (100) | 11.680 | 11.987 | **5.674** | 0.001 / 0.001 | |
| mesh_drop (170) | 7.450 | 8.081 | **6.148** | 1.340 / 1.297 | |
| village (200) | 4.780 | 5.203 | **2.689** | 0.916 / 0.875 | shape casts 0.750 / **0.558** |
| capsule_mesh (300) | 0.169 | 0.178 | **0.091** | 0.034 / 0.030 | |
| heightfield (300) | 0.160 | 0.163 | **0.079** | 0.017 / 0.016 | |
| pyramid (200) | 0.048 | 0.050 | **0.017** | | |
| queries (600) | 0.037 | 0.035 | **0.021** | 0.011 / 0.009 | see below |

Query scene per step (same runs), ms: closest rays 0.128 / **0.122**, all hit rays 0.042 / **0.044**,
sphere and box casts 0.206 / **0.162**, capsule overlaps 0.031 / **0.025**, movers 0.017 / **0.014**
(main / merged); native scalar in the same window 0.071, 0.020, 0.080, 0.012, 0.006.

Transforms, refit and sleep islands do not move (0.73, 0.025 and 0 ms on large_pyramid; 6.1, 0.14
and 0.11 on mesh_drop): transforms is the solver's finalize and continuous stage.

Interpreter (small pyramid, scalar artifact, two runs each, min): collide 0.259 base, 0.187 main,
**0.130** merged ms per step; step 6.22, 2.37, **2.20**.

**Whole step against native** (`ROUNDS=2 bench/as/run.sh` with all eight scenes on the merged tree,
interleaved with bench/native, load 13 to 35; session `bench/as/out/session_20261024_105400.log`):

| scene | native simd | native scalar | AS aot | AS / scalar | AS collide | native scalar collide |
|---|---|---|---|---|---|---|
| large_pyramid | 7.904 | 10.844 | 11.527 | 1.1 | 2.485 | 1.319 |
| many_bodies | 17.040 | 32.167 | 20.827 | 0.6 | 4.338 | 5.987 |
| pyramid | 0.055 | 0.098 | 0.091 | 0.9 | 0.011 | 0.007 |
| mesh_drop | 7.750 | 11.939 | 15.889 | 1.3 | 5.038 | 3.184 |
| village | 4.068 | 5.975 | 10.606 | 1.8 | 3.998 | 1.503 |
| queries | 0.288 | 0.274 | 0.468 | 1.7 | 0.022 | 0.009 |
| capsule_mesh | 0.140 | 0.178 | 0.319 | 1.8 | 0.087 | 0.035 |
| heightfield | 0.095 | 0.149 | 0.185 | 1.2 | 0.065 | 0.029 |

(many_bodies native scalar's best round ran at load 27, AS's at 35; the two rounds disagree by 30%,
so its ratio is not meaningful. AS interp pyramid: 2.845 ms per step, 29x native scalar.)

The query hashes of the AnimaScript and native runs are identical in every run (village and queries).
No AS run grew the heap: every scene reports `+0` wasm pages over its run (large_pyramid 1015 pages,
mesh_drop 1602, village 673, queries 145).

**On-CPU time per function, merged AnimaScript against native scalar** (Time Profiler on both, low
load, `profile.sh` with `DELAY=20` and `xctrace --launch` on `bench_scalar`; ms per step, the native
side includes its scene creation spread over the steps). "collide task" is the collide closure
(with `b3ResolveContactBodySim` in the base) against native `b3CollideTask`:

| scene | function | native | AS | AS / native |
|---|---|---|---|---|
| large_pyramid | collide task | 0.378 | 0.649 | 1.7 |
| large_pyramid | b3CollideCrossPairs | 0.056 | 0.044 | 0.8 |
| large_pyramid | b3PartitionMid | 0.056 | 0.040 | 0.7 |
| mesh_drop | b3ComputeMeshManifolds | 0.497 | 1.345 | 2.7 |
| mesh_drop | collide task | 0.181 | 0.322 | 1.8 |
| mesh_drop | b3ShapeDistance | 0.234 | 0.339 | 1.4 |
| mesh_drop | b3QueryMesh | 0.146 | 0.216 | 1.5 |
| mesh_drop | b3QueryTriangleAndHullEdges | 0.105 | 0.152 | 1.4 |
| mesh_drop | b3TimeOfImpact | 0.041 | 0.111 | 2.7 |
| mesh_drop | b3SolveContinuous (solver.as) | 0.146 | 0.708 | 4.8 |
| village | collide task | 0.219 | 0.751 | 3.4 |
| village | b3UpdateConvexContact | 0.065 | 0.249 | 3.8 |
| village | b3ShapeDistance | 0.303 | 0.423 | 1.4 |
| queries | b3ShapeDistance | 0.023 | 0.033 | 1.4 |

In the large_pyramid base profile the collide task and `b3ResolveContactBodySim` together held 0.45
of `b3SolveContactsW`'s samples; after changes 1 and 2 they held 0.12 (same scene, same solver).

## What is left

- Village (compound children against capsules, spheres and boxes) is the worst collide ratio, 2.7x
  native wall, 3.4x on CPU in the collide task: the compound path copies a `b3Shape` and fills a
  child per contact (now without allocating) and runs `b3UpdateConvexContact` out of line; the hull
  versus capsule and sphere paths go through the GJK. Inlining `b3UpdateConvexContact` into the task
  and a compound child cache per contact would be next.
- `b3ComputeMeshManifolds` is 2.7x native on CPU. It is a 16k instruction function with the
  triangle manifolds inlined and a flat profile; the remaining cost is the per access memory base and
  bound reloads (findings) and the four lane prefilter emulated in scalar structs. A v128 prefilter
  through `lib/simd.as` (the other agent's file) is the obvious next step: what it needs from simd.as is
  a `b3FloatW4` that is one v128 for Splat, Mul, Add, Sub, Min, Max, LessThan, GreaterThan, Or,
  MoveMask and Store, already exact under the bitselect forms notes/containers.md lists.
- The SAT (`b3ComputeSeparatingAxisW4`) is untouched at its call sites; it rides on simd.as and only
  shows up when contacts stop recycling (below 0.05 ms per step on the pyramids).
- Queries are 1.7 to 2.3x native per kind (wall, run.sh); `b3ShapeDistance` is 1.4x on CPU now, the tree traversals
  are at native speed, and what remains is the write barrier per pooled context store and the
  remaining per hit work in the callbacks.
- Transforms (`b3FinalizeBodiesTask`, `b3SolveContinuous`, 4.8x on mesh_drop) and sleep islands are
  the solver's.


## How it was measured

The machine was shared with other agents all session: the 1 minute load ran from 15 to 560 on 18
cores. Wall clock stage times inflate with preemption (one large_pyramid profile run had 9.3 s of
on-CPU samples in a 32 s run), so three kinds of evidence are used:

- `bench/as` runs of the base commit (c000c34) and the final one at the same time, one pair per
  round, three rounds, min per field (`cmp.sh` in the scratchpad: both processes share whatever load
  there is). The native rows come from `bench/native` in the same window.
- Time Profiler self time per function, which counts on-CPU samples only. Ratios against a solver
  function that this branch does not touch (`b3SolveContactsW`) cancel the coverage and the load.
- The same profiles on the native scalar build (`xctrace --launch` on `bench_scalar`), compared per
  function per step.

`bench/as/profile.sh` takes `DELAY=s` now (main.as spins that long before the first step), because
xctrace's attach took 10 to 15 s under this load and missed most of a run.

## New bench and parity scenes

`verify/scenes/scenes.c` and its mirror `scenes.as` gain three scenes, appended to the scene list so
existing indices hold:

- `mesh_drop`: upstream's `CreateMeshDropBenchmark` at release size, 100 x 100 small boxes that only
  collide with a 120 x 120 wave mesh (mesh contacts, continuous against the mesh). Scene LCG in place
  of `RandomVec3Uniform`.
- `village`: upstream's `CreateVillage` + `StepVillage` at release size: a rotated 100 x 100 tile
  compound (hull tiles, wave mesh patches, capsule and sphere props), 70 x 70 mixed bodies with every
  twentieth at 80 m/s (continuous against the compound), and a curtain of 401 sphere casts per step.
- `queries`: a static world with every shape type (rotated village compound, wave mesh, height
  field, single hulls, capsules, spheres, a ground box) and 100 resting bodies. Per step: 256 closest
  ray casts, 32 all hit ray casts, 64 sphere and 64 box casts, 64 capsule overlaps and 8 capsule
  movers (`Scene_SolveMove`: closest ray, collide mover, cast mover, plane solve). The all hit ray
  callback starts a closest ray cast every fifth hit and the overlap callback starts an overlap every
  seventh, so nested queries are exercised. Each kind is timed (`b3GetTicks`) and its results hashed.

`Scene_SolveMove` takes the mover as a parameter now (the mover scene passes its one mover).

Parity: `verify/traj.sh` runs `queries` by default and `mesh_drop` and `village` with `--large`
(120 steps each); the references are recorded with `NATIVE=1 verify/traj.sh queries mesh_drop
village` (verify/scenes/out is ignored, so a fresh checkout records them once). The new module
`scene_queries` (cref, as, mjs; the C side links scenes.c through `cref/scene_queries.flags`) prints
every query kind's hash and hit count and the movers' poses after each of 200 queries steps and 40
village steps, so query results are checked bit exact, not only their effect on bodies. The bench
prints the hashes too (`qRayHash=...`); AnimaScript and native match at 60 and 1500 steps.

Bench: `--define=SCENE=3..11` runs `capsule_mesh, heightfield, mesh_drop, village, queries, mover,
card_house, chains, bullets` through the scene mirror (run.sh and profile.sh copy
`verify/scenes/scenes.as` into `bench/as/scenes.as` with the import path rewritten), timed per
`Scene_Step` as bench/native does, and the result line adds `qRay qRayAll qShapeCast qOverlap
qMover` (ms per step) and their hashes; `bench/native/bench.c` prints the same fields and
`table.py` adds a query table. `--define=STEPLOG` logs every step's time and its stages over 0.5 ms.
Example: `AOT_SCENES="mesh_drop:170:5 village:200:6 queries:600:7" bench/as/run.sh`.

## Changes

Each change kept `KEEP_GOING=1 verify/run.sh` (29 modules) and `verify/traj.sh --large` (12
scenes) bit exact with `RASC_FLAGS="-O"` and `RASC_FLAGS="-O --enable-simd"` before its commit
(`traj.sh` with `WASM=...traj_simd.wasm` for the SIMD driver).

1. **Collide task reads sim fields in place** (36d5874). `b3ResolveContactBodySim` returned a whole
   `b3BodySim` (63 leaves, 252 B) by value per contact side: 615 loads and 419 stores and a multi
   value return through memory. It now resolves the sim array and the index
   (`b3ResolveContactBodySims` / `b3ResolveContactBodySimIndex`) and the task reads the transform,
   flags, maxExtent, center and localCenter where they live. large_pyramid profile: collide task
   plus resolve went from 0.45 to 0.22 of `b3SolveContactsW`'s samples.
2. **Relative transform in two inlinable parts** (05e6a6a). binaryen kept `b3InvMulWorldTransforms`
   out of line (14 float arguments, 8 on the stack, 7 results through memory, plus the entry stub);
   the call window was about a quarter of the collide task's samples. The task now writes
   `b3InvMulQuat` and `b3InvRotateVector` separately, same operations and order, and both inline.
   Collide task plus resolve: 0.22 to 0.12 of `b3SolveContactsW`; on CPU it is 0.86 ms/step against
   native `b3CollideTask` 0.67 ms/step (large_pyramid, 250 steps, both profiled the same way).
3. **Per contact scratch instead of allocation** (cd475fc). C bumps an arena or uses its stack; the
   port allocated per call. Now module scratch, grown on demand and reused, not cleared between calls
   (C's arena is not either, and every element is written before it is read):
   - `b3ComputeConvexManifold`: the 4 point buffer and the `b3LocalManifold` (zeroed with the newly
     exported `b3ZeroLocalManifold`, as C's `= { .points = ... }` zeroes it).
   - `b3UpdateContact` on a compound: the child (`b3GetCompoundChildInto`, new, fills a caller owned
     `b3ChildShape` and one element material array; `b3GetCompoundChild` keeps its API on top of it),
     the temporary child shape (`b3CopyShapeInto`, new) and its one material array.
   - `b3ComputeMeshManifolds` and `b3RefreshCache`: the triangle index buffer (512), the contact
     caches, the accepted and tentative manifold lists (pointer arrays became index arrays into the
     manifold buffer, as CONVENTIONS asks), the tentative triangles, the point buffer
     (32 x triangles), the pooled `b3LocalManifold`s, survivors, lane rows, clusters, cluster points,
     old manifolds and the consumed flags; `QSORT` closures became a reused `Q_LOOP` sorter; the
     triangle query callback captures nothing (closure plus context per refresh before).
   village collide: 5.9 to 3.2 ms (concurrent runs at load 170); mesh_drop collide minus 25 to 45%.
4. **GJK simplex backup as one memory copy** (bc63f3f). `backup = simplex` copied 49 fields through
   spilled locals every iteration. Both live in a two element module `StaticArray<b3Simplex>` now;
   the backup and the restore are one `memory.copy` and the two `{ 0 }` initializers one
   `memory.fill` (two small `unsafe` functions in distance.as). `b3ShapeDistance` samples went from
   0.99 to 0.79 of the tree traversal functions' in the query scene.
5. **Queries allocate nothing per call** (bc63f3f).
   - World queries (`b3World_OverlapAABB`, `OverlapShape`, `CastRay`, `CastRayClosest`, `CastShape`,
     `CollideMover`, `CastMover`): C's stack contexts are pooled by nesting depth
     (`b3AcquireQueryContext`, a pool and a depth per kind) because a user callback may start
     another query; the tree callbacks are plain functions that read the innermost context, so no
     closure per tree. `CastRayClosest` keeps its result in its context. `CollideMover` no longer
     allocates and zeroes its 64 plane buffer per call. The per shape copy of the cast input is one
     scratch.
   - Compound queries (overlap, ray cast, shape cast, mover, child cast input) keep one context each
     and plain tree callbacks; one shared `b3TreeView` (queries read its arrays when they start).
   - Primitive overlaps and casts (sphere, capsule, hull, mesh, height field, `b3ShapeCastShape`) reuse
     their `b3DistanceInput` / `b3ShapeCastPairInput` and proxies (`b3SetProxy`, `b3SetLocalProxy`,
     `b3SetShapeProxy`, new fill-in forms of the `b3Make...Proxy` helpers). mesh.as and
     height_field.as drop their private copies of `b3MakeLocalProxy`.
   - Time of impact (`b3ShapeTimeOfImpact`, mesh and compound paths, fallback): contexts, inputs,
     proxies and the compound child are scratch; callbacks capture nothing.
   - Nested queries are covered by the queries scene and `scene_queries` (a ray inside an all hit
     ray callback, an overlap inside an overlap callback).
   Query scene allocator samples (`__alloc` + `__new`) fell from 4.0% to about 1.5% of script time;
   the write barrier `__link` rose from 0.8% to 1.8%, since the pooled contexts store their
   references through the barrier on every query. What still allocates: a compound mesh child's `b3Mesh`, `b3QueryCompound`'s closure, and
   the debug paths.
6. **Per step buffers kept across steps** (af046d1): the collide task's awake contact index array
   (60 KB per step on large_pyramid) lives on the world; the broad phase keeps its pair contexts per
   worker, cross seeds and their queue, moved siblings and the sorted pair keys, and reuses its sort
   object; compound pair emission has one context and a plain callback; the sensor task has one
   query context, plain tree callback, reused visitor sorter, and reused visitor and local proxies.
7. **Unchecked GJK and mesh manifolds** (1db0bd2, 56e57ea). `@unchecked` on `b3ShapeDistance` and
   `b3ComputeMeshManifolds`. In-bounds by construction: the simplex lives in a fixed two element
   buffer and its count stays in [1, 4] (the loop stops at 4 before adding a vertex); support indices
   come from `b3GetProxySupport`, which reads the same points checked and returns an index below the
   count; every scratch buffer in the mesh function is grown at the top of the call to cover the
   triangle count, the point capacity and the cluster counts its loops are bounded by; cache and
   manifold indices are below the lengths set in the same call. Reads indexed by caller or shape
   supplied values stay checked through small helpers (`b3GetProxyPoint` for the cached simplex
   indices and point 0, `b3RecordSimplex` for the debug capacity, `b3MeshTriangleAt`,
   `b3MeshVertexAt`, `b3MaterialIndexAt`, `b3MaterialMapAt`, `b3SurfaceMaterialAt`). A cache whose
   count exceeds 4 can only come from outside `b3WriteCache`, as in C where it is an assert.
   `b3ShapeDistance` went from 0.83 to 0.64 of the traversal functions' samples in the query scene.
8. `@inline` on `b3PlaceLeaf` (tree rebuild); small, 0.07 to 0.04 ms/step on large_pyramid.

What did not help or was not done:
- The memory base and bound reloads WAMR emits around every store (see findings) dominate what is
  left in the collide task, the GJK and the mesh manifolds; nothing at source level removes them.
- Module constants: they are single instance loads in the AOT code; not worth a change.
- `b3QueryCompound` (public, takes a user function) still allocates its closure; it runs once per
  compound time of impact.
- The transforms stage (`b3FinalizeBodiesTask`, `b3SolveContinuous`) and sleep islands are in the
  solver files; the continuous callbacks they call into (`b3ShapeTimeOfImpact`) were made allocation
  free here, the rest belongs to the solver agent.


## Findings

- `StaticArray.fromArray<V>` traps (`unreachable`) for an array of structs; minimal repro appended
  to notes/findings.md. The village builder copies element by element.
- WAMR AOT with `--bounds-checks=1` reloads the memory base and the bound from the module instance
  after every store (`ldr x?, [x19, #0x178]` / `[x19, #0x1b8]` around each access in the collide
  task, the GJK and the mesh manifolds), so store dense code pays two extra loads and a compare per
  access. It is the floor under the remaining 1.3 to 2.8x on the narrow phase; hardware bounds (the
  guard page species) would remove the compare, not the reloads.
- A plain function passed where a closure type is expected compiles to a static closure record (no
  allocation); a capturing arrow allocates one per evaluation. Every hot tree query in these files now
  passes plain functions.
- Module constants do appear as instance loads in AOT code (`ldr s17, [x19, #0x1e4]` in the GJK), one
  load each from a hot cache line; no measurable cost was found, so nothing was changed for them.
