# solver: the step solver, graph, islands and solver sets

| C | AnimaScript |
|---|---|
| `src/solver.c` | `lib/solver.as` (below the marker) |
| `src/contact_solver.c` | `lib/contact_solver.as` (below the marker) |
| `src/contact_solver_wide.inl` at width 4, as `wide4.c` compiles it with `BOX3D_DISABLE_SIMD` | `lib/contact_solver_wide.as` (new) |
| `src/solver_set.c` | `lib/solver_set.as` |
| `src/island.c` | `lib/island.as` |
| `src/constraint_graph.c` | `lib/constraint_graph.as` |

Functions are in C order. The wide file runs on `lib/simd.as` through the W names, so it reads
line for line against the `.inl`, with `B3_SIMD_NONE` gather and scatter.

## Status

- `rasc check` passes on all six files (one warning: `b3MulM3VW` is unused, as in C).
- `verify/run.sh solver`: 124266 lines, 1.5 M values, 0 mismatches, with `-O`, without, and with
  `-D B3_ENABLE_ASSERT`. The full suite (28 modules, including world, joints, contact, bodyshape,
  mesh_contact, compound, mover) passes with the solver in.
- An instrumented copy showed every function in the six files runs under the driver except
  `b3MulM3VW` (unused in C) and `b3DestroyGraph` (the world porter replaces the world object in
  `b3DestroyWorld` instead of calling it).

### The check driver

`verify/cref/solver.c`, `verify/as/solver.as`, `verify/solver.mjs`. Four worlds, stepped one export
call per step with a collection between steps. After each step it prints every body's pose and
velocities, then from the internals: every solver set (body ids, state and sim flags, contact ids,
joint ids, island ids), every graph color (convex contact ids, scalar contact ids with manifold
counts, joint ids), every live island (set, local index, remove count, bodies, contacts, joints),
every body's set/local/island indices, move index, sleep time and velocity, flags, every joint's
set/color/local/island indices, every contact's indices, encoded sims, flags and all manifold
impulses and normal velocities, then the hit, joint and sensor events.

1. overflow: a plank falls with restitution and hit events onto 25 boxes, so most of its contacts
   overflow the colors; a bouncing ball (restitution, hit events, user material), a rolling capsule
   (rolling resistance), a tumbling rod (gyroscopic torque), a box with angular locks. The pile
   sleeps, an impulse wakes it, a box is destroyed (island split), the ball is re-woken.
2. mesh: boxes, capsules and spheres with restitution and hit events on a wave mesh (mesh contacts
   with up to 6 manifolds through the wide mesh path, 382 multi manifold lanes), a 70 m/s sphere
   swept into the mesh, restitution propagation switched on mid run.
3. joints: two boxes fall asleep apart and are joined while asleep (`b3MergeSolverSets`), then woken
   (a set with two islands); a hub with 24 distance joints (joint overflow); a revolute chain with
   force thresholds (joint events); a spherical pendulum; a weld; disable and enable of the hub
   (`b3TransferBody`, `b3TransferJoint`); a joint destroyed and a link body destroyed.
4. bullets: a bullet sphere into a dynamic stack, a 150 m/s capsule through a sensor into a thin
   wall (continuous, sensor hits from the sweep), warm starting switched off mid run.

Not yet run: the trajectory scenes of `verify/scenes` (no AnimaScript mirror of `scenes.c` yet).

## Signatures that differ from C

```as
// constraint_graph.as
b3CreateJointInGraph(world, jointId: i32): b3JointSim        // pushes new b3JointSim(), returns it
b3AddJointToGraph(world, jointSim: b3JointSim, jointId: i32)  // moves the reference into the new slot
// island.as
b3LinkContact(world, contact: b3Contact), b3UnlinkContact(world, contact)
b3LinkJoint(world, jointId: i32), b3UnlinkJoint(world, jointId: i32)
b3CreateIsland(world, setIndex): b3Island, b3SplitIslandTask(world)
// solver_set.as
b3TransferBody(world, targetSet, sourceSet, bodyId: i32), b3TransferJoint(world, targetSet, sourceSet, jointId: i32)
```

- `b3DestroySolverSet` and the sleep set reset (`*set = { 0 }`) replace the object in
  `world.solverSets`, so any reference to a destroyed set goes stale. The awake, static and disabled
  sets are never replaced.
- `b3CreateIsland` and `b3DestroyIsland` give the island fresh arrays (C's create and destroy);
  `b3SplitIsland` keeps the base arrays in locals and detaches them with fresh ones, so the base
  island reference stays valid where C has to cache fields before `b3CreateIsland`.
- solver.as private helpers: `b3InitBlocks(blocks, offset, dim, itemCount, blockType, colorIndex)`
  (C passes a pointer into the shared block buffer; each color owns a `StaticArray<b3SyncBlock>`),
  `b3InitStage(stages, stageIndex, ...)` and `b3InitColorStages(stages, stageIndex, ...)` return the
  next stage index where C returns `stage + 1`. `b3WorkerContext` is a class; `userTask` is the
  `u64` handle from `world.enqueueTaskFcn` (0 for NULL). Tasks go through the world porter's
  `enqueueTaskFcn` / `finishTaskFcn` with a closure carrying C's context, and `b3ParallelFor` takes
  closures over `b3FinalizeBodiesTask(start, end, worker, stepContext)` and `b3BulletBodyTask`.
- `b3ContinuousContext` is a class holding a copy of the fast body sim (the query reads only its
  flags and body id) and the fast shape; `b3ContinuousQueryCallback(proxyId, userData, context)` is
  wrapped in a closure for `b3DynamicTree_Query`. `b3SolveContinuous` copies the fast sim in and
  stores it back at the end; nothing else writes that sim during the sweep.
- The atomics are single threaded functions in solver.as (`b3AtomicCompareExchangeInt` and friends
  on `ref b3AtomicInt`), so the CAS claim and completion spin read as in C; with one worker every
  claim succeeds and the spin never waits.
- contact_solver.as: `b3ContactConstraint.constraints` is the first index into the overflow color's
  `manifoldConstraints`, `.contact` the contact id. `b3PrepareContacts_Overflow` and friends build
  their block with a shared `b3MakeOverflowBlock(count)`.
- contact_solver_wide.as, following the split stream of notes/decls.md:
  `b3PrepareConstraintW(c: ref b3ContactConstraintWide, manifolds, manifoldBase, contactIds:
  FixedArray<i32, 4>, manifoldCount, context, warmStartScale)` (`B3_NULL_INDEX` for C's NULL
  contact), and `b3WarmStartContactsW` / `b3PushContactsW` / `b3SolveContactsW` /
  `b3ApplyRestitutionW` / `b3StoreImpulsesW(block, context, [workerIndex,] constraints, manifolds,
  constraintBase, manifoldStarts: Int32Array, fixedManifoldCount)`. The first manifold of a block is
  the constraint index for convex (one manifold each) and `manifoldStarts[block.startIndex]` for
  mesh; `b3NextConstraintW` is `ci += 1; mi += manifoldCount`. Convex callers pass an empty
  `Int32Array` that the lazy ternary never reads.
- C's `B3_GATHER_LANES` over sim pointers is `b3GatherInvMassW(sims, lanes)` and
  `b3GatherInvInertiaW(sims, lanes)` over the awake indices, `B3_NULL_INDEX` standing for
  `b3_zeroBodySim`. The transposes of manifold and point floats are the equivalent `b3SetW` per
  output (a transpose of loads only moves bits). `( (const float*)&w )[lane]` is `b3LaneW(w, lane)`.
  C's lane pointer arrays (`contactLanes`, `manifoldLanes`) are module scratch arrays, since a
  FixedArray cannot hold references; they are not reentrant within one instance.
- The width dispatchers in contact_solver.as call the W4 versions directly (`simdWidth` is 4).

## Skipped

- `b3GetGraphColor` (debug draw colors).
- `b3GetWideContactConstraintByteCount`, `b3GetWideMeshConstraintByteCount`,
  `b3GetWideMeshManifoldByteCount` and their W4/W8 variants: allocation is by element counts.
- Every W8 function and the `B3_SIMD_HAS_WIDTH_8` dispatch branches.
- `b3ValidateIsland`, the `b3ValidateSolverSets` calls, `b3ValidateNoMoved`,
  `b3ValidateBroadPhase` (validation only), `B3_VALIDATE` lines.
- Tracy zones. The `b3Profile` timers are ported (notes/bench.md).
- `b3StackAlloc` / `b3StackFree`: each site is a local array; the overflow color's constraint arrays
  are reset to empty after the finalize pass (C leaves dangling pointers nobody reads).

## Calls into other modules

All exist and match: `b3GetBodySim`, `b3ShouldBodiesCollide`, `b3RefreshBodyContactIndices`,
`b3MakeRelativeSweep` (body), `b3ComputeShapeAABB`, `b3ComputeFatShapeAABB`,
`b3ComputeShapeMinExtent`, `b3ShapeTimeOfImpact`, `b3GetShapeUserMaterialId` (shape), the joint
prepare/warm start/solve functions, their `_Overflow` variants and `b3GetJointReaction` (joint),
`b3ParallelFor`, `world.enqueueTaskFcn` / `finishTaskFcn` / `userTreeTask` (world porter's additions
to `b3World`), `b3FindNameWithDefault` (name_cache.as), the tree and broad-phase helpers.

## Hazards found

1. Upstream: a body disabled and enabled since its last step keeps a stale `bodyMoveIndex`.
   `b3TrySleepIsland` then writes `fellAsleep` through it; C (release) writes past the event count,
   where the next step's finalize overwrites the slot, and a debug build would assert in
   `b3Array_Get`. The port skips the write when the index is past the array length instead of
   trapping (`verify/bodyshape` hit this). When the stale index is in range it writes the other
   body's event exactly as C does.
2. `b3FinalizeBodiesTask` works on copies of the state and sim: the state is final and stored before
   `b3SolveContinuous` (which writes its velocity), the sim is stored before the call and re-read
   after it, then stored at the end. Body fields are written through `world.bodies[id]` places.
3. The dummy static state (Hazards 3 of decls.md) is a copy of `b3_identityBodyState`; its flags are
   0, so the dynamic flag check never writes it, and real states are written through
   `states[index].field`.
4. `b3DestroyGraph` is never called; harmless while the world object is replaced on destroy.

## Wasm SIMD and jobs, per stage

Sizes are the C scalar build's (`sizeof`): body state 64 B, body sim 252 B, body 96 B, manifold
268 B (point 56 B), contact 216 B, joint sim 444 B (the port's is larger: the union is unfolded),
wide constraint 448 B, wide manifold 1296 B (a convex wide constraint is 1744 B, 436 B per lane),
overflow contact constraint 168 B, manifold constraint 324 B, island link 12 B, move event 48 B.

Every lane op of `lib/simd.as` maps to one or two `f32x4` instructions (exact replacements listed in
notes/containers.md, min/max/abs/blend/masks need the bitselect forms). Lane-wise math keeps C's
order exactly, so only horizontal sums and the sequential reductions need care.

| stage | where f32x4 replaces scalar lanes | independent as jobs? bytes crossing |
|---|---|---|
| prepare joints (flat over all colors) | none, scalar per joint | yes, per joint: in the joint sim plus the two bodies' sim fields it reads (transform, center, invMass, inertia, about 100 B each); out the prepared sim (444 B). |
| prepare wide contacts (flat) | all of `b3PrepareConstraintW`; the gathers become `v128.load` plus shuffles (C's SSE2 transposes), `b3PerpW`, `b3InvertSymW` are lane wise | yes, per wide constraint: in 4 contacts' friction, restitution, rolling, tangent velocity, encoded sims (about 40 B) plus their manifolds (268 B each), 8 sims' invMass and invInertiaWorld (40 B each), 8 states' velocities only with restitution or hit events (24 B each), about 400 B per contact; out the constraint and manifold, 436 B per contact, plus normal velocities written back into hit event manifolds (4 B per point). The lane-resident option below keeps the output on the job. |
| prepare mesh wide (flat) | as above, per manifold slot | yes, per group: as above with 268 B per manifold in and 1296 B per wide manifold slot out. |
| overflow prepare | none (scalar); could put a vec3 in a v128 lane triple | serial in C; one job: in each contact plus manifolds and two sims' mass data, out 168 B + 324 B per manifold. Must finish before the first warm start. |
| integrate velocities (body blocks) | vec3 adds and scales could run as one v128 with a dead w lane, exact; `b3MulMV` as column multiply-adds keeps C's order; dots and the gyroscopic `b3Solve3` stay scalar or extract lanes in order | yes, per body: in the sim's damping, invMass, gravityScale, force, torque, invInertiaWorld, inertia, rotation, flags (about 170 B) and the state's velocities and delta rotation (40 B); out 24 B. |
| warm start, solve, relax, restitution (one stage per color, iterated per sub-step) | wide contacts: everything, already 4 lanes; the B3_SIMD_NONE gather/scatter becomes the SSE2 style transposed `v128.load`/store of the 64 B state; joints scalar | within a color the blocks touch disjoint dynamic bodies (graph coloring), so each color's blocks are independent; colors run in order and overflow runs before them. If a job keeps its color's constraints resident across stages (they are written by prepare and read and written only by the same color's stages and store), only body states cross: in v, w, dp, dq (52 B of the 64) per body touched in that color, out v, w (24 B); static bodies never cross. Joints of the color use the same states. Without residency each stage would ship 436 B per contact both ways, which is far worse. |
| integrate positions (body blocks) | the lock blends and `dp + h v` exact in v128; `b3Length` and `b3IntegrateRotation` need ordered horizontal sums | yes, per body: in v, w, dp, dq, flags (56 B); out the same plus the speed cap flag (60 B). |
| overflow warm start/solve/relax/restitution | none | serial and between other stages: one job per stage, in the states of every body it touches (64 B each) and its constraints (or keep them resident), out the velocities. It orders before the colors of the same iteration. |
| store wide impulses (flat) | none needed; scalar lane extracts | yes, per wide constraint: out per manifold the friction, twist and rolling impulses (28 B) and per point 8 B, plus hit event bits (union of per worker bit sets afterwards, as C does). |
| store overflow impulses | none | serial; out as above. |
| finalize bodies (parallel for) | the sleep velocity arcs and transform update are vec3/quat work, exact lane wise except lengths and normalize | the body loop is independent, but it writes shape AABBs, fat AABBs, broad-phase moved flags and move events, and runs continuous collision against the shared static tree. As a job: in state 64 B, sim 252 B, body 96 B, and the shapes' geometry for the AABB (hull data can be large); out sim 252 B, body fields 16 B, move event 48 B, 48 B per shape, island bits and the split candidate. Continuous collision needs the trees, which cannot be copied per step, so it would stay on the main lane (C already defers bullets that way; deferring non-bullet fast bodies too would be a structural change). |
| bullets, hit/joint/sensor events, refit, island sleep | none | serial on the main lane. |
| split island (enqueued next to the solve in C) | none | yes, one job: in the island's body ids (4 B each), each body's island index (4 B), contact and joint links (12 B each); out the new island id per body, contact and joint (4 B each); the main lane applies the result. |

## rasc findings

None new. Relied on and confirmed working: `ref` bindings into elements of a `StaticArray` held in a
local that was read out of a `StaticArray<StaticArray<V>>`, `ref` into a `FixedArray` member of a
`ref`-bound element, a `ref` parameter forwarded to another `ref` parameter, struct ternaries
(`cond ? b3SplatW(1.0) : b3ZeroW()`), closures over a local class reference passed as tree query
callbacks, and FixedArray fields of structs inside a class.
