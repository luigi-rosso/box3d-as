# contact: contacts, sensors and the broad-phase

| C | AnimaScript |
|---|---|
| `src/contact.c` | `lib/contact.as` (below the decls marker) |
| `src/sensor.c` | `lib/sensor.as` |
| `src/broad_phase.c` | `lib/broad_phase.as` |

Every function of the three files is ported, in C order: `b3GetContactFullId`, `b3Contact_GetData`,
the register table (`b3AddType`, `b3InitializeContactRegisters`), `b3CreateContact`,
`b3DestroyContact`, `b3ComputeConvexManifold`, `b3UpdateConvexContact`, `b3UpdateContact`;
`b3OverlapSensor`, `b3SensorQueryCallback`, the visitor sort, `b3SensorTask`, `b3OverlapSensors`,
`b3DestroySensor`; `b3CreateBroadPhase`, `b3DestroyBroadPhase`, the proxy functions,
`b3GatherMovedSiblings`, the candidate batch (`b3FlushCandidatePairs`, `b3AddCandidatePair`),
`b3ShouldCreatePair`, the compound pair path (`b3CompoundChildCallback`, `b3EmitCompoundPairs`),
`b3TestPair`, `b3CollideProxyAndSubtree`, `b3VisitPair`, `b3CollideCrossPairs`, `b3SelfPairsTask`,
`b3GatherCrossSeeds`, `b3CrossPairsTask`, `b3UpdateTreesTask`, `b3EnqueueTreeUpdate`,
`b3UpdateBroadPhasePairs`, `b3BroadPhase_TestOverlap`, `b3BroadPhase_GetShapeIndex`,
`b3ValidateBroadPhase`, `b3ValidateNoMoved`.

The task structure is kept: the self and cross pair tasks and the sensor task are
`(startIndex, endIndex, workerIndex, context)` functions handed to `b3ParallelFor` through a closure
that carries the context; the tree rebuild goes through `world.enqueueTaskFcn` as in C; pair keys
and sensor bits live in the per worker task contexts.

## Verification

- `rasc check` on all three files: no diagnostics, also with `--strict` and `-D B3_ENABLE_ASSERT`.
- `verify/contact_collide.sh`: **17567 lines, 214345 values, 0 mismatches**, also without `-O`
  (`RASC_FLAGS=`) and with `RASC_FLAGS="-O -D B3_ENABLE_ASSERT"`. This is the scripted scene of
  the contact driver with `b3Solve` replaced by a no-op on both sides (C: `-DCONTACT_COLLIDE_ONLY` overrides `b3Solve`, the only
  symbol in the archive's `solver.c.o`; AS: the script copies `lib`, renames the real `b3Solve` if
  present and appends an empty one). Every dynamic body follows a scripted path through
  `b3Body_SetTransform`, so the run exercises pair finding (self, static cross, kinematic cross,
  compound children), contact creation order, convex manifolds of every type pair including SAT
  and simplex caches, compound children of every type (the shape copy path), mesh and height
  field contacts, contact recycling, begin/end touch, disjoint destruction, body destruction, a
  filter change and its undo, a custom filter callback and a pre-solve callback (both reject and
  accept a few hundred times), a static sensor destroyed at step 120 and a kinematic sensor at
  160, sensor begin/end events and overlap arrays. The other modules in the loop are the real ports
  (world, body, shape, island, graph, convex manifold, mesh contact, compound).
- `verify/run.sh contact` (`verify/cref/contact.c`, `verify/as/contact.as`, `verify/contact.mjs`):
  **107665 lines, 1433915 values, 0 mismatches** with the whole port in the loop (solver
  included), also without `-O` and with `-D B3_ENABLE_ASSERT`. Five scenes: `mixed` (all convex
  pairs, static and kinematic cross pairs, two bodies destroyed, a filter change, a body flung
  away), `sensor` (static and kinematic sensors, visitors with and without sensor events, a sensor
  shape destroyed holding overlaps, a visitor destroyed inside a sensor), `compound` (static
  compound with capsule, hull, sphere and mesh children under a pile), `heightfield`, and
  `scripted` (as above, with the solver running). After every step it prints the tree proxy
  counts and moved flags, the pair set count, worker 0's pair keys in arrival order, every contact
  slot (ids, sets, colors, islands, flags, edges, mixing values, the cache that contact actually
  uses, manifolds and points with impulses), the body contact lists and transforms, contact begin
  (with `b3Contact_IsValid` and `b3Contact_GetData`), end and hit events, sensor events and the
  sensor overlap arrays. 15894 of 26061 printed points carry a nonzero impulse and islands fall
  asleep. `CONTACT_SCENES=4` makes the runner play only the scripted scene.
- Not verified: `b3DestroyBroadPhase` (the world calls it at destroy, nothing reads the result),
  `b3Contact_GetData` on contacts without manifolds, more than one worker.

## Signatures that differ from C

- `b3CreateBroadPhase(bp, capacity: b3Capacity)` takes the capacity by value.
- `b3GatherMovedSiblings(tree, pairIndices: StaticArray<i32>)`; `bp.movedSiblings` is a fresh
  `StaticArray` during the pair update and the shared empty array outside it (C NULL).
- `b3GatherCrossSeeds(treeA, treeB, seeds, seedBase)`: C passes `crossSeeds + staticSeedCount`.
- `b3TestPair`, `b3CollideProxyAndSubtree`, `b3VisitPair`, `b3CollideCrossPairs` take nodes by value
  plus the node arrays where C takes node pointers. `b3VisitPair` returns the new stack count
  that C writes through `int* stackCount`.
- `b3PairContext` is a class per task call. It also owns the flush buffers (`plain`, `keys`,
  `hashes`, `candidates`) and the two traversal stacks, which C keeps on the stack (512 entries is
  past the FixedArray local limit). Neither function using a buffer can re-enter itself.
- Task and query callbacks take their C context as a typed last parameter (`b3SelfPairsTask(...,
  world)`, `b3CrossPairsTask(..., crossContext)`, `b3SensorTask(..., world)`,
  `b3CompoundChildCallback(proxyId, userData, compoundContext)`, `b3SensorQueryCallback(...,
  queryContext)`); a closure adapts each to the callback type.
- `b3EnqueueTreeUpdate` hands `world.enqueueTaskFcn` a closure over the world with task context 0,
  and treats `userTreeTask == 0` as C's NULL. (notes/decls.md listed these world fields as skipped;
  the world porter kept them, so this follows C.)
- `b3UpdateContact`, `b3UpdateConvexContact`, `b3ComputeConvexManifold` drop the `b3Arena`. The
  convex point buffer is a local `StaticArray<b3LocalManifoldPoint>(4)` and the local manifold a
  new `b3LocalManifold`.
- `b3Contact_GetData` returns a `b3ContactData` whose `manifolds` references the contact's own
  manifold array (C returns the pointer), `null` when there are none.
- `b3CreateContact` replaces the slot with `new b3Contact()` and advances the old generation
  (C's `*contact = (b3Contact){ 0 }`).
- The compound path in `b3UpdateContact` copies the parent with `b3CopyShape` and, for convex
  children, gives the copy its own one element `materials` holding the child material (C's inline
  `material` plus `materials = NULL`). Mesh children keep the parent's array, as in C.
- `b3DestroyContact` replaces `meshContact.triangleCache` with a fresh array (`b3Array_Destroy`).
  `b3DestroySensor` truncates the sensor's three arrays; the sensor object itself is dropped by
  the swap removal.
- The register table `s_registers[6][6]` is a flat `StaticArray` indexed `type1 * 6 + type2`.

## Skipped

- `b3PrefetchHash` and the `b3Prefetch` calls (cache hints only).
- The `b3ValidateSolverSets` call at the end of `b3UpdateBroadPhasePairs` and the `B3_VALIDATE`
  duplicate pair trap (validation is off in the reference).
- Tracy zones.

## Calls into other modules

All resolved against the current files: `b3GetWorld`, `b3AllocateManifolds`, `b3FreeManifolds`,
`b3ParallelFor` (world); `b3EncodeBodySimIndex`, `b3WakeBody(world, bodyId)`,
`b3GetBodyTransform(world, bodyId)`, `b3GetBodyTransformQuick(world, bodyId)`,
`b3ShouldBodiesCollide(world, bodyIdA, bodyIdB)` (body); `b3CopyShape`, `b3MakeShapeProxy`
(shape); `b3UnlinkContact` (island); `b3RemoveContactFromGraph` (graph); the six `b3Collide*`
functions with `ref` caches (convex_manifold); `b3ComputeMeshManifolds(..., materialMap:
Uint16Array?, ...)` (mesh_contact, `null` for plain meshes and height fields, as C's NULL);
`b3GetCompoundChild`, `b3GetCompoundTreeView`, `b3OverlapCompound` (compound); `b3OverlapHull`,
`b3OverlapMesh`, `b3OverlapHeightField`, `b3OverlapCapsule`, `b3OverlapSphere`.

## Hazards

1. notes/decls.md says a contact only uses the cache union member of its shape B type. Not quite:
   a compound capsule or sphere child against a hull is flipped, so that contact has a hull shape
   B and uses `simplexCache`. Each contact still uses one member for its whole life, so the two
   field split stays exact. In C the unused member aliases the used one, so a dump must pick the
   member by actual use (the driver's `UsesSatCache`).
2. Sensor overlaps are sorted with libc `qsort` and a comparator that never returns 0; the port
   uses the Tourbin `Q_LOOP` with `shapeId <`. Only ties could order differently, and there are
   none at this pin: nothing in `src/` pushes to `sensor->hits` (the task context `sensorHits` are
   reserved and never filled), and each shape sits in one tree, so a shape id appears once per
   sensor query. If continuous sensor hits are wired up later, a hit and a query entry for the
   same shape carry the same generation (the world is locked in between), so ties stay identical.
3. `b3DestroyContact` binds `ref edgeA = contact.edges[0]` and writes other contacts' edges
   between reads; an edge never links a contact to itself, so the aliasing matches C.
4. `b3CreateContact` reads body copies for flags, type and set, and writes head keys and counts
   through `world.bodies[id]`, reading the head key from the array again where C reads it through
   the pointer.
5. Per step allocations, all freed at the next frame boundary: the point buffer and local
   manifold per convex contact update, the shape copy, child shape and material array per
   compound child update, the shape proxy and 64 point buffer per sensor visitor test, one
   `b3PairContext` (with its stacks) per pair task call, the cross seed queue per gather. A per
   worker scratch pool can replace them without changing results if they show up in a profile.

## rasc findings

None new. Relied on: `ref` to a `FixedArray` element inside a class field
(`ref edgeA = contact.edges[0]`), `ref` into an element of a `StaticArray` field of a class and then
into its `FixedArray` (`ref mp = contact.manifolds[i].points[j]`), passing a struct leaf chain of a
class to a `ref` parameter (`contact.convexContact.cache.satCache`), calling a nullable function
typed field after a `!` (`world.frictionCallback!(...)`), and `Q_LOOP` with a sorter class over an
`Array` of structs.
