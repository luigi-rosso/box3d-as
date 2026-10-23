# decls: the dynamics layer declarations

The private headers of the dynamics layer, ported as declarations only, so the
function porters share one shape for every piece of data. Each file holds its
header's types, enums, flags, constants and static inline functions, and ends
with `// functions from <file>.c follow`. Append the C functions below that
line, in C order.

| C header | AnimaScript |
|---|---|
| `src/physics_world.h` | `lib/physics_world.as` |
| `src/body.h` | `lib/body.as` |
| `src/shape.h` | `lib/shape.as` |
| `src/contact.h` | `lib/contact.as` |
| `src/solver.h` | `lib/solver.as` |
| `src/solver_set.h` | `lib/solver_set.as` |
| `src/island.h` | `lib/island.as` |
| `src/constraint_graph.h` | `lib/constraint_graph.as` |
| `src/joint.h` | `lib/joint.as` |
| `src/sensor.h` | `lib/sensor.as` |
| `src/broad_phase.h` | `lib/broad_phase.as` |
| `src/contact_solver.h`, the types of `src/contact_solver_wide.inl` | `lib/contact_solver.as` |
| `src/manifold.h` | `lib/manifold.as` |
| `src/compound.h` | `lib/compound.as` |
| `src/parallel_for.h` | `lib/parallel_for.as` |

Every file passes `rasc check` (stub runtime). The modules import each other
in a cycle, as the C headers do; rasc resolves it.

## Verification

`verify/as/decls.as` builds every struct and class and exercises the access
patterns below (refs into wide constraint arrays and into a joint sim's union
member, Array places on struct arrays, class arrays, Int32Array views, spans,
the hull database map). The harness only runs modules with a C twin, so the
driver is not registered. Run it by hand:

```sh
RASC=~/Projects/rive-master/packages/rasc/animascript/out/release/rasc
AS_STD=~/Projects/rive-master/packages/rasc/animascript/std
$RASC verify/as/decls.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime frame -O -o verify/out/decls.wasm
node --input-type=module -e "import { loadWasm } from './verify/lib.mjs'; \
  console.log(loadWasm('verify/out/decls.wasm').exports.instantiate())"
```

It prints `0` (the number of failed expectations) under frame and stub, with
and without `-O`, and with `-D B3_ENABLE_ASSERT`.

## The rules every porter follows

### Pointer parameters

| C | AnimaScript |
|---|---|
| `b3World*` | `world: b3World` |
| `b3Body*` | `bodyId: i32`, the index into `world.bodies` (equals `body.id`) |
| `b3Joint*` | `jointId: i32`, the index into `world.joints` |
| `b3Shape*`, `b3Contact*`, `b3Island*`, `b3SolverSet*`, `b3Sensor*`, `b3GraphColor*`, `b3BroadPhase*`, `b3ConstraintGraph*`, `b3StepContext*`, `b3TaskContext*`, `b3JointSim*` | the class reference |
| `b3BodySim*`, `b3BodyState*` | an index into the owning `Array` (`set.bodySims`, `context.sims`, `context.states`) |
| a function returning `b3Body*` / `b3Joint*` | returns the index |
| `float* out` pairs | `ref f32` params; the caller passes `out[0]`, `out[1]` of a `let out: FixedArray<f32, 2>;` (a plain f32 local is refused) |
| `void* context` + callback | dropped; a closure carries it |
| `b3Arena arena` | dropped (CONVENTIONS) |

### Struct arrays versus class arrays

Struct elements live flat in `Array<T>`, and rasc refuses a `ref` into a
growable array ("Array<V> elements may reallocate"), including passing
`arr[i]` to a `ref` parameter. So C's `T* p = b3Array_Get(a, i)` becomes:

- reads: `a[i].field`, or copy once `let p = a[i];` when nothing writes;
- writes: `a[i].field = v` straight through the place, or copy, mutate,
  store back `a[i] = p`. A write into a copy that is never stored is lost
  (rasc warns W3001 only when the copy is dead).

Struct arrays: `world.bodies`, `world.joints`, `world.fatAABBs`,
`set.bodySims`, `set.bodyStates`, `set.islandSims`, `color.contacts`
(`b3ContactSpec`), `island.contacts` / `joints` (links), sensor visitors,
`taskContext.sensorHits`, the event arrays, `meshContact.triangleCache`.

Class arrays hold references that never move, so C's pointer into them stays
valid across pushes (C's own pointers do not; the port is safer there):
`world.contacts`, `world.shapes`, `world.islands`, `world.solverSets`,
`world.sensors`, `world.taskContexts`, `world.sensorTaskContexts`,
`set.jointSims`, `color.jointSims`, `graph.colors`, `bp.trees`.

Growing a class array through `length` (so `b3Array_AddIndex`,
`b3Array_Emplace`, `b3Array_Resize`) fills it with null. Push `new X()`
instead, for example `b3Array_Resize(world->taskContexts, n)` becomes a loop
of `push(new b3TaskContext())`.

Fixed size scratch the solver fills once per step (wide constraints, overflow
constraints, blocks, stages, spans) is `StaticArray`, so `ref c = arr[i]`
binds and refs can be passed to `ref` params.

### Zero values and slot reuse

All fields default to zero, as C's `{ 0 }`. Free slots in the sparse world
arrays keep their element, as C does:

- `b3CreateContact`: `*contact = (b3Contact){ 0 }` keeping the generation is
  `world.contacts[contactId] = new b3Contact()` then
  `generation = old + 1`. Push `new b3Contact()` for a new slot.
- `b3CreateShapeInternal` reuses the shape without zeroing it (generation
  advances); keep the object.
- `b3CreateIsland`, `b3DestroySolverSet` (`*set = (b3SolverSet){ 0 }`) and the
  sleep set reset: assign a fresh object (or fresh arrays), then the fields C
  sets after.

### Copies of classes

A class assignment aliases. Where C copies by value:

- `b3Shape childShapeA; memcpy(&childShapeA, shapeA, ...)` (contact.c 750):
  `b3CopyShape(shapeA)` in shape.as copies every field (arrays and geometry
  alias, as in C). Then give it its own one element `materials` before
  writing the child material, or the parent's array is overwritten.
- `memcpy` of a `b3JointSim` between sim arrays is always a move (the source
  is removed right after): push the same reference, never leave it in two
  arrays. `memset(jointSim, 0, ...)` after `b3Array_Emplace` is
  `push(new b3JointSim())`.
- `memcpy` of `b3BodySim` / `b3IslandSim` copies the struct:
  `target.bodySims.push(source.bodySims[i])`.

### Enums and flags

Plain enums are `type = i32` plus constants. Flag enums take the type of the
field that stores them, so masks need no casts: `b3BodyFlags = u32`,
`b3ContactFlags = u32`, `b3ShapeFlags = u8`.

### Nullable fields

`T?` means C may hold NULL; read with `!` after the check C makes.
`b3Shape.hull`, `mesh`, `heightField`, `compound`;
`b3World.frictionCallback`, `restitutionCallback` (never null after
`b3CreateWorld` installs the defaults), `preSolveFcn`, `customFilterFcn`.
Everything else is non null and starts as an empty array: C's NULL arrays
(`contact.manifolds`, `color.wideConstraints`, step scratch) are zero length
arrays, so test `length == 0` or the count C keeps beside them. Zero length
`StaticArray` defaults are shared module constants (nothing can be written to
them); growable `Array` defaults are always per object.

## Per file

### physics_world.as

- `b3SetType` enum: `b3_staticSet`, `b3_disabledSet`, `b3_awakeSet`,
  `b3_firstSleepingSet`.
- `b3TaskContext` (class): `sensorHits: Array<b3SensorHit>`,
  `pairKeys: Array<u64>`, `contactStateBitSet`, `jointStateBitSet`,
  `hitEventBitSet`, `hasHitEvents`, `awakeIslandBitSet`, `splitSleepTime`,
  `splitIslandId`, the profiling counters `satCallCount`, `satCacheHitCount`,
  `distanceIterations`, `pushBackIterations`, `rootIterations`,
  `recycledContactCount`, `manifoldCounts: FixedArray<i32, 8>`
  (B3_CONTACT_MANIFOLD_COUNT_BUCKETS; FixedArray needs a literal).
  Dropped: `arena`, `points`, `pointCount`, `lines`, `lineCount`, `cacheLine`.
- `b3HullDatabaseEntry` (class): `hull: b3HullData`, `referenceCount: i32`.
  `b3HullDatabase` (class): `buckets: Map<u64, Array<b3HullDatabaseEntry>>`
  keyed by `b3HullData.hash`, `count`. It replaces the verstable map keyed by
  content: look up the bucket, then compare content field by field (C trusts
  only content, not the hash). Nothing iterates it in an order that reaches
  results.
- `b3World` (class), every field kept except the skipped ones below:
  `broadPhase: b3BroadPhase`, `constraintGraph: b3ConstraintGraph`, the id
  pools `bodyIdPool`, `solverSetIdPool`, `jointIdPool`, `contactIdPool`,
  `islandIdPool`, `shapeIdPool` (`b3IdPool`, replace with `b3CreateIdPool()`
  in `b3CreateWorld`), `bodies: Array<b3Body>`,
  `solverSets: Array<b3SolverSet>`, `joints: Array<b3Joint>`,
  `contacts: Array<b3Contact>`, `islands: Array<b3Island>`,
  `shapes: Array<b3Shape>`, `fatAABBs: Array<b3AABB>`, `compoundShapeCount`,
  `hullDatabase: b3HullDatabase`, `names: Map<u32, string>`,
  `sensors: Array<b3Sensor>`, `taskContexts: Array<b3TaskContext>`,
  `sensorTaskContexts: Array<b3SensorTaskContext>`, `bodyMoveEvents`,
  `sensorBeginEvents`, `contactBeginEvents`,
  `sensorEndEvents` / `contactEndEvents: StaticArray<Array<...>>` of 2 (filled
  by the constructor), `endEventArrayIndex`, `contactHitEvents`,
  `jointEvents`, `stepIndex: u64`, `splitIslandId`, `gravity`,
  `hitEventThreshold`, `restitutionThreshold`, `restitutionIterations`,
  `enableRestitutionPropagation`, `maxLinearSpeed`,
  `isotropicInertiaTolerance`, `contactSpeed`, `contactHertz`,
  `contactDampingRatio`, `contactRecycleDistance`, `frictionCallback?`,
  `restitutionCallback?`, `generation: u16`, `profile: b3Profile`,
  `satCallCount`, `satCacheHitCount`, `manifoldCounts: FixedArray<i32, 8>`,
  `maxCapacity: b3Capacity`, `preSolveFcn?`, `customFilterFcn?`,
  `workerCount`, `simdWidth`, `userData: u64`, `inv_h`, `inv_dt`,
  `activeTaskCount`, `taskCount`, `worldId: u16`, `enableSleep`, `locked`,
  `enableWarmStarting`, `enableContinuous`, `enableSpeculative`, `inUse`.
- Skipped b3World fields: `stack` (local arrays), `manifoldAllocators`,
  `manifoldAllocatorMutex` (no block allocator), `debugBodySet`,
  `debugJointSet`, `debugContactSet`, `debugIslandSet`, `createDebugShape`,
  `destroyDebugShape`, `userDebugShapeContext` (debug draw; only
  `b3World_Draw` touches them), `enqueueTaskFcn`, `finishTaskFcn`,
  `userTaskContext`, `userTreeTask`, `scheduler` (no task system: run each
  task inline where C enqueues it; `taskCount` and `activeTaskCount` stay so
  the C bookkeeping ports as is), `recording`, `preSolveContext`,
  `customFilterContext` (closures). `names` replaces `b3NameCache`: the key is
  C's name id (`b3Hash32` of the name, `B3_NULL_NAME` 0 for none); the world
  porter owns that replacement for `b3AddName` / `b3FindName`. Also skipped:
  `B3_DEBUG_POINT_CAPACITY`, `B3_DEBUG_LINE_CAPACITY`, `b3DebugPoint`,
  `b3DebugLine`. `b3Profile` stays; its times come from the timers of notes/bench.md.
- `b3AllocateManifolds(world, count): StaticArray<b3Manifold>` returns
  `b3_emptyManifolds` for 0 (C NULL), else a fresh zeroed array.
  `b3FreeManifolds(world, manifolds, count)` is a no-op kept for call sites.

### body.as

- `b3BodyFlags = u32` and every flag: `b3_lockLinearX` ... `b3_lockAngularZ`,
  `b3_isFast`, `b3_isBullet`, `b3_isSpeedCapped`, `b3_hadTimeOfImpact`,
  `b3_allowFastRotation`, `b3_enlargeBulletBounds`, `b3_dynamicFlag`,
  `b3_enableSleep`, `b3_enableContactRecycling`, `b3_dirtyMass`,
  `b3_enableGyroscopicTorque`, `b3_allLocks`, `b3_fixedRotation`,
  `b3_bodyTransientFlags`.
- `b3Body` (struct, in `world.bodies`), C fields verbatim; `userData: u64`.
- `b3BodyState` (struct, 16 leaves, in `set.bodyStates`), padding fields
  kept. `b3_identityBodyState` (value global, `deltaRotation.s = 1`).
- `b3BodySim` (struct, 63 leaves, in `set.bodySims`).
- Inline: `b3EncodeBodySimIndex(body: b3Body)` (copy),
  `b3IsStaticSimIndex`, `b3DecodeAwakeIndex`, `b3SleepBodySimIndex`,
  `b3MakeRelativeSweep(bodySim: b3BodySim, base)` (copy; an Array element
  cannot be a ref).

### shape.as

- `b3ShapeFlags = u8`: `b3_enableSensorEvents`, `b3_enableContactEvents`,
  `b3_enableCustomFiltering`, `b3_enableHitEvents`, `b3_enablePreSolveEvents`,
  `b3_enableSpeculative`.
- `b3Shape` (class, in `world.shapes`). The geometry union is one field per
  member: `capsule: b3Capsule`, `sphere: b3Sphere`, `hull: b3HullData?`,
  `mesh: b3Mesh?` (C embeds the b3Mesh by value; assign
  `new b3Mesh(data, scale)`), `heightField: b3HeightFieldData?`,
  `compound: b3CompoundData?`. `userData` and `userShape` are `u64`.
- Deviation: C's inline `material` plus heap `materials` become one
  `materials: StaticArray<b3SurfaceMaterial>`, never null, length 1 for the
  single material case (C's inline one), `materialCount` long otherwise.
  C's `shape->material = m` is `shape.materials = new StaticArray(1);
  shape.materials[0] = m`. `b3GetShapeMaterials(shape)` returns it, so the
  setters that write `b3GetShapeMaterials(shape)[0].friction` work as in C.
  Freeing in `b3DestroyShapeAllocations` vanishes.
- `b3CopyShape(shape): b3Shape` (port only, see Copies of classes).
- Inline: `b3GetShapeMaterials`, `b3GetHeightFieldTriangleCount`,
  `b3GetMeshTriangle(mesh: b3Mesh, triangleIndex)`, `b3ShouldShapesCollide`,
  `b3ShouldQueryCollide(shapeFilter, queryFilter)` (values),
  `b3IsConvex`. The hull, height field and mesh functions declared in
  shape.h belong to hull.c, height_field.c and mesh.c.

### contact.as

- `B3_FORCE_GHOST_COLLISIONS`.
- `b3ContactCache` (struct): the union as two non aliasing fields
  `satCache`, `simplexCache`. A contact only uses the member for its shape B
  type, which never changes, so nothing reads the other member.
- `b3TriangleCache` (struct, constructor `(triangleIndex, cache)`).
- `b3ContactFlags = u32` and every flag (`b3_contactTouchingFlag` ...
  `b3_enableSpeculativePoints`).
- `b3ContactEdge` (struct), `b3ConvexContact` (struct),
  `b3MeshContact` (class: `triangleCache: Array<b3TriangleCache>`,
  `queryBounds`).
- `b3Contact` (class, in `world.contacts`): `edges: FixedArray<b3ContactEdge, 2>`,
  `manifolds: StaticArray<b3Manifold>` (`b3_emptyManifolds` for C NULL; it is
  the contact's own storage, so `contact.manifolds[i].field = v` writes
  through and `ref m = contact.manifolds[i]` binds), the union as
  `convexContact: b3ConvexContact` and `meshContact: b3MeshContact` (always
  allocated; it could be made lazy later if allocation shows up), the rest
  verbatim.
- `b3ContactSpec` (struct).
- There is no `b3ContactSim` in Box3D; CONVENTIONS names it from Box2D.

### solver.as

- Enums `b3SolverStageType` (`b3_stagePrepareJoints` ...
  `b3_stageStoreImpulses`) and `b3SolverBlockType` (`b3_bodyBlock` ...
  `b3_overflowBlock`).
- `b3SolverBlock` (struct), `b3SyncBlock` (struct: `block`, `syncIndex:
  b3AtomicInt`), `b3Softness` (struct, constructor
  `(biasRate, massScale, impulseScale)`).
- `b3SolverStage` (class): `blocks: StaticArray<b3SyncBlock>`, `type`,
  `blockCount`, `colorIndex: u8`, `completionCount`. C slices one graph block
  buffer per color (`graphColorBlocks[i] = baseGraphBlock`); give each color
  its own `StaticArray<b3SyncBlock>` of `graphBlockCounts[i]` and let
  `b3InitBlocks` take an offset for the joint, wide and mesh sub ranges.
  The body, convex, mesh and joint block arrays are plain StaticArrays.
- Spans (classes, built with a constructor, the pointer fields alias the
  color's arrays): `b3WidePrepareSpan(start, count, contacts: Array<i32>)`,
  `b3ContactPrepareSpan(start, count, contacts: Array<b3ContactSpec>)`,
  `b3MeshPrepareSpan(start, count, contacts, order: Int32Array)` where
  `order` is `meshLaneOrder.subarray(contactBase)` of an `Int32Array`
  mesh lane order, `b3JointPrepareSpan(start, count, joints: Array<b3JointSim>)`.
  The span tables are `StaticArray<span>` of `activeColorCount + 1`; the
  sentinel passes any array of the right type (it is never read).
- `b3StepContext` (class, `new b3StepContext(world)` replaces
  `b3StepContext context = { 0 }`; `graph` is `world.constraintGraph`):
  `dt`, `inv_dt`, `h`, `inv_h`, `subStepCount`, `contactSoftness`,
  `staticSoftness`, `maxLinearVelocity`, `world`, `graph`,
  `states: Array<b3BodyState>` and `sims: Array<b3BodySim>` (assign the awake
  set's arrays: `context.states = awakeSet.bodyStates`, an alias, as C's
  `.data`), `enlargedShapes: StaticArray<i32>`, `enlargedShapeCount`,
  `bulletBodies: StaticArray<i32>`, `bulletBodyCount: b3AtomicInt`,
  `anyRestitution: b3AtomicInt`, `awakeContactIndices: StaticArray<i32>`,
  `wideConstraints`, `wideManifolds` (new), `widePrepareSpans`,
  `wideContactCount`, `overflowSpans`, `wideMeshConstraints`,
  `wideMeshManifolds` (new), `wideMeshManifoldStarts: Int32Array`,
  `meshPrepareSpans`, `jointPrepareSpans`, `activeColorCount`,
  `workerCount`, `stages: StaticArray<b3SolverStage>`, `stageCount`,
  `enableWarmStarting`, `atomicSyncBits: b3AtomicU32`,
  `mainClaimed: b3AtomicInt`. Dropped: `padding1..3`.
  Atomics are the core.as structs; read and write `.value` (single thread).
- `b3MakeSoft(hertz, zeta, h): b3Softness`.

### contact_solver.as

- `b3ManifoldConstraintPoint`, `b3ManifoldConstraint`
  (`points: FixedArray<b3ManifoldConstraintPoint, 4>`) structs.
- `b3ContactConstraint` (struct, in `overflow.contactConstraints`): pointer
  fields keep their names and become indices: `constraints` is the index of
  the first manifold constraint in `overflow.manifoldConstraints` (C's
  `manifoldBase + specs[i].manifoldStart`, so assign `specs[i].manifoldStart`),
  `contact` is the contact id. C's `constraint->constraints + m` is
  `color.manifoldConstraints[cc.constraints + m]`.
- The wide (W4) types of contact_solver_wide.inl, struct names with the `W4`
  suffix as C's `B3_WIDE` expands them, exported also under the typedef names
  the .inl uses: `b3Vec2W`, `b3QuatW`, `b3SymMatrix2W`, `b3SymMatrix3W`,
  `b3Matrix3W`, `b3ContactConstraintPointWide`, `b3ContactManifoldWide`,
  `b3ContactConstraintWide`, `b3BodyStateW`. Import those names in the wide4
  port. `b3ContactConstraintWide.indexA` / `indexB` are
  `FixedArray<i32, 4>` (base 1, as C), `contacts` is `FixedArray<i32, 4>` of
  contact ids with `B3_NULL_INDEX` where C stores NULL. `_Alignas` has no
  equivalent and needs none. `b3ConvexConstraintWide` is not declared, see
  below.

The wide constraint stream. C stores `[c1 m11 m12 c2 m21 ...]`: each
`b3ContactConstraintWide` followed by its manifolds, addressed by byte
arithmetic (`b3GetConstraintW`, `b3GetManifoldsW`, `b3NextConstraintW`). The
port splits it into two arrays with no byte arithmetic:

- convex (one manifold per constraint, C's `b3ConvexConstraintWide`):
  `context.wideConstraints[i]` and `context.wideManifolds[i]`, both
  `wideContactCount` long;
- mesh: `context.wideMeshConstraints[g]` (`meshGroupCount` long) and
  `context.wideMeshManifolds[s]` (`meshSlotCount` long); group g owns slots
  `wideMeshManifoldStarts[g] .. wideMeshManifoldStarts[g + 1]`, and
  `wideMeshManifoldStarts` is the `Int32Array` of `meshGroupCount + 1`.

So C's `c = b3GetConstraintW(base, manifoldStarts, index, fixed)` becomes a
constraint index plus a first manifold index (`index` for convex,
`manifoldStarts[index]` for mesh), and `b3NextConstraintW` becomes `+ 1` on
both (plus `manifoldCount` on the manifold index for mesh). Bind with
`ref c = constraints[k]` and `ref m = manifolds[j]`. Byte counts
(`b3GetWideContactConstraintByteCount` and friends) are not needed for
allocation; port them only if the counters want them.

### constraint_graph.as

- `B3_OVERFLOW_INDEX` (23), `B3_DYNAMIC_COLOR_COUNT` (20).
- `b3GraphColor` (class): `bodySet: b3BitSet`, `jointSims: Array<b3JointSim>`,
  `convexContacts: Array<i32>`, `contacts: Array<b3ContactSpec>`, and the
  per step constraint storage, which references the step context's arrays
  plus where this color's slice starts:
  - `wideConstraints`, `wideManifolds` (the step's convex arrays),
    `wideConstraintStart` (C's `wideBase`), `wideConstraintCount`: C's
    `color->wideConstraints + k` is index `wideConstraintStart + k` in both;
  - `wideMeshConstraints`, `wideMeshManifolds` (the step's mesh arrays),
    `wideMeshConstraintStart` (C's `meshGroupBase`),
    `wideMeshManifoldStarts: Int32Array` (a `subarray(meshGroupBase)` view of
    the step's starts, so C's `color->wideMeshManifoldStarts[k]` reads the
    same; its values index `wideMeshManifolds` directly, since C's
    `starts[k] - starts[0]` is relative to a color base that already sits at
    `starts[0]`), `wideMeshConstraintCount`;
  - `manifoldConstraints: StaticArray<b3ManifoldConstraint>`,
    `contactConstraints: StaticArray<b3ContactConstraint>` (overflow only).
  C's NULL is a zero length array plus the zero count.
- `b3ConstraintGraph` (class): `colors: StaticArray<b3GraphColor>` of
  `B3_GRAPH_COLOR_COUNT`, filled by the constructor. `b3CreateGraph(graph,
  bodyCapacity)` then sets the bitsets as C does.

### solver_set.as

- `b3SolverSet` (class, in `world.solverSets`): `bodySims: Array<b3BodySim>`,
  `bodyStates: Array<b3BodyState>`, `jointSims: Array<b3JointSim>`,
  `contactIndices: Array<i32>`, `islandSims: Array<b3IslandSim>`, `setIndex`.

### island.as

- `b3ContactLink`, `b3JointLink` (structs), `b3Island` (class:
  `bodies: Array<i32>`, `contacts: Array<b3ContactLink>`,
  `joints: Array<b3JointLink>`, plus the ints), `b3IslandSim` (struct).

### joint.as

- `b3JointEdge` (struct), `b3Joint` (struct, in `world.joints`;
  `edges: FixedArray<b3JointEdge, 2>`).
- Per type structs, fields verbatim: `b3DistanceJoint`, `b3MotorJoint`,
  `b3ParallelJoint`, `b3PrismaticJoint`, `b3RevoluteJoint`,
  `b3SphericalJoint`, `b3WeldJoint`, `b3WheelJoint`.
- `b3JointSim` is a class. Deviation from the CONVENTIONS list, which calls it
  a struct: with eight union members it is about 440 leaves, every C use goes
  through a `b3JointSim*` into a growable array (which rasc cannot ref), and
  every C copy between arrays is a move. The union is one field per member
  (`distanceJoint`, `motorJoint`, `parallelJoint`, `revoluteJoint`,
  `sphericalJoint`, `prismaticJoint`, `weldJoint`, `wheelJoint`) embedded flat
  in the object. C's `b3RevoluteJoint* joint = &base->revoluteJoint;` is
  `ref joint = base.revoluteJoint;` (a `let` would copy and lose every
  write).
- Inline: `b3SolvePointConstraint(mA, mB, iA, iB, rA, rB, rhs): b3Vec3`.
- The joint and explosion defaults live in types.as (see notes/types.md);
  re-export them from joint.as rather than defining them.

### sensor.as

- `b3SensorHit` (struct, `(sensorId, visitorId)`), `b3Visitor` (struct,
  `(shapeId, generation)`), `b3Sensor` (class: `hits`, `overlaps1`,
  `overlaps2: Array<b3Visitor>`, `shapeId`), `b3SensorTaskContext` (class:
  `eventBits`).

### broad_phase.as

- `B3_PROXY_TYPE(key)`, `B3_PROXY_ID(key)`, `B3_PROXY_KEY(id, type)` as inline
  functions.
- `b3BroadPhase` (class): `trees: StaticArray<b3DynamicTree>` of
  `b3_bodyTypeCount` (filled by the constructor; `b3CreateBroadPhase`
  replaces each with `b3DynamicTree_Create`), `movedSiblings:
  StaticArray<i32>` (C's stack buffer; zero length outside the pair update),
  `pairSet: b3HashSet` (replace with `b3CreateSet` in `b3CreateBroadPhase`).
  C's `bp->trees + type` is `bp.trees[type]`.
- Inline: `b3BroadPhase_MarkProxyMovedSerial`, `b3BroadPhase_MarkProxyMoved`
  (call into lib/dynamic_tree.as).

### manifold.as

- `B3_MAX_CLIP_POINTS`, `b3SeparatingAxis`, `b3AxisQuery`, `b3ClipVertex`
  (structs), `b3FeatureOwner` (`b3_featureShapeA`, `b3_featureShapeB`),
  `b3FeaturePair_single` (value global), `b3MakeFeatureId`,
  `b3GetBestAxis(query: b3AxisQuery)` (copy; its `B3_VALIDATE` lines are
  skipped).

### compound.as

- `b3GetCompoundTreeView(compound): b3TreeView` with `compound.nodes` and null
  proxies (b3TreeView is in lib/dynamic_tree.as).

### parallel_for.as

- `b3ParallelForCallback = (startIndex, endIndex, workerIndex) => void`; the
  closure carries C's context. `b3ParallelFor(world, callback, itemCount,
  minRange, name)` runs the blocks inline on worker 0.

## Hazards

1. Copy then mutate. `let body = world.bodies[id]; body.sleepTime = 0;` loses
   the write. Every struct element (body, joint, body sim, body state, contact
   spec, link) is written through `arr[i].field` or stored back.
2. Stale copies across calls that move sims. C warns "careful calling this
   because it can invalidate body, state, joint, and contact pointers" for
   `b3WakeBody`; the same holds for `b3WakeSolverSet`, `b3TrySleepIsland`,
   `b3MergeSolverSets`, `b3TransferBody`, `b3TransferJoint`, `b3LinkJoint`
   (merges sets), `b3RemoveContactFromGraph` / `b3RemoveJointFromGraph`
   (swap removal) and island splitting. A copy of a body, sim or state taken
   before such a call, and any saved `setIndex` / `localIndex`, must be
   re-read after it, exactly where C re-fetches its pointer.
3. C's dummy state for static bodies (`b3BodyState dummyState =
   b3_identityBodyState; stateA = indexA == B3_NULL_INDEX ? &dummyState :
   states + indexA`): copy in with a ternary
   (`let stateA = indexA == B3_NULL_INDEX ? b3_identityBodyState :
   states[indexA]`), mutate, and store back only when the index is not null.
   Writes to the dummy were discarded in C too.
4. `b3GetBodySim` / `b3GetBodyState` return pointers that C writes through
   (see the signatures below).
5. Aliased arrays. `context.states` / `sims`, span arrays, and color slices
   alias arrays owned elsewhere; C's `.data` pointers are only valid while the
   array does not reallocate, the port's aliases while the Array object is not
   replaced. Never replace a set's or color's Array object (`= new Array()`)
   during a step; clear with `length = 0`. Destroying a sleeping set may
   replace its arrays.
6. A class copy aliases. `b3CopyShape` before mutating a shape temporary;
   give its `materials` a fresh one element array before writing it.
7. Class arrays grown through `length` hold null (see Struct arrays versus
   class arrays).
8. `b3Shape.materials` is shared by `b3CopyShape`, by design of the memcpy.
9. `sensor.c` sorts with libc `qsort` and a comparator that never returns 0
   (notes/containers.md).

## Cross module signatures

These are the shapes callers in other files rely on. The owner may keep
local helpers in any form.

physics_world.c (world porter)

- `b3GetWorldFromId(id: b3WorldId): b3World`, `b3GetUnlockedWorldFromId`,
  `b3GetWorld(index: i32): b3World`, `b3GetUnlockedWorld(index)`.
- `b3AddHullToDatabase(world, src: b3HullData): b3HullData`,
  `b3AddOwnedHullToDatabase(world, owned: b3HullData): b3HullData`,
  `b3RemoveHullFromDatabase(world, data: b3HullData)`.
- `b3World_RebuildStaticTree(worldId: b3WorldId)`.

parallel_for.c: `b3ParallelFor(world, callback: b3ParallelForCallback,
itemCount, minRange, name: string)`.

body.c (body porter)

- `b3GetBodyFullId(world, bodyId: b3BodyId): i32` (the body index).
- `b3GetBodyTransformQuick(world, bodyId: i32): b3WorldTransform`,
  `b3GetBodyTransform(world, bodyId: i32): b3WorldTransform`,
  `b3MakeBodyId(world, bodyId: i32): b3BodyId`.
- `b3ShouldBodiesCollide(world, bodyIdA: i32, bodyIdB: i32): bool`,
  `b3IsBodyAwake(world, bodyId): bool`.
- `b3GetBodySim(world, bodyId): b3BodySim` returns a copy. To write, use
  `world.solverSets[body.setIndex].bodySims[body.localIndex]`.
- `b3GetBodyState(world, bodyId): i32` returns the index into
  `world.solverSets[b3_awakeSet].bodyStates`, `B3_NULL_INDEX` where C returns
  NULL.
- `b3WakeBody(world, bodyId): bool`, `b3WakeBodyWithLock(world, bodyId): bool`,
  `b3UpdateBodyMassData(world, bodyId)`, `b3SyncBodyFlags(world, bodyId)`,
  `b3RefreshBodyContactIndices(world, bodyId)`.

shape.c (shape porter): `b3CreateShapeProxy(world, shape: b3Shape, type,
transform, forcePairCreation)`, `b3DestroyShapeProxy(shape, bp)`,
`b3DestroyShapeAllocations(world, shape)`, `b3ComputeShapeMass(shape)`,
`b3ComputeShapeExtent(shape, localCenter)`, `b3ComputeShapeMinExtent`,
`b3ComputeSweptSphereAABB(shape: b3Sphere, xf1, xf2)`,
`b3ComputeSweptCapsuleAABB(shape: b3Capsule, ...)`, `b3ComputeShapeAABB(shape,
transform)`, `b3ComputeFatShapeAABB(shape, transform, extra)`,
`b3ComputeSweptShapeAABB(shape, sweep: b3Sweep, time)`, `b3GetShapeCentroid`,
`b3GetShapeArea`, `b3GetShapeProjectedArea`, `b3GetShapeUserMaterialId(shape,
childIndex, triangleIndex): u64`, `b3MakeShapeProxy(shape): b3ShapeProxy`,
`b3RayCastShape(shape, transform, input: b3RayCastInput)`,
`b3ShapeCastShape(shape, transform, input: b3ShapeCastInput)`,
`b3OverlapShape(shape, transform, proxy: b3ShapeProxy)`,
`b3ShapeTimeOfImpact(shapeA, shapeB, sweepA: b3Sweep, sweepB: b3Sweep,
maxFraction)`, `b3CollideMover(planes: StaticArray<b3PlaneResult>,
planeCapacity, shape, transform, mover: b3Capsule): i32`. Small const struct
pointers (`const b3Sweep*`, `const b3Sphere*`, ...) pass by value.

contact.c (contact porter)

- `b3CreateContact(world, shapeA: b3Shape, shapeB: b3Shape, childIndex)`,
  `b3DestroyContact(world, contact: b3Contact, wakeBodies)`.
- `b3UpdateContact(world, workerIndex, contact, shapeA, localCenterA, xfA,
  shapeB, localCenterB, xfB, isFast): bool` (arena dropped).
- `b3ComputeMeshManifolds(world, workerIndex, contact, shapeA, materialMap:
  Uint16Array, xfA, shapeB, xfB, isFast): bool` (mesh_contact.c; the map is
  the child's `materialIndices` view, as in C).
- `b3InitializeContactRegisters()`.

sensor.c: `b3OverlapSensors(world)`, `b3DestroySensor(world, sensorShape:
b3Shape)`.

broad_phase.c: `b3CreateBroadPhase(bp, capacity: b3Capacity)`,
`b3DestroyBroadPhase(bp)`, `b3BroadPhase_CreateProxy(bp, proxyType, aabb,
categoryBits, shapeIndex, forcePairCreation): i32`,
`b3BroadPhase_DestroyProxy(bp, proxyKey)`, `b3BroadPhase_MoveProxy(bp,
proxyKey, aabb)`, `b3BroadPhase_GetShapeIndex(bp, proxyKey): i32`,
`b3UpdateBroadPhasePairs(world)`, `b3BroadPhase_TestOverlap(bp, proxyKeyA,
proxyKeyB): bool`.

solver_set.c: `b3DestroySolverSet(world, setIndex)`, `b3WakeSolverSet(world,
setIndex)`, `b3TrySleepIsland(world, islandId)`, `b3MergeSolverSets(world,
setIndex1, setIndex2)`, `b3TransferBody(world, targetSet, sourceSet,
bodyId: i32)`, `b3TransferJoint(world, targetSet, sourceSet, jointId: i32)`.

island.c: `b3CreateIsland(world, setIndex): b3Island`, `b3DestroyIsland(world,
islandId)`, `b3LinkContact(world, contact)`, `b3UnlinkContact(world,
contact)`, `b3LinkJoint(world, jointId)`, `b3UnlinkJoint(world, jointId)`,
`b3SplitIsland(world, baseId)`, `b3SplitIslandTask(world)` (C's
`void* context` is the world).

constraint_graph.c: `b3CreateGraph(graph, bodyCapacity)`,
`b3DestroyGraph(graph)`, `b3AddContactToGraph(world, contact)`,
`b3RemoveContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex,
meshContact)`, `b3CreateJointInGraph(world, jointId): b3JointSim` (pushes a
new sim and returns it), `b3AddJointToGraph(world, jointSim, jointId)` (moves
`jointSim` into the color: store the given reference in the new slot instead
of copying into the fresh one), `b3RemoveJointFromGraph(world, bodyIdA,
bodyIdB, colorIndex, localIndex)`.

solver.c: `b3Solve(world, stepContext)`.

contact_solver.c and wide4: `b3PrepareContacts_Mesh(block: b3SolverBlock,
context)` and the other `_Mesh`, `_Convex`, `_Overflow` stage functions keep
their C parameters (`b3SolverBlock` by value, the context reference,
`workerIndex` for store). The W8 and width dispatch variants are skipped;
`world.simdWidth` is always 4, so the dispatchers call the W4 versions.

joint.c (joint porter)

- `b3DestroyJointInternal(world, jointId, wakeBodies)`,
  `b3GetJointFullId(world, jointId: b3JointId): i32`,
  `b3GetJointSim(world, jointId: i32): b3JointSim`,
  `b3GetJointSimCheckType(jointId: b3JointId, type): b3JointSim`.
- `b3PrepareJoint(joint: b3JointSim, context)`, `b3WarmStartJoint`,
  `b3SolveJoint(joint, context, useBias)`, the `_Overflow` variants on the
  context, the per type `b3Prepare*Joint(base: b3JointSim, context)`,
  `b3WarmStart*Joint`, `b3Solve*Joint`, and `b3Get*JointForce` /
  `b3Get*JointTorque(world, base): b3Vec3`.
- `b3GetJointReaction(world, sim, invTimeStep, force: ref f32, torque: ref
  f32)`; solver.c passes `out[0]`, `out[1]` of a `FixedArray<f32, 2>`.
- Debug draw functions are skipped.

## What each porter needs to know

### world + queries (physics_world.c, parallel_for.c)

- `new b3World()` allocates every container; `b3CreateWorld` then does what C
  does (id pools via `b3CreateIdPool`, the three fixed solver sets pushed as
  `new b3SolverSet()` with `setIndex` from the pool, `b3CreateBroadPhase`,
  `b3CreateGraph`, task contexts pushed one by one with their bitsets from
  `b3CreateBitSet`). The world table `b3_worlds` is a module global
  `StaticArray<b3World>` filled with `new b3World()`; `b3DestroyWorld`
  replaces the slot's object rather than clearing fields one by one, keeping
  `generation`.
- No scheduler: where C calls `enqueueTaskFcn`, call the task inline and keep
  the `taskCount` / `activeTaskCount` bookkeeping. `b3ParallelFor` runs the
  callback over its blocks in order on worker 0.
- Hull database and name map replacements are yours (above).
- `b3Collide` builds `awakeContactIndices` as a `StaticArray<i32>`; the step
  context is `new b3StepContext(world)`.
- Event arrays are `Array<event struct>`; the double buffered end events are
  `world.sensorEndEvents[world.endEventArrayIndex]`.
- Counters: `stackUsed`, `arenaCapacity` and byte counts cannot match C
  (notes/containers.md).

### body + shape (body.c, shape.c)

- Bodies are structs reached by id; write through `world.bodies[id]`. Body
  sims and states are structs in the set arrays, reached through
  `body.setIndex` / `body.localIndex`. Read them once into locals and store
  the fields you change.
- `b3GetBodySim` returns a copy, `b3GetBodyState` an index (see signatures);
  check every caller that writes through C's pointer.
- Shapes are classes; reuse the slot's object on id reuse (C does not zero
  it). Materials are the single `materials` array (deviation above). Use
  `new b3Mesh(data, b3SafeScale(scale))` for the embedded mesh.
- `b3CopyShape` exists for the contact porter; keep it in sync if a field is
  added to `b3Shape`.
- Bullet and sleep flags are `u32`; shape flags are `u8`.

### contact + sensor + broad-phase (contact.c, sensor.c, broad_phase.c)

- Contacts are classes: keep the reference across calls freely. On create,
  replace the slot with `new b3Contact()` (C zeroes it) and advance the
  generation from the old object.
- `contact.manifolds` is a StaticArray from `b3AllocateManifolds`, empty for
  none; write manifold points in place with `ref m = contact.manifolds[i]`.
- `b3UpdateContact`'s compound path needs `b3CopyShape` plus a fresh one
  element `materials` on the copy.
- The contact cache union is two fields; zeroing `satCache` (mesh_contact.c
  684) leaves `simplexCache` alone, which nothing reads.
- `b3BroadPhase.movedSiblings` is a local `StaticArray<i32>` for the pair
  update (C's stack buffer); the pair tasks run inline.
- Task context `pairKeys` and `sensorHits` are growable arrays per worker.

### solver + graph + islands + sets (solver.c, constraint_graph.c, island.c, solver_set.c, contact_solver.c, wide4)

- Wide constraint layout: split arrays, see contact_solver.as and
  constraint_graph.as. Allocate `context.wideConstraints` and
  `wideManifolds` with `wideContactCount` elements, `wideMeshConstraints`
  with `meshGroupCount`, `wideMeshManifolds` with `meshSlotCount`,
  `wideMeshManifoldStarts` and `meshLaneOrder` as `Int32Array`s; the overflow
  color gets fresh `contactConstraints` and `manifoldConstraints` each step.
  Then hand each color the arrays plus its start.
- Gather and scatter (`b3GatherBodies`, `b3ScatterBodies`) take
  `states: Array<b3BodyState>` and the constraint's `indexA` / `indexB`
  FixedArrays; store only the velocity fields back, as C does.
- `b3ContactConstraintWide.contacts[lane]` is a contact id or
  `B3_NULL_INDEX`; fetch `world.contacts[id]` where C dereferences.
- Joint sims are classes: moves between sets and colors push the reference;
  `b3CreateJointInGraph` pushes `new b3JointSim()`.
- Stages and blocks: see solver.as. Single worker, so the sync atomics are
  plain fields, but keep their updates so the code diffs.
- Islands and sets are classes; sims inside are structs (copy on transfer).
- Scratch from `world->stack` becomes local arrays (notes/containers.md has
  every site).

### joints (joint.c, distance_joint.c, motor_joint.c, parallel_joint.c, prismatic_joint.c, revolute_joint.c, spherical_joint.c, weld_joint.c, wheel_joint.c)

- `b3JointSim` is a class; bind the type member with
  `ref joint = base.revoluteJoint;`, never `let`.
- `b3Joint` is a struct in `world.joints`; functions take the joint id.
- Body states come from `context.states` (an Array alias of the awake set's
  states) with the dummy identity state pattern from Hazards 3; body sims from
  `context.sims` or the set arrays, read only.
- `b3Softness` and `b3MakeSoft` are in solver.as; `b3SolvePointConstraint` is
  in joint.as.
- Prismatic and wheel joints call `b3GetBodySim` / `b3GetBodyState`; use the
  signatures above.

## rasc findings

No compiler or std changes. New ones found while laying these out:

1. A FixedArray length must be an integer literal; a constant is refused:
   ```as
   const N: i32 = 8;
   class C { a: FixedArray<i32, N>; } // error: FixedArray length must be an integer literal
   ```
   Workaround: the literal with a comment naming the constant.
2. Passing an element of a growable `Array<V>` to a `ref V` parameter is
   refused with the same message as a `ref` binding ("Array<V> elements may
   reallocate; copy or index directly"). Expected from the docs; listed
   because it decides every `T*` parameter above.
3. Confirmed working, relied on above: `ref x = obj.structField` and passing
   `obj.structField` to a `ref` parameter when `obj` is a class; `ref` into a
   `FixedArray` inside a `StaticArray` element; `Map<u64, Array<Class>>`;
   `Int32Array.subarray` views sharing storage; nullable class fields `T?`.
