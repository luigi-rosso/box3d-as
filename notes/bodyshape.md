# bodyshape: body.c and shape.c

## What is ported

| C | AnimaScript |
|---|---|
| `src/body.c` | `lib/body.as`, below the `body.h` declarations |
| `src/shape.c` | `lib/shape.as`, below the `shape.h` declarations |

Every function of both files, in C order: body create and destroy, every `b3Body_*` function, mass
data and automatic mass, isotropic inertia, type changes, enable and disable, sleep, bullets, names,
user data, contact and joint lists, motion locks, target transforms, the body queries
(`b3Body_CastRay`, `b3Body_CastShape`, `b3Body_OverlapShape`, `b3Body_CollideMover`,
`b3Body_TimeOfImpactMover`, `b3Body_GetClosestPoint`); shape create for every geometry type (sphere,
capsule with the degenerate to sphere path, hull through the world hull database, transformed hull,
mesh, height field, baked compound), shape destroy, proxies, margins, mass, AABBs (plain, fat,
swept), extents, centroids, areas, ray, shape cast and overlap dispatch, the mover dispatch
`b3CollideMover`, materials, filters, event flags, every `b3Shape_*` function, wind, the shape time
of impact for convex, mesh, height field and compound, and `b3GetShapeUserMaterialId`.

Imports changed in the header part of both files (the declarations themselves are untouched).

## Signatures that differ from C

- Internal functions take `bodyId: i32` for `b3Body*` (decls). `b3GetBodySim` returns a copy,
  `b3GetBodyState` returns the awake set index or `B3_NULL_INDEX`.
- `b3Body_GetClosestPoint(bodyId, result: ref b3Vec3, target)`.
- `b3Body_GetContactData` / `b3Shape_GetContactData` take `StaticArray<b3ContactData>` and store a
  new `b3ContactData` in each slot they fill. `b3Body_GetShapes`, `b3Body_GetJoints`,
  `b3Shape_GetSensorData` take `StaticArray` of ids. `b3Body_CollideMover` takes
  `StaticArray<b3BodyPlaneResult>`.
- `b3Body_CastShape`, `b3Body_OverlapShape` take the proxy by reference (`b3ShapeProxy` is a class);
  `b3Body_CastShape` copies it into its input as C does.
- Small const struct pointers pass by value: `b3Shape_SetSphere(id, sphere)`,
  `b3Shape_SetCapsule(id, capsule)`, `b3Body_CollideMover(..., mover: b3Capsule, ...)`,
  `b3ShapeTimeOfImpact(shapeA, shapeB, sweepA: b3Sweep, sweepB: b3Sweep, maxFraction)`,
  `b3ComputeSweptShapeAABB(shape, sweep: b3Sweep, time)`.
- `b3Shape_GetMesh` returns a new `b3Mesh` (C returns the embedded struct by value).
  `b3Shape_GetHull`, `b3Shape_GetHeightField`, `b3Shape_GetCompound` return the nullable reference.
- `b3CollideMover` calls `b3CollideMoverAndMesh` / `b3CollideMoverAndHeightField` with plane offset 0
  and `b3CollideMoverAndCompound(planes, capacity, ...)` (the compound port has no offset).
- Static helpers: `b3UpdateShapeAABBs` returns the fat AABB for the caller to store in
  `world.fatAABBs` (C writes through a pointer). `b3CreateShapeInternal` / `b3CreateShape` take a
  private `b3ShapeGeometry` holder in place of `const void* geometry`. `b3MeshImpactContext` and
  `b3CompoundImpactContext` are classes captured by the query closures; `b3CopyTOIInput` replaces
  C's by value copies of `b3TOIInput` and copies both proxies.
- `b3UpdateIsotropicInertia(world, bodyId, bodySim: b3BodySim)` takes the sim copy (it only reads
  the inertia).

## Materials

`materials` is always the array (decls). Single material shapes get a fresh one element array on
every create, so a reused slot never writes into a previous owner's array. `b3DestroyShapeAllocations`
resets `materials` and `materialCount` when C's `materials != NULL` would hold, which the port reads as
`type == b3_compoundShape || materialCount > 1`; the reset is not observable after destroy. A compound
with zero materials gets a one element zero material where C falls back to the inline material.

## Skipped

- `B3_REC`, `B3_REC_CREATE` and every `world->recording` branch (`b3CreateHullShape`,
  `b3CreateTransformedHullShape`, `b3CreateMeshShape`, `b3CreateHeightFieldShape`,
  `b3CreateBakedCompoundShape`, `b3Shape_SetHull`, `b3Shape_SetMesh`).
- `B3_VALIDATE` and the validation calls: `b3ValidateSolverSets`, `b3ValidateIsland`,
  `b3ValidateConnectivity`.
- Debug draw: the `destroyDebugShape` callback in `b3DestroyShapeAllocationForShapeChange` (the port
  only clears `userShape`), the `B3_DEBUG_WIND` block in `b3Shape_ApplyWind`.
- The `BOX3D_DOUBLE_PRECISION` path of `b3ComputeFatShapeAABB` and the `#if 0` fallback in
  `b3ShapeTimeOfImpact`.
- Stack allocation of `masses` in `b3UpdateBodyMassData` is a local `StaticArray`.

## Calls into unported code

None at the time of writing: every callee exists and `rasc check` passes on both files against the
other modules as they stand.

## Hazards found

1. Upstream stale `bodyMoveIndex`, which trapped the port of `b3TrySleepIsland` (solver_set.as, not
   mine). `b3Body_Disable` keeps the body's `bodyMoveIndex`; after a step without the body, a
   `b3Body_Enable` then `b3Body_SetAwake(false)` reaches `b3TrySleepIsland` with an index past
   `world.bodyMoveEvents.length`. C release writes `fellAsleep` past the count (dead capacity), or onto
   another body's live event when the stale index is below the count. The port trapped, because its two
   `B3_ASSERT(world.bodyMoveEvents[bodyMoveIndex]...)` lines evaluate their arguments even with asserts
   off (notes/types.md, rasc findings) and the write was bounds checked. solver_set.as now guards the
   write with `bodyMoveIndex < world.bodyMoveEvents.length`, which is the faithful form: below the
   count it still marks the other body's event as C does. `verify/bodyshape` covers it in
   `phaseExtrasJoints`.
2. `b3Shape_ApplyWind` takes the body sim pointer before `b3WakeBodyWithLock`. When the body was
   asleep, C writes the wind force into the destroyed sleeping set, so it is lost. The port reads a copy
   and stores the force only when the body was already awake.
3. `b3Body_SetType` returns with the world still locked when the body has a compound or height field
   shape. Kept.
4. `b3Shape_SetFilter` computes `destroyProxy` after assigning the filter, so it is always true. Kept.
5. A destroyed body id whose slot was reused reads the new occupant in C release; the port does the
   same, but traps under `-D B3_ENABLE_ASSERT` (generation check). The driver skips invalid ids.
6. Struct copies: every body, body sim and body state write goes through the array place or a copy
   stored back before the next call that can touch the same element (`b3SyncBodyFlags`,
   `b3UpdateIsotropicInertia`, wakes and transfers). `b3UpdateBodyMassData` and `b3Body_SetMassData`
   copy the sim after the first flag sync and store it back before every return and before the
   isotropic inertia sync.

## Verification

`verify/cref/bodyshape.c`, `verify/as/bodyshape.as`, `verify/bodyshape.mjs`. Two worlds through the
public API, with internals (body set and island indices, flags, sims, states, shape margins, proxy
keys, fat AABBs, id pool counts) read from the world on both sides:

- world 1: static box with a sensor, two material mesh with negative scale, height field, dynamic
  sphere, capsule, bullet box, transformed and shared rock hulls, three shape body with a zero density
  shape, kinematic, sleeping, disabled, motion locked (deferred mass), fixed rotation; every getter;
  queries per body (closest point, ray, shape cast, overlap, mover planes, mover TOI) and per shape
  (ray, closest point); every setter, forces and impulses on awake and sleeping bodies, mass data,
  type changes static, kinematic, dynamic, disable and enable, sleep and wake, target transform,
  shape density, materials, filters, flags, shape type changes, wind, shape and body destroy with id
  reuse; 90 steps with contact and sensor data.
- world 2: baked compound with every child type, mesh plus sensor, height field, 9 fast bodies (half
  bullets) into each, a revolute and a spherical joint; joint lists, compound ray, shape cast and
  overlap, hit and sensor events every step (user material ids), joints through type changes, disable,
  enable, sleep, `b3Shape_SetMesh`, destroy of a jointed body. Probed: the compound and mesh time of
  impact callbacks, `b3ShapeTimeOfImpact`, `b3GetShapeUserMaterialId`, `b3ComputeSweptShapeAABB`,
  `b3MakeLocalProxy`, `b3ComputeProxyAABB`, `b3ResetProxy` and `b3CollideMover` all run.

Results: 4912 lines, 92126 values, 0 mismatches under `-O`, without `-O`, and with
`-D B3_ENABLE_ASSERT`. The rest of `verify/run.sh` passes with these files in place.

Not covered: the lost force path of hazard 2 (C undefined behavior), `b3Body_SetType` on a body
with a compound (returns locked), compounds with zero materials, worlds with more than one worker.

## rasc findings

1. A whole struct element store through a call result is refused, though a leaf store through the
   same call compiles:
   ```as
   struct M { f: f32 = 0; }
   class S { materials: StaticArray<M> = new StaticArray<M>(1); }
   function getM(s: S): StaticArray<M> { return s.materials; }
   export function f(x: f32): f32 {
     let s = new S();
     let m = M();
     getM(s)[0].f = x; // accepted
     getM(s)[0] = m;   // error: indexed stores of value classes only work on StaticArray
     return s.materials[0].f;
   }
   ```
   Workaround: bind the array to a local first (`b3Shape_SetSurfaceMaterial`, `b3Shape_SetMeshMaterial`).
2. A nested conditional of float literals does not take the contextual type:
   ```as
   export function f(t: i32): f32 {
     let a: f32 = t == 0 ? 0.0 : 24.0;                 // fine
     let b: f32 = t == 0 ? 0.0 : t == 1 ? 24.0 : -18.0; // error: cannot implicitly convert f64 to f32
     return a + b;
   }
   ```
   Workaround: `<f32>` on each literal.
3. Not a bug, relied on: a leading underscore silences the unused parameter warning
   (`_triangleIndex` in `b3MeshTimeOfImpactFcn`, C's `B3_UNUSED`).
