# Upstream Box3D issues

Found while porting against `f36d061`. The port reproduces each one exactly,
since it stays faithful; the fix belongs upstream.

## b3CreateHull crashes or hangs on near coplanar clouds

2D points uniform in [-1, 1]^2 with z uniform in [-1e-5, 1e-5]: the release
build segfaults in `b3NewellPlane` on some seeds and never returns on others.
A debug build asserts at hull.c:544 (`twin->twin == edge`, reached from
`b3HullBuilder_ConnectEdges` in `ConnectFaces` in `MergeCoplanar`) or at
hull.c:1093 (`b3CheckConsistency` after `ConnectFaces`). In the hangs
`ConnectEdges` merges a face with itself. Repro and seeds in `notes/hull.md`.
Anyone building hulls from user or asset data can hit it.

## Joint force getters that look wrong

- `b3GetWheelJointForce` sums `lowerSuspensionLimit + upperSuspensionImpulse +
  suspensionSpringImpulse`, adding a limit (a length) to impulses; the lower
  suspension impulse was probably meant.
- `b3GetPrismaticJointForce` adds `upperImpulse` where the other joints
  subtract the upper limit impulse.

Both only affect the reported force, not the simulation.

## Stale bodyMoveIndex writes past the move event array

`b3Body_Disable`, a step, then `b3Body_Enable` and `b3Body_SetAwake(false)`:
`b3TrySleepIsland` uses a `bodyMoveIndex` past the end of
`world->bodyMoveEvents`. A release build writes past the count (harmless while
inside capacity) or marks another body's event when the index is below the
count. The port guards the write with a bounds check that matches C's visible
result. Repro in `verify/cref/bodyshape.c`.

## b3Body_SetType returns with the world locked

When the body has a compound or height field shape, the early return skips
the unlock.

## b3Shape_SetFilter always destroys the proxy

`destroyProxy` is computed after the new filter is assigned, so it is always
true.

## b3Shape_ApplyWind loses the force on a sleeping body

It keeps the body sim pointer across the wake, so the force lands in the
sleeping set that the wake destroys.
