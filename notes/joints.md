# joints: joint.c and every *_joint.c

## Ported

| C | AnimaScript |
|---|---|
| `src/joint.c` | `lib/joint.as`, below the declarations |
| `src/distance_joint.c` | `lib/distance_joint.as` |
| `src/motor_joint.c` | `lib/motor_joint.as` |
| `src/parallel_joint.c` | `lib/parallel_joint.as` |
| `src/prismatic_joint.c` | `lib/prismatic_joint.as` |
| `src/revolute_joint.c` | `lib/revolute_joint.as` |
| `src/spherical_joint.c` | `lib/spherical_joint.as` |
| `src/weld_joint.c` | `lib/weld_joint.as` |
| `src/wheel_joint.c` | `lib/wheel_joint.as` |

Every function in C order except the skipped ones below: joint creation for all nine types (filter
included), destroy, the contacts-between-bodies sweep, set placement (disabled, static, awake via
`b3CreateJointInGraph`, sleeping with the set merge), island linking, every `b3Joint_*` and per type
`b3XJoint_*` getter and setter, `b3GetJointReaction`, the force and torque getters, linear and
angular separation, and prepare / warm start / solve for every type plus the overflow loops. The
joint and explosion defaults are re-exported from `lib/types.as`.

Joint events are built in solver.c (`b3GetJointReaction` is the joint side); world.c reads them.

## Signatures that differ from C

- `b3Joint*` is the joint index (`jointId: i32`), as notes/decls.md says:
  `b3GetJointSim(world, jointId: i32)`, `b3DestroyJointInternal(world, jointId: i32, wakeBodies)`,
  `b3GetJointFullId` returns the index.
- `b3JointPair` (static in C) is a small class `{ joint: i32, jointSim: b3JointSim }`.
  `b3MakeJointId(world, pair)` (port only, unexported) builds the id the nine create functions
  return, since `b3JointId` has no positional constructor.
- Defs pass by value (`def: b3RevoluteJointDef`); `B3_CHECK_JOINT_DEF` is
  `B3_ASSERT(def.base.internalValue == B3_SECRET_COOKIE)`.
- `float*` out pairs are `ref f32`: `b3Joint_GetConstraintTuning(jointId, hertz, dampingRatio)`,
  `b3DistanceJoint_GetSpringForceRange(jointId, lower, upper)`, `b3GetJointReaction(..., force,
  torque)`. Callers pass `out[0]`, `out[1]` of a `FixedArray<f32, 2>`.
- `b3Joint_SetUserData` / `GetUserData` take and return `u64`.
- `b3DestroyJoint`: C tests `world == NULL` after `b3GetWorld`, which never returns NULL; dropped.
  The unlocked getters return `b3World?` and the port tests them as C does.
- `b3PrismaticJoint_GetSpeed` and `b3WheelJoint_GetSpinSpeed`: `b3GetBodyState` returns an index into
  the awake set's states (`B3_NULL_INDEX` for NULL), so C's `stateA ? stateA->v : zero` is an index
  test.

## Port shapes

- `ref joint = base.revoluteJoint;` everywhere C takes `&base->revoluteJoint`.
- Body states: `let stateA = joint.indexA == B3_NULL_INDEX ? b3_identityBodyState :
  states[joint.indexA]` (a copy). C writes velocities under `if (stateA->flags & b3_dynamicFlag)`;
  the port writes `states[joint.indexA].linearVelocity = vA` inside the same test. The identity
  state has no flags, so the index is never null there.
- `b3WarmStartMotorJoint` is the one place C writes states without the flags test (it writes
  kinematic body states too). The port writes through `states[index]` when the index is not null;
  C's writes to the dummy were discarded. The parity driver covers it with a kinematic body.
- Body and sim reads in prepare copy the struct once (read only).
- In `b3CreateJoint` every body and joint write goes through `world.bodies[id]` / `world.joints[id]`
  places; set indices are re-read after `b3WakeSolverSet` and `b3MergeSolverSets`, and the merged
  sim is re-fetched as C does.
- `b3DestroyJointInternal` copies the joint's two edges first. Another joint's edges never alias
  them (bodyA != bodyB), so the copies stay valid while the neighbours are relinked.
- Per step functions guard their asserts with `if (B3_ASSERT_ENABLED)`; API functions use plain
  `B3_ASSERT`.

## Skipped

- Debug draw: `b3DrawJoint`, `b3DrawDistanceJoint`, `b3DrawParallelJoint`, `b3DrawPrismaticJoint`,
  `b3DrawRevoluteJoint`, `b3DrawSphericalJoint`, `b3DrawWeldJoint`, `b3DrawWheelJoint`.
- Recording: every `B3_REC` / `B3_REC_CREATE` line. Setters whose only use of the world was the
  recording no longer look it up (so C's world validity assert in them is gone too).
- `#if 0` code: `b3Joint_SetSpringRotationTarget`, `b3WheelJoint_Dump`.
- Validation: `b3ValidateSolverSets` calls (B3_VALIDATE).
- Tracy zones in the overflow loops.

## Calls into other modules

All exist now with the predicted signatures (checked against the live tree): `b3GetWorld`,
`b3GetUnlockedWorld`, `b3GetUnlockedWorldFromId` (physics_world), `b3GetBodyFullId`, `b3MakeBodyId`,
`b3GetBodyTransform`, `b3WakeBody`, `b3GetBodySim`, `b3GetBodyState` (body), `b3DestroyContact`
(contact), `b3WakeSolverSet`, `b3MergeSolverSets` (solver_set), `b3LinkJoint`, `b3UnlinkJoint`
(island), `b3CreateJointInGraph`, `b3RemoveJointFromGraph` (constraint_graph),
`b3BroadPhase_MarkProxyMovedSerial` (broad_phase), `b3AllocId`, `b3FreeId`, `b3Array_RemoveSwap`.

## Hazards found

- Upstream quirk kept verbatim: `b3GetWheelJointForce` builds the axial component as
  `lowerSuspensionLimit + upperSuspensionImpulse + suspensionSpringImpulse` (a limit where an
  impulse is meant, and the upper impulse added). `b3GetPrismaticJointForce` likewise adds
  `upperImpulse`. Worth reporting upstream; ported as is for parity.
- `b3WarmStartMotorJoint` writes kinematic states (above).
- `b3Joint_GetAngularSeparation` asserts false for the wheel joint; the driver skips it there.

## Verification

- `rasc check` on all nine files against the working tree (stub runtime): 0 errors, 0 warnings.
  Every callee exists with the predicted signature. A planted error confirmed the check reaches
  these files. Compiled to wat under `--runtime frame`, no joint function contains an `f64`
  instruction.
- `verify/run.sh joints`: 10760 lines, 174674 values, 0 mismatches, bit exact against the scalar C
  build. It also passes with `RASC_FLAGS=` (no `-O`) and with `-D B3_ENABLE_ASSERT` (no assert fires).
  Files: `verify/cref/joints.c`, `verify/as/joints.as`, `verify/joints.mjs`. A collection runs
  after every step, so the world is moved mid run.
- The scene is one world holding every joint type:
  - rigid and soft distance joints, with limit and motor
  - motor joints to a static body and to a kinematic one
  - parallel, prismatic on a tilted axis, revolute
  - spherical with cone, twist, motor and spring
  - rigid and soft weld
  - wheel with suspension, spin motor and steering
  - a filter joint
- 130 steps, with a script that exercises:
  - setters at steps 40, 50 and 70
  - collideConnected toggles and `SetLocalFrameA`
  - destroy, with and without waking
  - at step 90, new joints between two bodies asleep in separate sets (the sets merge), between a
    sleeping body and the static ground, and onto a disabled body
  - `b3Joint_WakeBodies`
  - warm starting off for steps 100 to 104
  - joint events through force and torque thresholds
- Each step prints the events, every public getter, `b3GetJointReaction`, the internal joint sim
  (impulses, masses, axes, softness) and every body's motion.
- Limit coverage, from nonzero impulse counts:
  - engaged: distance upper, prismatic lower and upper, revolute lower and upper, spherical lower
    and upper twist and cone, wheel upper suspension and upper steering
  - never engaged: wheel lower suspension and wheel lower steering. Steering is disabled at step
    50, before the -0.9 target reaches the limit.

## rasc findings

No compiler or std changes, no new refusals. Observation: `rasc check` does not flag an f64 literal
local (`let x = 1.5;` is only "unused"); f64 leaks show only in review or the wat. A constant
`1.0 / 60.0` passed as an f32 argument is the known f64 constant expression (types finding 2); the
driver divides at f32.
