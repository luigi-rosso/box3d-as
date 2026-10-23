// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box3d/id.h, include/box3d/types.h and src/types.c
// The query, tree, mover, geometry and collision groups of types.h live in lib/collision.as.

import { B3_SECRET_COOKIE, FLT_MAX, UINT64_MAX, b3GetLengthUnitsPerMeter } from "./core";
import { B3_HUGE } from "./constants";
import { b3Pos, b3Quat, b3Quat_identity, b3Transform, b3Vec3, b3WorldTransform } from "./math_functions";
import { b3Manifold, b3PlaneResult } from "./collision";

// id.h

/// World id references a world instance. This should be treated as an opaque handle.
export struct b3WorldId {
	index1: u16 = 0;
	generation: u16 = 0;
}

/// Body id references a body instance. This should be treated as an opaque handle.
export struct b3BodyId {
	index1: i32 = 0;
	world0: u16 = 0;
	generation: u16 = 0;
}

/// Shape id references a shape instance. This should be treated as an opaque handle.
export struct b3ShapeId {
	index1: i32 = 0;
	world0: u16 = 0;
	generation: u16 = 0;
}

/// Joint id references a joint instance. This should be treated as an opaque handle.
export struct b3JointId {
	index1: i32 = 0;
	world0: u16 = 0;
	generation: u16 = 0;
}

/// Contact id references a contact instance. This should be treated as an opaque handle.
export struct b3ContactId {
	index1: i32 = 0;
	world0: u16 = 0;
	padding: i16 = 0;
	generation: u32 = 0;
}

/// Use these to make your identifiers null.
/// You may also use zero initialization to get null.
export const b3_nullWorldId: b3WorldId = b3WorldId();
export const b3_nullBodyId: b3BodyId = b3BodyId();
export const b3_nullShapeId: b3ShapeId = b3ShapeId();
export const b3_nullJointId: b3JointId = b3JointId();
export const b3_nullContactId: b3ContactId = b3ContactId();

// B3_IS_NULL( id ) is id.index1 == 0 and B3_IS_NON_NULL( id ) is id.index1 != 0.
// B3_ID_EQUALS( id1, id2 ) is id1 == id2: integer structs compare every field.

/// Store a world id into a uint32_t.
export function b3StoreWorldId(id: b3WorldId): u32 {
	return (<u32>id.index1 << 16) | <u32>id.generation;
}

/// Load a uint32_t into a world id.
export function b3LoadWorldId(x: u32): b3WorldId {
	let id: b3WorldId;
	id.index1 = <u16>(x >> 16);
	id.generation = <u16>x;
	return id;
}

/// Store a body id into a uint64_t.
export function b3StoreBodyId(id: b3BodyId): u64 {
	return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

/// Load a uint64_t into a body id.
export function b3LoadBodyId(x: u64): b3BodyId {
	let id: b3BodyId;
	id.index1 = <i32>(x >> 32);
	id.world0 = <u16>(x >> 16);
	id.generation = <u16>x;
	return id;
}

/// Store a shape id into a uint64_t.
export function b3StoreShapeId(id: b3ShapeId): u64 {
	return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

/// Load a uint64_t into a shape id.
export function b3LoadShapeId(x: u64): b3ShapeId {
	let id: b3ShapeId;
	id.index1 = <i32>(x >> 32);
	id.world0 = <u16>(x >> 16);
	id.generation = <u16>x;
	return id;
}

/// Store a joint id into a uint64_t.
export function b3StoreJointId(id: b3JointId): u64 {
	return (<u64>id.index1 << 32) | (<u64>id.world0 << 16) | <u64>id.generation;
}

/// Load a uint64_t into a joint id.
export function b3LoadJointId(x: u64): b3JointId {
	let id: b3JointId;
	id.index1 = <i32>(x >> 32);
	id.world0 = <u16>(x >> 16);
	id.generation = <u16>x;
	return id;
}

/// Store a contact id into three uint32 values
export function b3StoreContactId(id: b3ContactId, values: StaticArray<u32>): void {
	values[0] = <u32>id.index1;
	values[1] = <u32>id.world0;
	values[2] = id.generation;
}

/// Load a contact id from three uint32 values.
export function b3LoadContactId(values: StaticArray<u32>): b3ContactId {
	let id: b3ContactId;
	id.index1 = <i32>values[0];
	id.world0 = <u16>values[1];
	id.padding = 0;
	id.generation = values[2];
	return id;
}

// types.h

export const B3_DEFAULT_CATEGORY_BITS: u64 = UINT64_MAX;
export const B3_DEFAULT_MASK_BITS: u64 = UINT64_MAX;

// Callbacks drop C's void* context; the closure carries it. Debug draw and the task
// system are skipped, so their callback types only keep the def fields compiling.

/// Task interface
export type b3TaskCallback = (taskContext: u64) => void;

/// These functions can be provided to Box3D to invoke a task system.
export type b3EnqueueTaskCallback = (task: b3TaskCallback, taskContext: u64, userContext: u64, taskName: string) => u64;

/// Finishes a user task object that wraps a Box3D task. This must block until the task has completed.
export type b3FinishTaskCallback = (userTask: u64, userContext: u64) => void;

/// Debug shape creation. b3DebugShape is skipped, so the shape arrives as 0.
export type b3CreateDebugShapeCallback = (debugShape: u64, userContext: u64) => u64;
export type b3DestroyDebugShapeCallback = (userShape: u64, userContext: u64) => void;

/// Optional friction mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// @warning This function should not attempt to modify Box3D state or user application state.
export type b3FrictionCallback = (frictionA: f32, userMaterialIdA: u64, frictionB: f32, userMaterialIdB: u64) => f32;

/// Optional restitution mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// @warning This function should not attempt to modify Box3D state or user application state.
export type b3RestitutionCallback = (
	restitutionA: f32,
	userMaterialIdA: u64,
	restitutionB: f32,
	userMaterialIdB: u64,
) => f32;

/// Prototype for a contact filter callback.
/// This is called when a contact pair is considered for collision. This allows you to
/// perform custom logic to prevent collision between shapes. This is only called if
/// one of the two shapes has custom filtering enabled. @see b3ShapeDef.
/// Return false if you want to disable the collision
/// @warning Do not attempt to modify the world inside this callback
export type b3CustomFilterFcn = (shapeIdA: b3ShapeId, shapeIdB: b3ShapeId) => bool;

/// Prototype for a pre-solve callback.
/// This is called after a contact is updated. This allows you to inspect a
/// collision before it goes to the solver.
/// Return false if you want to disable the contact this step
/// @warning Do not attempt to modify the world inside this callback
export type b3PreSolveFcn = (shapeIdA: b3ShapeId, shapeIdB: b3ShapeId, point: b3Pos, normal: b3Vec3) => bool;

/// Prototype callback for overlap queries.
/// Called for each shape found in the query.
/// @return false to terminate the query.
export type b3OverlapResultFcn = (shapeId: b3ShapeId) => bool;

/// Prototype callback for ray casts.
/// Called for each shape found in the query. You control how the ray cast
/// proceeds by returning a float:
/// return -1: ignore this shape and continue
/// return 0: terminate the ray cast
/// return fraction: clip the ray to this point
/// return 1: don't clip the ray and continue
/// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
export type b3CastResultFcn = (
	shapeId: b3ShapeId,
	point: b3Pos,
	normal: b3Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
) => f32;

/// Optional world capacities that can be use to avoid run-time allocations
export struct b3Capacity {
	/// Number of expected static shapes.
	staticShapeCount: i32 = 0;

	/// Number of expected dynamic and kinematic shapes.
	dynamicShapeCount: i32 = 0;

	/// Number of expected static bodies.
	staticBodyCount: i32 = 0;

	/// Number of expected dynamic and kinematic bodies.
	dynamicBodyCount: i32 = 0;

	/// Number of expected contacts.
	contactCount: i32 = 0;
}

/// World definition used to create a simulation world. Must be initialized using b3DefaultWorldDef.
export class b3WorldDef {
	/// Gravity vector. Box3D has no up-vector defined.
	gravity: b3Vec3 = b3Vec3();

	/// Restitution speed threshold, usually in m/s. Collisions above this
	/// speed have restitution applied (will bounce).
	restitutionThreshold: f32 = 0;

	/// Number of iterations of the restitution solver. More iterations can lead to less box spinning.
	/// @see B3_MAX_RESTITUTION_ITERATIONS
	restitutionIterations: i32 = 0;

	/// Enable full contact propagation in the restitution solver. Expensive.
	enableRestitutionPropagation: bool = false;

	/// Hit event speed threshold, usually in m/s. Collisions above this
	/// speed can generate hit events if the shape also enables hit events.
	hitEventThreshold: f32 = 0;

	/// Contact stiffness. Cycles per second. Increasing this increases the speed of overlap recovery, but can introduce jitter.
	contactHertz: f32 = 0;

	/// Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this with
	/// the trade-off that overlap resolution becomes more energetic.
	contactDampingRatio: f32 = 0;

	/// This parameter controls how fast overlap is resolved and usually has units of meters per second. This only
	/// puts a cap on the resolution speed. The resolution speed is increased by increasing the hertz and/or
	/// decreasing the damping ratio.
	contactSpeed: f32 = 0;

	/// Maximum linear speed. Usually meters per second.
	maximumLinearSpeed: f32 = 0;

	/// Use this to disable gyroscopic torque on sphere-like objects automatically.
	/// Range is [0, 1]. A sphere is 0 and a 1x1x2 box is 0.5.
	isotropicInertiaTolerance: f32 = 0;

	/// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
	frictionCallback: b3FrictionCallback? = null;

	/// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
	restitutionCallback: b3RestitutionCallback? = null;

	/// Can bodies go to sleep to improve performance
	enableSleep: bool = false;

	/// Enable continuous collision
	enableContinuous: bool = false;

	/// Number of workers to use with the provided task system. The port runs single threaded.
	workerCount: u32 = 0;

	/// function to spawn task
	enqueueTask: b3EnqueueTaskCallback? = null;

	/// function to finish a task
	finishTask: b3FinishTaskCallback? = null;

	/// User context that is provided to enqueueTask and finishTask
	userTaskContext: u64 = 0;

	/// User data associated with a world
	userData: u64 = 0;

	/// Used to create debug draw shapes. This is called when a shape is
	/// first drawn using b3DebugDraw.
	createDebugShape: b3CreateDebugShapeCallback? = null;

	/// Used to destroy debug draw shapes. This is called when a shape is modified or destroyed.
	destroyDebugShape: b3DestroyDebugShapeCallback? = null;

	/// This is passed to the debug shape callbacks to provide a user context.
	userDebugShapeContext: u64 = 0;

	/// Optional initial capacities
	capacity: b3Capacity = b3Capacity();

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// The body simulation type.
/// Each body is one of these three types. The type determines how the body behaves in the simulation.
export type b3BodyType = i32;

/// zero mass, zero velocity, may be manually moved
export const b3_staticBody: b3BodyType = 0;

/// zero mass, velocity set by user, moved by solver
export const b3_kinematicBody: b3BodyType = 1;

/// positive mass, velocity determined by forces, moved by solver
export const b3_dynamicBody: b3BodyType = 2;

/// number of body types
export const b3_bodyTypeCount: b3BodyType = 3;

/// Motion locks to restrict the body movement
export struct b3MotionLocks {
	/// Prevent translation along the x-axis
	linearX: bool = false;

	/// Prevent translation along the y-axis
	linearY: bool = false;

	/// Prevent translation along the z-axis
	linearZ: bool = false;

	/// Prevent rotation around the x-axis
	angularX: bool = false;

	/// Prevent rotation around the y-axis
	angularY: bool = false;

	/// Prevent rotation around the z-axis
	angularZ: bool = false;
}

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
/// Body definitions are temporary objects used to bundle creation parameters.
/// Must be initialized using b3DefaultBodyDef().
export class b3BodyDef {
	/// The body type: static, kinematic, or dynamic.
	type: b3BodyType = 0;

	/// The initial world position of the body. Bodies should be created with the desired position.
	position: b3Pos = b3Vec3();

	/// The initial world rotation of the body.
	rotation: b3Quat = b3Quat();

	/// The initial linear velocity of the body's origin. Usually in meters per second.
	linearVelocity: b3Vec3 = b3Vec3();

	/// The initial angular velocity of the body. Radians per second.
	angularVelocity: b3Vec3 = b3Vec3();

	/// Linear damping is used to reduce the linear velocity.
	linearDamping: f32 = 0;

	/// Angular damping is used to reduce the angular velocity.
	angularDamping: f32 = 0;

	/// Scale the gravity applied to this body. Non-dimensional.
	gravityScale: f32 = 0;

	/// Sleep speed threshold, default is 0.05 meters per second
	sleepThreshold: f32 = 0;

	/// Continuous collision safety factor. Non-dimensional. Recommended range [0.01, 0.5].
	safetyFactor: f32 = 0;

	/// Optional body name for debugging. Empty for none.
	name: string = "";

	/// Use this to store application specific body data.
	userData: u64 = 0;

	/// Motions locks to restrict linear and angular movement
	motionLocks: b3MotionLocks = b3MotionLocks();

	/// Set this flag to false if this body should never fall asleep.
	enableSleep: bool = false;

	/// Is this body initially awake or sleeping?
	isAwake: bool = false;

	/// Treat this body as a high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	isBullet: bool = false;

	/// Used to disable a body. A disabled body does not move or collide.
	isEnabled: bool = false;

	/// This allows this body to bypass rotational speed limits. Should only be used
	/// for circular objects, like wheels.
	allowFastRotation: bool = false;

	/// Enable contact recycling. True by default. Leaving this enabled improves performance
	/// but may lead to ghost collision that should be avoided on characters.
	enableContactRecycling: bool = false;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// This is used to filter collision on shapes. It affects shape-vs-shape collision
/// and shape-versus-query collision (such as b3World_CastRay).
export struct b3Filter {
	/// The collision category bits. Normally you would just set one bit. The category bits should
	/// represent your application object types.
	categoryBits: u64 = 0;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	maskBits: u64 = 0;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	/// always wins against the mask bits.
	groupIndex: i32 = 0;

	constructor(categoryBits: u64, maskBits: u64, groupIndex: i32) {
		this.categoryBits = categoryBits;
		this.maskBits = maskBits;
		this.groupIndex = groupIndex;
	}
}

/// Material properties supported per triangle on meshes and height fields
export struct b3SurfaceMaterial {
	/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	friction: f32 = 0;

	/// The coefficient of restitution (bounce) usually in the range [0,1].
	/// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	restitution: f32 = 0;

	/// The rolling resistance usually in the range [0,1]. This is only used for spheres and capsules.
	rollingResistance: f32 = 0;

	/// The tangent velocity for conveyor belts. This is local to the shape and will be projected
	/// onto the contact surface.
	tangentVelocity: b3Vec3 = b3Vec3();

	/// User material identifier. This is passed with query results and to friction and restitution
	/// combining functions. It is not used internally.
	userMaterialId: u64 = 0;

	/// Custom debug draw color. Ignored if 0. The low 24 bits are RGB.
	customColor: u32 = 0;

	/// Explicit padding. Must be zero.
	padding: u32 = 0;
}

/// Shape type
export type b3ShapeType = i32;

/// A capsule is an extruded sphere
export const b3_capsuleShape: b3ShapeType = 0;

/// A baked compound shape composed of spheres, capsules, hulls, and meshes
export const b3_compoundShape: b3ShapeType = 1;

/// A height field useful for terrain
export const b3_heightShape: b3ShapeType = 2;

/// A convex hull
export const b3_hullShape: b3ShapeType = 3;

/// A triangle soup
export const b3_meshShape: b3ShapeType = 4;

/// A sphere with an offset
export const b3_sphereShape: b3ShapeType = 5;

/// The number of shape types
export const b3_shapeTypeCount: b3ShapeType = 6;

/// Used to create a shape
export class b3ShapeDef {
	/// Optional shape name for debugging. Empty for none.
	name: string = "";

	/// Use this to store application specific shape data.
	userData: u64 = 0;

	/// Surface material used on mesh shapes per triangle. Ignored for convex shapes. Ignored for compound shapes.
	materials: StaticArray<b3SurfaceMaterial>? = null;

	/// Surface material count.
	materialCount: i32 = 0;

	/// The base surface material. Ignored for compound shapes.
	baseMaterial: b3SurfaceMaterial = b3SurfaceMaterial();

	/// The density, usually in kg/m^3.
	density: f32 = 0;

	/// Explosion scale for b3World_Explode. non-dimensional
	explosionScale: f32 = 0;

	/// Contact filtering data.
	filter: b3Filter = b3Filter();

	/// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See b3WorldDef.
	enableCustomFiltering: bool = false;

	/// A sensor shape generates overlap events but never generates a collision response.
	isSensor: bool = false;

	/// Enable sensor events for this shape. This applies to sensors and non-sensors. False by default, even for sensors.
	enableSensorEvents: bool = false;

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	enableContactEvents: bool = false;

	/// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	enableHitEvents: bool = false;

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	///	and must be carefully handled due to multithreading. Ignored for sensors.
	enablePreSolveEvents: bool = false;

	/// When shapes are created they will scan the environment for collision the next time step.
	invokeContactCreation: bool = false;

	/// Should the body update the mass properties when this shape is created. Default is true.
	updateBodyMass: bool = false;

	/// Enable speculative collision.
	enableSpeculativeContact: bool = false;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// Profiling data. Times are in milliseconds. Profiling timers are skipped, so these read 0.
export struct b3Profile {
	step: f32 = 0;
	pairs: f32 = 0;
	collide: f32 = 0;
	solve: f32 = 0;
	solverSetup: f32 = 0;
	constraints: f32 = 0;
	prepareConstraints: f32 = 0;
	integrateVelocities: f32 = 0;
	warmStart: f32 = 0;
	solveImpulses: f32 = 0;
	integratePositions: f32 = 0;
	relaxImpulses: f32 = 0;
	restitution: f32 = 0;
	storeImpulses: f32 = 0;
	splitIslands: f32 = 0;
	transforms: f32 = 0;
	sensorHits: f32 = 0;
	jointEvents: f32 = 0;
	hitEvents: f32 = 0;
	refit: f32 = 0;
	bullets: f32 = 0;
	sleepIslands: f32 = 0;
	sensors: f32 = 0;
}

/// Counters that give details of the simulation size.
export struct b3Counters {
	byteCount: i64 = 0;
	bodyCount: i32 = 0;
	shapeCount: i32 = 0;
	contactCount: i32 = 0;
	jointCount: i32 = 0;
	islandCount: i32 = 0;
	stackUsed: i32 = 0;
	arenaCapacity: i32 = 0;
	staticTreeHeight: i32 = 0;
	treeHeight: i32 = 0;
	satCallCount: i32 = 0;
	satCacheHitCount: i32 = 0;
	taskCount: i32 = 0;
	colorCounts: FixedArray<i32, 24>;
	manifoldCounts: FixedArray<i32, 8>;

	/// Number of contacts touched by the collide pass
	/// graph contacts + awake-set non-touching
	awakeContactCount: i32 = 0;

	/// Number of contacts recycled in the most recent step.
	recycledContactCount: i32 = 0;

	/// Maximum number of time of impact outer iterations
	distanceIterations: i32 = 0;

	/// Maximum TOI push backs.
	pushBackIterations: i32 = 0;

	/// Maximum TOI root solver iterations.
	rootIterations: i32 = 0;
}

/// Joint type enumeration. This is useful because all joint types use b3JointId and sometimes you
/// want to get the type of a joint.
export type b3JointType = i32;
export const b3_parallelJoint: b3JointType = 0;
export const b3_distanceJoint: b3JointType = 1;
export const b3_filterJoint: b3JointType = 2;
export const b3_motorJoint: b3JointType = 3;
export const b3_prismaticJoint: b3JointType = 4;
export const b3_revoluteJoint: b3JointType = 5;
export const b3_sphericalJoint: b3JointType = 6;
export const b3_weldJoint: b3JointType = 7;
export const b3_wheelJoint: b3JointType = 8;

/// Base joint definition used by all joint types. The local frames are measured from the
/// body's origin rather than the center of mass because:
/// 1. You might not know where the center of mass will be.
/// 2. If you add/remove shapes from a body and recompute the mass, the joints will be broken.
export struct b3JointDef {
	/// User data pointer
	userData: u64 = 0;

	/// The first attached body
	bodyIdA: b3BodyId = b3BodyId();

	/// The second attached body
	bodyIdB: b3BodyId = b3BodyId();

	/// The first local joint frame
	localFrameA: b3Transform = b3Transform();

	/// The second local joint frame
	localFrameB: b3Transform = b3Transform();

	/// Force threshold for joint events
	forceThreshold: f32 = 0;

	/// Torque threshold for joint events
	torqueThreshold: f32 = 0;

	/// Constraint hertz (advanced feature)
	constraintHertz: f32 = 0;

	/// Constraint damping ratio (advanced feature)
	constraintDampingRatio: f32 = 0;

	/// Debug draw scale
	drawScale: f32 = 0;

	/// Set this flag to true if the attached bodies should collide
	collideConnected: bool = false;

	/// Used internally to detect a valid definition. DO NOT SET.
	internalValue: i32 = 0;
}

/// Distance joint definition.
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs.
export struct b3DistanceJointDef {
	/// Base joint definition
	base: b3JointDef = b3JointDef();

	/// The rest length of this joint. Clamped to a stable minimum value.
	length: f32 = 0;

	/// Enable the distance constraint to behave like a spring. If false
	/// then the distance joint will be rigid, overriding the limit and motor.
	enableSpring: bool = false;

	/// The lower spring force controls how much tension it can sustain
	lowerSpringForce: f32 = 0;

	/// The upper spring force controls how much compression it can sustain
	upperSpringForce: f32 = 0;

	/// The spring linear stiffness Hertz, cycles per second
	hertz: f32 = 0;

	/// The spring linear damping ratio, non-dimensional
	dampingRatio: f32 = 0;

	/// Enable/disable the joint limit
	enableLimit: bool = false;

	/// Minimum length. Clamped to a stable minimum value.
	minLength: f32 = 0;

	/// Maximum length. Must be greater than or equal to the minimum length.
	maxLength: f32 = 0;

	/// Enable/disable the joint motor
	enableMotor: bool = false;

	/// The maximum motor force, usually in newtons
	maxMotorForce: f32 = 0;

	/// The desired motor speed, usually in meters per second
	motorSpeed: f32 = 0;
}

/// A motor joint is used to control the relative position and velocity between two bodies.
export struct b3MotorJointDef {
	/// Base joint definition
	base: b3JointDef = b3JointDef();

	/// The desired linear velocity
	linearVelocity: b3Vec3 = b3Vec3();

	/// The maximum motor force in newtons
	maxVelocityForce: f32 = 0;

	/// The desired angular velocity
	angularVelocity: b3Vec3 = b3Vec3();

	/// The maximum motor torque in newton-meters
	maxVelocityTorque: f32 = 0;

	/// Linear spring hertz for position control
	linearHertz: f32 = 0;

	/// Linear spring damping ratio
	linearDampingRatio: f32 = 0;

	/// Maximum spring force in newtons
	maxSpringForce: f32 = 0;

	/// Angular spring hertz for position control
	angularHertz: f32 = 0;

	/// Angular spring damping ratio
	angularDampingRatio: f32 = 0;

	/// Maximum spring torque in newton-meters
	maxSpringTorque: f32 = 0;
}

/// A filter joint is used to disable collision between two specific bodies.
export struct b3FilterJointDef {
	/// Base joint definition
	base: b3JointDef = b3JointDef();
}

/// Parallel joint definition. Constrains the angle between axis z in body A and axis z in body B
/// using a spring. Useful to keep a body upright.
export struct b3ParallelJointDef {
	/// Base joint definition
	base: b3JointDef = b3JointDef();

	/// The spring stiffness Hertz, cycles per second
	hertz: f32 = 0;

	/// The spring damping ratio, non-dimensional
	dampingRatio: f32 = 0;

	/// The maximum spring torque, typically in newton-meters.
	maxTorque: f32 = 0;
}

/// Prismatic joint definition. Body B may slide along the x-axis in local frame A.
/// Body B cannot rotate relative to body A. The joint translation is zero when the
/// local frame origins coincide in world space.
export struct b3PrismaticJointDef {
	/// Base joint definition
	base: b3JointDef = b3JointDef();

	/// Enable a linear spring along the prismatic joint axis
	enableSpring: bool = false;

	/// The spring stiffness Hertz, cycles per second
	hertz: f32 = 0;

	/// The spring damping ratio, non-dimensional
	dampingRatio: f32 = 0;

	/// The target translation for the joint in meters. The spring-damper will drive
	/// to this translation.
	targetTranslation: f32 = 0;

	/// Enable/disable the joint limit
	enableLimit: bool = false;

	/// The lower translation limit
	lowerTranslation: f32 = 0;

	/// The upper translation limit
	upperTranslation: f32 = 0;

	/// Enable/disable the joint motor
	enableMotor: bool = false;

	/// The maximum motor force, typically in newtons
	maxMotorForce: f32 = 0;

	/// The desired motor speed, typically in meters per second
	motorSpeed: f32 = 0;
}

/// Revolute joint definition. A point on body B is fixed to a point on body A.
/// Allows relative rotation about the z-axis.
export struct b3RevoluteJointDef {
	/// Base joint definition.
	base: b3JointDef = b3JointDef();

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	/// This defines the zero angle for the joint limit.
	targetAngle: f32 = 0;

	/// Enable a rotational spring on the revolute hinge axis.
	enableSpring: bool = false;

	/// The spring stiffness Hertz, cycles per second.
	hertz: f32 = 0;

	/// The spring damping ratio, non-dimensional.
	dampingRatio: f32 = 0;

	/// A flag to enable joint limits.
	enableLimit: bool = false;

	/// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
	lowerAngle: f32 = 0;

	/// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
	upperAngle: f32 = 0;

	/// A flag to enable the joint motor.
	enableMotor: bool = false;

	/// The maximum motor torque, typically in newton-meters.
	maxMotorTorque: f32 = 0;

	/// The desired motor speed in radians per second.
	motorSpeed: f32 = 0;
}

/// Spherical joint definition. A point on body B is fixed to a point on body A.
/// Allows rotation about the shared point.
export struct b3SphericalJointDef {
	/// Base joint definition
	base: b3JointDef = b3JointDef();

	/// Enable a rotational spring that attempts to align the two joint frames.
	enableSpring: bool = false;

	/// The spring stiffness Hertz, cycles per second. This may be clamped internally
	/// according to the time step to maintain stability. Non-negative number.
	hertz: f32 = 0;

	/// The spring damping ratio, non-dimensional. Non-negative number.
	dampingRatio: f32 = 0;

	/// Target spring rotation, joint frame B relative to joint frame A.
	targetRotation: b3Quat = b3Quat();

	/// A flag to enable the cone limit. The cone is centered on the frameA z-axis.
	enableConeLimit: bool = false;

	/// The angle for the cone limit in radians. Valid range is [0, pi]
	coneAngle: f32 = 0;

	/// A flag to enable the twist limit. The twist is centered on the frameB z-axis.
	enableTwistLimit: bool = false;

	/// The angle for the lower twist limit in radians. Minimum of -0.99*pi radians.
	lowerTwistAngle: f32 = 0;

	/// The angle for the upper twist limit in radians. Maximum of 0.99*pi radians.
	upperTwistAngle: f32 = 0;

	/// A flag to enable the joint motor
	enableMotor: bool = false;

	/// The maximum motor torque, typically in newton-meters. Non-negative number.
	maxMotorTorque: f32 = 0;

	/// The desired motor angular velocity in radians per second. This is the relative angular
	/// velocity between the two bodies in world space.
	/// motorVelocity = angularVelocityB - angularVelocityA
	motorVelocity: b3Vec3 = b3Vec3();
}

/// Weld joint definition
/// Connects two bodies together rigidly. This constraint provides springs to mimic
/// soft-body simulation.
/// @note The approximate solver in Box3D cannot hold many bodies together rigidly
export struct b3WeldJointDef {
	/// Base joint definition
	base: b3JointDef = b3JointDef();

	/// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	linearHertz: f32 = 0;

	/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	angularHertz: f32 = 0;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	linearDampingRatio: f32 = 0;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	angularDampingRatio: f32 = 0;
}

/// Wheel joint definition
/// Body A is the chassis and body B is the wheel.
/// The wheel rotates around the local z-axis in frame B.
/// The wheel translates along the local x-axis in frame A.
/// The wheel can optionally steer along the x-axis in frame A.
export struct b3WheelJointDef {
	/// Base joint definition
	base: b3JointDef = b3JointDef();

	/// Enable a linear spring along the local axis
	enableSuspensionSpring: bool = false;

	/// Spring stiffness in Hertz
	suspensionHertz: f32 = 0;

	/// Spring damping ratio, non-dimensional
	suspensionDampingRatio: f32 = 0;

	/// Enable/disable the joint linear limit
	enableSuspensionLimit: bool = false;

	/// The lower suspension translation limit
	lowerSuspensionLimit: f32 = 0;

	/// The upper translation limit
	upperSuspensionLimit: f32 = 0;

	/// Enable/disable the joint rotational motor
	enableSpinMotor: bool = false;

	/// The maximum motor torque, typically in newton-meters
	maxSpinTorque: f32 = 0;

	/// The desired motor speed in radians per second
	spinSpeed: f32 = 0;

	/// Enable steering, otherwise the steering is fixed forward
	enableSteering: bool = false;

	/// Steering stiffness in Hertz
	steeringHertz: f32 = 0;

	/// Spring damping ratio, non-dimensional
	steeringDampingRatio: f32 = 0;

	/// The target steering angle in radians
	targetSteeringAngle: f32 = 0;

	/// The maximum steering torque in N*m
	maxSteeringTorque: f32 = 0;

	/// Enable/disable the steering angular limit
	enableSteeringLimit: bool = false;

	/// The lower steering angle in radians
	lowerSteeringLimit: f32 = 0;

	/// The upper steering angle in radians
	upperSteeringLimit: f32 = 0;
}

/// The explosion definition is used to configure options for explosions. Explosions
/// consider shape geometry when computing the impulse.
export struct b3ExplosionDef {
	/// Mask bits to filter shapes
	maskBits: u64 = 0;

	/// The center of the explosion in world space
	position: b3Pos = b3Vec3();

	/// The radius of the explosion
	radius: f32 = 0;

	/// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	falloff: f32 = 0;

	/// Impulse per unit area. This applies an impulse according to the shape area that
	/// is facing the explosion. Explosions only apply to spheres, capsules, and hulls. This
	/// may be negative for implosions.
	impulsePerArea: f32 = 0;
}

/**
 * @defgroup event Events
 * World event types. Event containers reference the world's event arrays; count fields
 * keep the C counts.
 */

/// A begin-touch event is generated when a shape starts to overlap a sensor shape.
export struct b3SensorBeginTouchEvent {
	/// The id of the sensor shape
	sensorShapeId: b3ShapeId = b3ShapeId();

	/// The id of the shape that began touching the sensor shape
	visitorShapeId: b3ShapeId = b3ShapeId();
}

/// An end touch event is generated when a shape stops overlapping a sensor shape.
///	Therefore you should always confirm the shape id is valid using b3Shape_IsValid.
export struct b3SensorEndTouchEvent {
	/// The id of the sensor shape
	///	@warning this shape may have been destroyed
	sensorShapeId: b3ShapeId = b3ShapeId();

	/// The id of the shape that stopped touching the sensor shape
	///	@warning this shape may have been destroyed
	visitorShapeId: b3ShapeId = b3ShapeId();
}

/// Sensor events are buffered in the world and are available
///	as begin/end overlap event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
export class b3SensorEvents {
	/// Array of sensor begin touch events
	beginEvents: Array<b3SensorBeginTouchEvent> = new Array<b3SensorBeginTouchEvent>(0);

	/// Array of sensor end touch events
	endEvents: Array<b3SensorEndTouchEvent> = new Array<b3SensorEndTouchEvent>(0);

	/// The number of begin touch events
	beginCount: i32 = 0;

	/// The number of end touch events
	endCount: i32 = 0;
}

/// A begin-touch event is generated when two shapes begin touching.
export struct b3ContactBeginTouchEvent {
	/// Id of the first shape
	shapeIdA: b3ShapeId = b3ShapeId();

	/// Id of the second shape
	shapeIdB: b3ShapeId = b3ShapeId();

	/// The transient contact id. This contact may be destroyed automatically when the world is modified or simulated.
	/// Use b3Contact_IsValid before using this id.
	contactId: b3ContactId = b3ContactId();
}

/// An end touch event is generated when two shapes stop touching.
export struct b3ContactEndTouchEvent {
	/// Id of the first shape
	///	@warning this shape may have been destroyed
	shapeIdA: b3ShapeId = b3ShapeId();

	/// Id of the first shape
	///	@warning this shape may have been destroyed
	shapeIdB: b3ShapeId = b3ShapeId();

	/// Id of the contact.
	///	@warning this contact may have been destroyed
	contactId: b3ContactId = b3ContactId();
}

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
/// This may be reported for speculative contacts that have a confirmed impulse.
export struct b3ContactHitEvent {
	/// Id of the first shape
	shapeIdA: b3ShapeId = b3ShapeId();

	/// Id of the second shape
	shapeIdB: b3ShapeId = b3ShapeId();

	/// Id of the contact.
	///	@warning this contact may have been destroyed
	contactId: b3ContactId = b3ContactId();

	/// Point where the shapes hit at the beginning of the time step.
	/// This is a mid-point between the two surfaces. It could be at speculative
	/// point where the two shapes were not touching at the beginning of the time step.
	point: b3Pos = b3Vec3();

	/// Normal vector pointing from shape A to shape B
	normal: b3Vec3 = b3Vec3();

	/// The speed the shapes are approaching. Always positive. Typically in meters per second.
	approachSpeed: f32 = 0;

	/// User material on shape A
	userMaterialIdA: u64 = 0;

	/// User material on shape B
	userMaterialIdB: u64 = 0;
}

/// Contact events are buffered in the world and are available
///	as event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
export class b3ContactEvents {
	/// Array of begin touch events
	beginEvents: Array<b3ContactBeginTouchEvent> = new Array<b3ContactBeginTouchEvent>(0);

	/// Array of end touch events
	endEvents: Array<b3ContactEndTouchEvent> = new Array<b3ContactEndTouchEvent>(0);

	/// Array of hit events
	hitEvents: Array<b3ContactHitEvent> = new Array<b3ContactHitEvent>(0);

	/// Number of begin touch events
	beginCount: i32 = 0;

	/// Number of end touch events
	endCount: i32 = 0;

	/// Number of hit events
	hitCount: i32 = 0;
}

/// Body move events triggered when a body moves.
/// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
/// This also has a flag to indicate that the body went to sleep so the application can also
/// sleep that actor/entity/object associated with the body.
/// @note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
export struct b3BodyMoveEvent {
	/// The body user data.
	userData: u64 = 0;

	/// The body transform.
	transform: b3WorldTransform = b3Transform();

	/// The body id.
	bodyId: b3BodyId = b3BodyId();

	/// Did the body fall asleep this time step?
	fellAsleep: bool = false;
}

/// Body events are buffered in the world and are available
///	as event arrays after the time step is complete.
///	Note: this data becomes invalid if bodies are destroyed
export class b3BodyEvents {
	/// Array of move events
	moveEvents: Array<b3BodyMoveEvent> = new Array<b3BodyMoveEvent>(0);

	/// Number of move events
	moveCount: i32 = 0;
}

/// Joint events report joints that are awake and have a force and/or torque exceeding the threshold
/// The observed forces and torques are not returned for efficiency reasons.
export struct b3JointEvent {
	/// The joint id
	jointId: b3JointId = b3JointId();

	/// The user data from the joint for convenience
	userData: u64 = 0;
}

/// Joint events are buffered in the world and are available
/// as event arrays after the time step is complete.
/// Note: this data becomes invalid if joints are destroyed
export class b3JointEvents {
	/// Array of events
	jointEvents: Array<b3JointEvent> = new Array<b3JointEvent>(0);

	/// Number of events
	count: i32 = 0;
}

/// The contact data for two shapes. By convention the manifold normal points
/// from shape A to shape B.
/// @see b3Shape_GetContactData() and b3Body_GetContactData()
export class b3ContactData {
	/// The contact id. You may hold onto this to track a contact across time steps.
	/// This id may become orphaned. Use b3Contact_IsValid before using it for other functions.
	contactId: b3ContactId = b3ContactId();

	/// The first shape id.
	shapeIdA: b3ShapeId = b3ShapeId();

	/// The second shape id.
	shapeIdB: b3ShapeId = b3ShapeId();

	/// The contact manifold. This references internal data and may become invalid. Do not store
	/// this reference.
	manifolds: StaticArray<b3Manifold>? = null;

	/// The number of contact manifolds. For mesh and height-field collision there can be multiple manifolds.
	manifoldCount: i32 = 0;
}

/// The query filter is used to filter collisions between queries and shapes. For example,
/// you may want a ray-cast representing a projectile to hit players and the static environment
/// but not debris.
/// The C name field only tags recordings, which the port skips, so it is left out to keep this a struct.
export struct b3QueryFilter {
	/// The collision category bits of this query. Normally you would just set one bit.
	categoryBits: u64 = 0;

	/// The collision mask bits. This states the shape categories that this
	/// query would accept for collision.
	maskBits: u64 = 0;

	/// Optional id combined with name to identify this query in a recording, e.g. an entity id.
	id: u64 = 0;

	constructor(categoryBits: u64, maskBits: u64, id: u64) {
		this.categoryBits = categoryBits;
		this.maskBits = maskBits;
		this.id = id;
	}
}

/// Result from b3World_RayCastClosest.
export struct b3RayResult {
	/// The shape hit.
	shapeId: b3ShapeId = b3ShapeId();

	/// The world point of the hit.
	point: b3Pos = b3Vec3();

	/// The world normal of the shape surface at the hit point.
	normal: b3Vec3 = b3Vec3();

	/// The user material id at the hit point. This can be per triangle
	/// if the shape is a mesh, height-field, or compound with child mesh.
	userMaterialId: u64 = 0;

	/// The fraction of the input ray.
	fraction: f32 = 0;

	/// The triangle index if the shape is a mesh, height-field, or compound with
	/// child mesh.
	triangleIndex: i32 = 0;

	/// The child index if the shape is a compound.
	childIndex: i32 = 0;

	/// The number of BVH nodes visited. Diagnostic.
	nodeVisits: i32 = 0;

	/// The number of BVH leaves visited. Diagnostic.
	leafVisits: i32 = 0;

	/// Did the ray hit? If false, all other data is invalid.
	hit: bool = false;
}

/// Body cast result for ray and shape casts.
export struct b3BodyCastResult {
	/// The shape hit.
	shapeId: b3ShapeId = b3ShapeId();

	/// The world point on the shape surface.
	point: b3Pos = b3Vec3();

	/// The world normal vector on the shape surface.
	normal: b3Vec3 = b3Vec3();

	/// The fraction along the ray hit.
	/// hit point = origin + fraction * translation
	fraction: f32 = 0;

	/// The triangle index if the shape is a mesh or height-field.
	triangleIndex: i32 = 0;

	/// The user material id at the hit point. This can be per triangle
	/// if the shape is a mesh, height-field, or compound with child mesh.
	userMaterialId: u64 = 0;

	/// The number of iterations used. Diagnostic.
	iterations: i32 = 0;

	/// Did the cast hit? If false, all other fields are invalid.
	hit: bool = false;
}

/// Body plane result for movers.
export struct b3BodyPlaneResult {
	/// The shape id on the body.
	shapeId: b3ShapeId = b3ShapeId();

	/// The plane result.
	result: b3PlaneResult = b3PlaneResult();
}

/// Body time of impact result for movers.
export struct b3BodyTOIResult {
	/// The hit point in world space.
	point: b3Pos = b3Vec3();

	/// The hit normal. Points from the body to the mover.
	normal: b3Vec3 = b3Vec3();

	/// The sweep time of the collision.
	fraction: f32 = 0;

	/// The hit shape.
	shapeId: b3ShapeId = b3ShapeId();
}

/// Used to collect collision planes for character movers.
/// Return true to continue gathering planes.
export type b3PlaneResultFcn = (shapeId: b3ShapeId, planes: StaticArray<b3PlaneResult>, planeCount: i32) => bool;

/// Used to filter shapes for shape casting character movers.
/// Return true to accept the collision
export type b3MoverFilterFcn = (shapeId: b3ShapeId) => bool;

// types.c

/// Use this to initialize your world definition
export function b3DefaultWorldDef(): b3WorldDef {
	let lengthUnits = b3GetLengthUnitsPerMeter();

	let def = new b3WorldDef();
	def.gravity.x = 0.0;
	def.gravity.y = -10.0;
	def.hitEventThreshold = 1.0 * lengthUnits;
	def.restitutionThreshold = 1.0 * lengthUnits;
	def.restitutionIterations = 2;
	def.enableRestitutionPropagation = false;
	def.contactSpeed = 3.0 * lengthUnits;
	def.contactHertz = 30.0;
	def.contactDampingRatio = 10.0;

	// 400 meters per second, faster than the speed of sound
	def.maximumLinearSpeed = 400.0 * lengthUnits;

	def.isotropicInertiaTolerance = 0.2;

	def.enableSleep = true;
	def.enableContinuous = true;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your body definition
export function b3DefaultBodyDef(): b3BodyDef {
	let def = new b3BodyDef();
	def.type = b3_staticBody;
	def.rotation = b3Quat_identity;
	def.sleepThreshold = 0.05 * b3GetLengthUnitsPerMeter();
	def.safetyFactor = 0.5;
	def.gravityScale = 1.0;
	def.enableSleep = true;
	def.isAwake = true;
	def.isEnabled = true;
	def.enableContactRecycling = true;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your filter
export function b3DefaultFilter(): b3Filter {
	let filter = b3Filter(B3_DEFAULT_CATEGORY_BITS, B3_DEFAULT_MASK_BITS, 0);
	return filter;
}

/// Use this to initialize your query filter
export function b3DefaultQueryFilter(): b3QueryFilter {
	let filter = b3QueryFilter(B3_DEFAULT_CATEGORY_BITS, B3_DEFAULT_MASK_BITS, 0);
	return filter;
}

/// Use this to initialize your surface material
export function b3DefaultSurfaceMaterial(): b3SurfaceMaterial {
	let surfaceMaterial = b3SurfaceMaterial();
	surfaceMaterial.friction = 0.6;
	return surfaceMaterial;
}

/// Use this to initialize your shape definition
export function b3DefaultShapeDef(): b3ShapeDef {
	let lengthUnits = b3GetLengthUnitsPerMeter();

	let def = new b3ShapeDef();
	def.baseMaterial = b3DefaultSurfaceMaterial();
	// density of water
	def.density = 1000.0 / (lengthUnits * lengthUnits * lengthUnits);
	def.explosionScale = 1.0;
	def.filter = b3DefaultFilter();
	def.updateBodyMass = true;
	def.invokeContactCreation = true;
	def.enableSpeculativeContact = true;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

// The joint and explosion defaults below are defined in src/joint.c. They live here with
// the other defaults so lib/joint.as re-exports them instead of defining them.

function b3DefaultJointDef(): b3JointDef {
	let def = b3JointDef();
	def.localFrameA.q = b3Quat_identity;
	def.localFrameB.q = b3Quat_identity;
	def.forceThreshold = FLT_MAX;
	def.torqueThreshold = FLT_MAX;
	def.constraintHertz = 60.0;
	def.constraintDampingRatio = 2.0;
	def.drawScale = b3GetLengthUnitsPerMeter();
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

/// Use this to initialize your joint definition
export function b3DefaultParallelJointDef(): b3ParallelJointDef {
	let def = b3ParallelJointDef();
	def.base = b3DefaultJointDef();
	def.hertz = 1.0;
	def.dampingRatio = 1.0;
	def.maxTorque = FLT_MAX;
	return def;
}

/// Use this to initialize your joint definition
export function b3DefaultDistanceJointDef(): b3DistanceJointDef {
	let def = b3DistanceJointDef();
	def.base = b3DefaultJointDef();
	def.lowerSpringForce = -FLT_MAX;
	def.upperSpringForce = FLT_MAX;
	def.length = 1.0;
	def.maxLength = B3_HUGE();
	return def;
}

/// Use this to initialize your joint definition
export function b3DefaultMotorJointDef(): b3MotorJointDef {
	let def = b3MotorJointDef();
	def.base = b3DefaultJointDef();
	return def;
}

/// Use this to initialize your joint definition
export function b3DefaultFilterJointDef(): b3FilterJointDef {
	let def = b3FilterJointDef();
	def.base = b3DefaultJointDef();
	return def;
}

/// Use this to initialize your joint definition
export function b3DefaultPrismaticJointDef(): b3PrismaticJointDef {
	let def = b3PrismaticJointDef();
	def.base = b3DefaultJointDef();
	return def;
}

/// Use this to initialize your joint definition.
export function b3DefaultRevoluteJointDef(): b3RevoluteJointDef {
	let def = b3RevoluteJointDef();
	def.base = b3DefaultJointDef();
	return def;
}

/// Use this to initialize your joint definition.
export function b3DefaultSphericalJointDef(): b3SphericalJointDef {
	let def = b3SphericalJointDef();
	def.base = b3DefaultJointDef();
	def.targetRotation = b3Quat_identity;
	return def;
}

/// Use this to initialize your joint definition
export function b3DefaultWeldJointDef(): b3WeldJointDef {
	let def = b3WeldJointDef();
	def.base = b3DefaultJointDef();
	return def;
}

/// Use this to initialize your joint definition
export function b3DefaultWheelJointDef(): b3WheelJointDef {
	let def = b3WheelJointDef();
	def.base = b3DefaultJointDef();
	def.enableSuspensionSpring = true;
	def.suspensionHertz = 1.0;
	def.suspensionDampingRatio = 0.7;
	def.steeringHertz = 1.0;
	def.steeringDampingRatio = 0.7;
	return def;
}

/// Use this to initialize your explosion definition
export function b3DefaultExplosionDef(): b3ExplosionDef {
	let def = b3ExplosionDef();
	def.maskBits = B3_DEFAULT_MASK_BITS;
	return def;
}
