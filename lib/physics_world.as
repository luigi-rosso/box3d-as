// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/physics_world.c

import {
	B3_ASSERT,
	B3_NULL_INDEX,
	B3_SECRET_COOKIE,
	FLT_EPSILON,
	FLT_MAX,
	b3AtomicInt,
	b3GetMilliseconds,
	b3GetTicks,
	b3Log,
	b3OnMain,
} from "./core";
import {
	B3_CONTACT_MANIFOLD_COUNT_BUCKETS,
	B3_CONTACT_RECYCLE_ANGULAR_DISTANCE,
	B3_CONTACT_RECYCLE_DISTANCE,
	B3_GRAPH_COLOR_COUNT,
	B3_LINEAR_SLOP,
	B3_MAX_RESTITUTION_ITERATIONS,
	B3_MAX_WORKERS,
	B3_MAX_WORLDS,
	B3_MESH_REST_OFFSET,
	B3_SPECULATIVE_DISTANCE,
} from "./constants";
import { b3BitSet, b3CreateBitSet, b3InPlaceUnion, b3SetBit, b3SetBitCountAndClear } from "./bitset";
import { b3AllocId, b3CreateIdPool, b3GetIdCapacity, b3GetIdCount, b3IdPool } from "./id_pool";
import { b3Array_RemoveSwap } from "./container";
import { b3CTZ64 } from "./ctz";
import { b3GetSIMDWidth, b3OverlapV } from "./simd";
import {
	b3AABB,
	b3AABB_Union,
	b3Abs,
	b3Add,
	b3ClampFloat,
	b3ClampInt,
	b3Conjugate,
	b3Cross,
	b3DistanceSquared,
	b3Dot,
	b3DotQuat,
	b3InvMulQuat,
	b3InvRotateVector,
	b3InvTransformWorldPoint,
	b3IsValidAABB,
	b3IsValidFloat,
	b3IsValidPosition,
	b3IsValidVec3,
	b3LengthSquared,
	b3MakeAABB,
	b3MakeMatrixFromQuat,
	b3Max,
	b3MaxFloat,
	b3MaxInt,
	b3Min,
	b3MinFloat,
	b3MinInt,
	b3MulAdd,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3Normalize,
	b3OffsetAABB,
	b3OffsetPos,
	b3Pos,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3Transform,
	b3ToRelativeTransform,
	b3ToVec3,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3ModifiedCross } from "./math_internal";
import {
	b3BoxCastInput,
	b3Capsule,
	b3DistanceInput,
	b3HullData,
	b3Manifold,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeProxy,
	b3SimplexCache,
	b3TreeStats,
} from "./collision";
import {
	b3BodyEvents,
	b3BodyId,
	b3BodyMoveEvent,
	b3Capacity,
	b3CastResultFcn,
	b3ContactBeginTouchEvent,
	b3ContactEndTouchEvent,
	b3ContactEvents,
	b3ContactHitEvent,
	b3ContactId,
	b3Counters,
	b3CustomFilterFcn,
	b3EnqueueTaskCallback,
	b3ExplosionDef,
	b3FinishTaskCallback,
	b3FrictionCallback,
	b3JointEvent,
	b3JointEvents,
	b3JointId,
	b3MoverFilterFcn,
	b3OverlapResultFcn,
	b3PlaneResultFcn,
	b3PreSolveFcn,
	b3Profile,
	b3QueryFilter,
	b3RayResult,
	b3RestitutionCallback,
	b3SensorBeginTouchEvent,
	b3SensorEndTouchEvent,
	b3SensorEvents,
	b3ShapeId,
	b3TaskCallback,
	b3WorldDef,
	b3WorldId,
	b3_bodyTypeCount,
	b3_dynamicBody,
	b3_kinematicBody,
	b3_staticBody,
	b3_compoundShape,
} from "./types";
import {
	b3DynamicTree_BoxCast,
	b3DynamicTree_GetHeight,
	b3DynamicTree_GetProxyCount,
	b3DynamicTree_GetRootBounds,
	b3DynamicTree_Query,
	b3DynamicTree_RayCast,
	b3DynamicTree_Rebuild,
} from "./dynamic_tree";
import { b3SetProxy, b3ShapeDistance } from "./distance";
import {
	b3CloneHull,
	b3DestroyHull,
	b3HullMap,
	b3HullMap_erase_itr,
	b3HullMap_get,
	b3HullMap_insert,
	b3HullMap_is_end,
	b3HullMap_key,
	b3HullMap_setVal,
	b3HullMap_size,
	b3HullMap_val,
} from "./hull";
import { b3CreateNameCache, b3NameCache } from "./name_cache";
import { b3Body, b3BodySim, b3GetBodyTransformQuick, b3IsStaticSimIndex, b3WakeBody, b3_isFast } from "./body";
import { b3BroadPhase, b3CreateBroadPhase, b3UpdateBroadPhasePairs } from "./broad_phase";
import { b3AddContactToGraph, b3ConstraintGraph, b3CreateGraph, b3RemoveContactFromGraph } from "./constraint_graph";
import {
	b3Contact,
	b3DestroyContact,
	b3InitializeContactRegisters,
	b3UpdateContact,
	b3_contactEnableContactEvents,
	b3_contactRecycleFlag,
	b3_contactTouchingFlag,
	b3_emptyManifolds,
	b3_relativeTransformValid,
	b3_simDisjoint,
	b3_simMeshContact,
	b3_simStartedTouching,
	b3_simStoppedTouching,
	b3_simTouchingFlag,
} from "./contact";
import { b3Island, b3LinkContact, b3UnlinkContact } from "./island";
import { b3Joint } from "./joint";
import { b3OverlapSensors, b3Sensor, b3SensorHit, b3SensorTaskContext } from "./sensor";
import {
	b3CollideMover,
	b3DestroyShapeAllocations,
	b3GetShapeCentroid,
	b3GetShapeMaterials,
	b3GetShapeProjectedArea,
	b3MakeShapeProxy,
	b3OverlapShape,
	b3RayCastShape,
	b3Shape,
	b3ShapeCastShape,
	b3ShouldQueryCollide,
} from "./shape";
import { b3MakeSoft, b3Solve, b3StepContext } from "./solver";
import { b3SolverSet, b3WakeSolverSet } from "./solver_set";
import { b3ParallelFor } from "./parallel_for";

export type b3SetType = i32;
export const b3_staticSet: b3SetType = 0;
export const b3_disabledSet: b3SetType = 1;
export const b3_awakeSet: b3SetType = 2;
export const b3_firstSleepingSet: b3SetType = 3;

// Per thread task storage
export class b3TaskContext {
	// Collect per thread sensor continuous hit events.
	sensorHits: Array<b3SensorHit> = new Array<b3SensorHit>();

	// Broad-phase pairs.
	pairKeys: Array<u64> = new Array<u64>();

	// These bits align with the b3ConstraintGraph::contactBlocks and signal a change in contact status
	contactStateBitSet: b3BitSet = new b3BitSet();

	// These bits align with the joint id capacity and signal a change in contact status
	jointStateBitSet: b3BitSet = new b3BitSet();

	// These bits align with the contact id capacity and signal a hit event.
	hitEventBitSet: b3BitSet = new b3BitSet();

	// Fast-path flag: true when this worker set at least one bit in hitEventBitSet this step.
	hasHitEvents: bool = false;

	// Used to put islands to sleep
	awakeIslandBitSet: b3BitSet = new b3BitSet();

	// Per worker split island candidate
	splitSleepTime: f32 = 0;
	splitIslandId: i32 = 0;

	// Profiling
	satCallCount: i32 = 0;
	satCacheHitCount: i32 = 0;
	distanceIterations: i32 = 0;
	pushBackIterations: i32 = 0;
	rootIterations: i32 = 0;

	// Number of contacts recycled this step (collide pass).
	recycledContactCount: i32 = 0;

	// B3_CONTACT_MANIFOLD_COUNT_BUCKETS; a FixedArray length must be a literal
	manifoldCounts: FixedArray<i32, 8>;

	// Awake contact slots a parallel part leaves to the main lane: mesh, height field and compound contacts
	deferredContacts: StaticArray<i32> = new StaticArray<i32>(0);
	deferredCount: i32 = 0;
}

// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
export class b3World {
	broadPhase: b3BroadPhase = new b3BroadPhase();
	constraintGraph: b3ConstraintGraph = new b3ConstraintGraph();

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b3Body.
	bodyIdPool: b3IdPool = new b3IdPool();

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	bodies: Array<b3Body> = new Array<b3Body>();

	// Provides free list for solver sets.
	solverSetIdPool: b3IdPool = new b3IdPool();

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	solverSets: Array<b3SolverSet> = new Array<b3SolverSet>();

	// Used to create stable ids for joints
	jointIdPool: b3IdPool = new b3IdPool();

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	joints: Array<b3Joint> = new Array<b3Joint>();

	// Used to create stable ids for contacts
	contactIdPool: b3IdPool = new b3IdPool();

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	contacts: Array<b3Contact> = new Array<b3Contact>();

	// Used to create stable ids for islands
	islandIdPool: b3IdPool = new b3IdPool();

	// This is a sparse array that maps island ids to the island data stored in the solver sets.
	islands: Array<b3Island> = new Array<b3Island>();

	shapeIdPool: b3IdPool = new b3IdPool();

	// These are sparse arrays that point into the pools above
	shapes: Array<b3Shape> = new Array<b3Shape>();

	// This follows the shape array and is split out for the hot path in the narrow phase.
	fatAABBs: Array<b3AABB> = new Array<b3AABB>();

	// Compound shapes are static only, so only the broad-phase static pass can meet one. This
	// lets a world without compounds skip the shape type read in the pair batch.
	compoundShapeCount: i32 = 0;

	// Reference counted store of shared hull data keyed by content. Shapes hold a
	// reference to the hull stored in the db.
	hullDatabase: b3HullMap = new b3HullMap();

	// Name cache for shape and body names.
	names: b3NameCache = b3CreateNameCache();

	// This is a dense array of sensor data.
	sensors: Array<b3Sensor> = new Array<b3Sensor>();

	// Per thread storage
	taskContexts: Array<b3TaskContext> = new Array<b3TaskContext>();

	// C's per step stack buffer of awake contact indices for the collide task, kept across steps
	collideContactIndices: StaticArray<i32> = new StaticArray<i32>(0);
	sensorTaskContexts: Array<b3SensorTaskContext> = new Array<b3SensorTaskContext>();

	bodyMoveEvents: Array<b3BodyMoveEvent> = new Array<b3BodyMoveEvent>();
	sensorBeginEvents: Array<b3SensorBeginTouchEvent> = new Array<b3SensorBeginTouchEvent>();
	contactBeginEvents: Array<b3ContactBeginTouchEvent> = new Array<b3ContactBeginTouchEvent>();

	// End events are double buffered so that the user doesn't need to flush events
	sensorEndEvents: StaticArray<Array<b3SensorEndTouchEvent>> = new StaticArray<Array<b3SensorEndTouchEvent>>(2);
	contactEndEvents: StaticArray<Array<b3ContactEndTouchEvent>> = new StaticArray<Array<b3ContactEndTouchEvent>>(2);
	endEventArrayIndex: i32 = 0;

	contactHitEvents: Array<b3ContactHitEvent> = new Array<b3ContactHitEvent>();
	jointEvents: Array<b3JointEvent> = new Array<b3JointEvent>();

	// Id that is incremented every time step
	stepIndex: u64 = 0;

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
	splitIslandId: i32 = 0;

	gravity: b3Vec3 = b3Vec3();
	hitEventThreshold: f32 = 0;
	restitutionThreshold: f32 = 0;
	restitutionIterations: i32 = 0;
	enableRestitutionPropagation: bool = false;
	maxLinearSpeed: f32 = 0;
	isotropicInertiaTolerance: f32 = 0;
	contactSpeed: f32 = 0;
	contactHertz: f32 = 0;
	contactDampingRatio: f32 = 0;
	contactRecycleDistance: f32 = 0;

	// Never null after b3CreateWorld, which installs the defaults as C does.
	frictionCallback: b3FrictionCallback? = null;
	restitutionCallback: b3RestitutionCallback? = null;

	generation: u16 = 0;

	profile: b3Profile = b3Profile();
	satCallCount: i32 = 0;
	satCacheHitCount: i32 = 0;
	// B3_CONTACT_MANIFOLD_COUNT_BUCKETS; a FixedArray length must be a literal
	manifoldCounts: FixedArray<i32, 8>;

	maxCapacity: b3Capacity = b3Capacity();

	preSolveFcn: b3PreSolveFcn? = null;

	customFilterFcn: b3CustomFilterFcn? = null;

	workerCount: i32 = 0;
	simdWidth: i32 = 0;
	enqueueTaskFcn: b3EnqueueTaskCallback = b3DefaultAddTaskFcn;
	finishTaskFcn: b3FinishTaskCallback = b3DefaultFinishTaskFcn;
	userTaskContext: u64 = 0;
	userTreeTask: u64 = 0;

	userData: u64 = 0;

	// latest inverse sub-step
	inv_h: f32 = 0;

	// latest inverse full-step
	inv_dt: f32 = 0;

	activeTaskCount: i32 = 0;
	taskCount: i32 = 0;

	worldId: u16 = 0;

	enableSleep: bool = false;

	// This indicates there is a world write operation in progress. This is for debugging and
	// not a real mutex. This should have minimal performance impact.
	locked: bool = false;
	enableWarmStarting: bool = false;
	enableContinuous: bool = false;
	enableSpeculative: bool = false;
	inUse: bool = false;

	constructor() {
		for (let i = 0; i < 2; ++i) {
			this.sensorEndEvents[i] = new Array<b3SensorEndTouchEvent>();
			this.contactEndEvents[i] = new Array<b3ContactEndTouchEvent>();
		}
	}
}

// The block allocator is not ported: C zeroes the block, so a fresh array is exact.
export function b3AllocateManifolds(world: b3World, count: i32): StaticArray<b3Manifold> {
	if (count == 0) {
		return b3_emptyManifolds;
	}

	return new StaticArray<b3Manifold>(count);
}

export function b3FreeManifolds(world: b3World, manifolds: StaticArray<b3Manifold>, count: i32): void {}

// functions from physics_world.c follow

const b3_worlds = new StaticArray<b3World>(B3_MAX_WORLDS);
for (let i = 0; i < B3_MAX_WORLDS; ++i) {
	b3_worlds[i] = new b3World();
}
let b3_worldCount = b3AtomicInt();
let b3_maxWorldCount: i32 = 0;

const b3_emptyContactIndices = new StaticArray<i32>(0);

export function b3AddHullToDatabase(world: b3World, src: b3HullData): b3HullData {
	let database = world.hullDatabase;

	// Compare by content to de-duplicate. Not trusting the hash.
	let itr = b3HullMap_get(database, src);
	if (b3HullMap_is_end(itr) == false) {
		// Bump reference count.
		b3HullMap_setVal(database, itr, b3HullMap_val(database, itr) + 1);
		return b3HullMap_key(database, itr);
	}

	let clone = b3CloneHull(src);
	B3_ASSERT(clone != null);

	// Start with reference count of 1.
	b3HullMap_insert(database, clone!, 1);
	return clone!;
}

export function b3AddOwnedHullToDatabase(world: b3World, owned: b3HullData): b3HullData {
	let database = world.hullDatabase;

	let itr = b3HullMap_get(database, owned);
	if (b3HullMap_is_end(itr) == false) {
		b3HullMap_setVal(database, itr, b3HullMap_val(database, itr) + 1);
		b3DestroyHull(owned);
		return b3HullMap_key(database, itr);
	}

	// Take ownership of input hull.
	b3HullMap_insert(database, owned, 1);
	return owned;
}

export function b3RemoveHullFromDatabase(world: b3World, data: b3HullData): void {
	let database = world.hullDatabase;

	let itr = b3HullMap_get(database, data);
	B3_ASSERT(b3HullMap_is_end(itr) == false);

	let referenceCount = b3HullMap_val(database, itr) - 1;
	b3HullMap_setVal(database, itr, referenceCount);
	if (referenceCount == 0) {
		// Erase through the iterator we already have so the lookup runs once.
		let owned = b3HullMap_key(database, itr);
		b3HullMap_erase_itr(database, itr);
		b3DestroyHull(owned);
	}
}

export function b3GetUnlockedWorldFromId(id: b3WorldId): b3World? {
	B3_ASSERT(1 <= id.index1 && <i32>id.index1 <= B3_MAX_WORLDS);
	let world = b3_worlds[id.index1 - 1];
	B3_ASSERT(id.index1 == world.worldId + 1);
	B3_ASSERT(id.generation == world.generation);

	// A world accessed from an id should not be locked
	if (world.locked) {
		B3_ASSERT(false);
		return null;
	}
	return world;
}

export function b3GetWorldFromId(id: b3WorldId): b3World {
	B3_ASSERT(1 <= id.index1 && <i32>id.index1 <= B3_MAX_WORLDS);
	let world = b3_worlds[id.index1 - 1];
	B3_ASSERT(id.index1 == world.worldId + 1);
	B3_ASSERT(id.generation == world.generation);
	return world;
}

export function b3GetWorld(index: i32): b3World {
	B3_ASSERT(0 <= index && index < B3_MAX_WORLDS);
	let world = b3_worlds[index];
	B3_ASSERT(<i32>world.worldId == index);
	return world;
}

export function b3GetUnlockedWorld(index: i32): b3World? {
	B3_ASSERT(0 <= index && index < B3_MAX_WORLDS);
	let world = b3_worlds[index];
	B3_ASSERT(<i32>world.worldId == index);
	if (world.locked) {
		B3_ASSERT(false);
		return null;
	}

	return world;
}

// The task closure carries C's taskContext.
function b3DefaultAddTaskFcn(task: b3TaskCallback, taskContext: u64, userContext: u64, name: string): u64 {
	task(taskContext);
	return 0;
}

function b3DefaultFinishTaskFcn(userTask: u64, userContext: u64): void {}

export function b3DefaultFrictionCallback(frictionA: f32, materialA: u64, frictionB: f32, materialB: u64): f32 {
	return Mathf.sqrt(frictionA * frictionB);
}

export function b3DefaultRestitutionCallback(restitutionA: f32, materialA: u64, restitutionB: f32, materialB: u64): f32 {
	return b3MaxFloat(restitutionA, restitutionB);
}

function b3CreateWorkerContexts(world: b3World): void {
	world.taskContexts.length = 0;
	world.sensorTaskContexts.length = 0;

	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = new b3TaskContext();
		taskContext.contactStateBitSet = b3CreateBitSet(1024);
		taskContext.hitEventBitSet = b3CreateBitSet(1024);
		taskContext.hasHitEvents = false;
		taskContext.jointStateBitSet = b3CreateBitSet(1024);
		taskContext.awakeIslandBitSet = b3CreateBitSet(256);
		taskContext.splitIslandId = B3_NULL_INDEX;
		world.taskContexts.push(taskContext);

		let sensorTaskContext = new b3SensorTaskContext();
		sensorTaskContext.eventBits = b3CreateBitSet(128);
		world.sensorTaskContexts.push(sensorTaskContext);
	}
}

function b3DestroyWorkerContexts(world: b3World): void {
	world.taskContexts.length = 0;
	world.sensorTaskContexts.length = 0;
}

export function b3CreateWorld(def: b3WorldDef): b3WorldId {
	B3_ASSERT(def.internalValue == B3_SECRET_COOKIE);

	B3_ASSERT(B3_LINEAR_SLOP() <= B3_MESH_REST_OFFSET());
	B3_ASSERT(B3_MESH_REST_OFFSET() < B3_SPECULATIVE_DISTANCE());

	let worldId = B3_NULL_INDEX;
	for (let i = 0; i < B3_MAX_WORLDS; ++i) {
		if (b3_worlds[i].inUse == false) {
			worldId = i;
			break;
		}
	}

	if (worldId == B3_NULL_INDEX) {
		b3Log("B3_MAX_WORLDS of " + B3_MAX_WORLDS.toString() + " exceeded!!!");
		B3_ASSERT(worldId != B3_NULL_INDEX);
		return b3WorldId();
	}

	// b3AtomicFetchAddInt
	let oldCount = b3_worldCount.value;
	b3_worldCount.value += 1;
	b3_maxWorldCount = b3MaxInt(b3_maxWorldCount, oldCount + 1);

	b3InitializeContactRegisters();

	// The fresh object is C's memset; it already holds the empty containers C creates or reserves below.
	let revision = b3_worlds[worldId].generation;
	let world = new b3World();
	b3_worlds[worldId] = world;

	world.simdWidth = b3GetSIMDWidth();

	world.worldId = <u16>worldId;
	world.generation = revision;
	world.inUse = true;

	b3CreateBroadPhase(world.broadPhase, def.capacity);
	b3CreateGraph(world.constraintGraph, 16);

	// pools
	world.bodyIdPool = b3CreateIdPool();

	// add empty static, active, and disabled body sets
	world.solverSetIdPool = b3CreateIdPool();

	// static set
	let set = new b3SolverSet();
	set.setIndex = b3AllocId(world.solverSetIdPool);
	world.solverSets.push(set);
	B3_ASSERT(world.solverSets[b3_staticSet].setIndex == b3_staticSet);

	// disabled set
	set = new b3SolverSet();
	set.setIndex = b3AllocId(world.solverSetIdPool);
	world.solverSets.push(set);
	B3_ASSERT(world.solverSets[b3_disabledSet].setIndex == b3_disabledSet);

	// awake set
	set = new b3SolverSet();
	set.setIndex = b3AllocId(world.solverSetIdPool);
	world.solverSets.push(set);
	B3_ASSERT(world.solverSets[b3_awakeSet].setIndex == b3_awakeSet);

	world.shapeIdPool = b3CreateIdPool();

	world.contactIdPool = b3CreateIdPool();

	world.jointIdPool = b3CreateIdPool();

	world.islandIdPool = b3CreateIdPool();

	world.endEventArrayIndex = 0;

	world.stepIndex = 0;
	world.splitIslandId = B3_NULL_INDEX;
	world.activeTaskCount = 0;
	world.taskCount = 0;
	world.gravity = def.gravity;
	world.hitEventThreshold = def.hitEventThreshold;
	world.restitutionThreshold = def.restitutionThreshold;
	world.restitutionIterations = b3ClampInt(def.restitutionIterations, 0, B3_MAX_RESTITUTION_ITERATIONS);
	world.enableRestitutionPropagation = def.enableRestitutionPropagation;
	world.maxLinearSpeed = def.maximumLinearSpeed;
	world.isotropicInertiaTolerance = b3ClampFloat(def.isotropicInertiaTolerance, 0.0, 1.0);
	world.contactSpeed = def.contactSpeed;
	world.contactHertz = def.contactHertz;
	world.contactDampingRatio = def.contactDampingRatio;
	world.contactRecycleDistance = B3_CONTACT_RECYCLE_DISTANCE();

	if (def.frictionCallback == null) {
		world.frictionCallback = b3DefaultFrictionCallback;
	} else {
		world.frictionCallback = def.frictionCallback;
	}

	if (def.restitutionCallback == null) {
		world.restitutionCallback = b3DefaultRestitutionCallback;
	} else {
		world.restitutionCallback = def.restitutionCallback;
	}

	world.enableSleep = def.enableSleep;
	world.locked = false;
	world.enableWarmStarting = true;
	world.enableContinuous = def.enableContinuous;
	world.enableSpeculative = true;
	world.userTreeTask = 0;
	world.userData = def.userData;

	let workerCount = <i32>def.workerCount;
	let enqueueTask = def.enqueueTask;
	let finishTask = def.finishTask;
	if (workerCount > 0 && enqueueTask != null && finishTask != null) {
		// External task system
		world.workerCount = b3MinInt(workerCount, B3_MAX_WORKERS);
		world.enqueueTaskFcn = enqueueTask;
		world.finishTaskFcn = finishTask;
		world.userTaskContext = def.userTaskContext;
	} else {
		// Serial fallback, which also stands in for the built-in scheduler C starts when workerCount > 1
		world.workerCount = 1;
		world.enqueueTaskFcn = b3DefaultAddTaskFcn;
		world.finishTaskFcn = b3DefaultFinishTaskFcn;
		world.userTaskContext = 0;
	}

	b3CreateWorkerContexts(world);

	// add one to worldId so that 0 represents a null b3WorldId
	let id: b3WorldId;
	id.index1 = <u16>(worldId + 1);
	id.generation = world.generation;
	return id;
}

export function b3DestroyWorld(worldId: b3WorldId): void {
	// b3AtomicFetchAddInt
	b3_worldCount.value -= 1;

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.locked = true;

	// Destroying every shape releases all hull references, so the database is empty.
	let shapeCapacity = world.shapes.length;
	for (let i = 0; i < shapeCapacity; ++i) {
		let shape = world.shapes[i];
		if (shape.id != B3_NULL_INDEX) {
			b3DestroyShapeAllocations(world, shape);
		}
	}

	B3_ASSERT(b3HullMap_size(world.hullDatabase) == 0);

	// Wipe world but preserve generation. The fresh object drops every container C frees one by one.
	let generation = world.generation;
	let wiped = new b3World();
	wiped.generation = generation + 1;
	b3_worlds[worldId.index1 - 1] = wiped;
}

export function b3GetWorldCount(): i32 {
	return b3_worldCount.value;
}

export function b3GetMaxWorldCount(): i32 {
	return b3_maxWorldCount;
}

// C returns a pointer to the sim; here the sim array and the index within it, so the collide
// task reads only the fields it needs.
function b3ResolveContactBodySims(
	world: b3World,
	awakeSims: Array<b3BodySim>,
	staticSims: Array<b3BodySim>,
	encodedIndex: i32,
	bodyId: i32,
): Array<b3BodySim> {
	if (encodedIndex >= 0) {
		return awakeSims;
	}

	if (b3IsStaticSimIndex(encodedIndex)) {
		return staticSims;
	}

	return world.solverSets[world.bodies[bodyId].setIndex].bodySims;
}

@inline function b3ResolveContactBodySimIndex(world: b3World, encodedIndex: i32, bodyId: i32): i32 {
	if (encodedIndex >= 0) {
		return encodedIndex;
	}

	if (b3IsStaticSimIndex(encodedIndex)) {
		return -encodedIndex - 2;
	}

	return world.bodies[bodyId].localIndex;
}

// Runs the collide tasks of a step somewhere else, such as on script lanes, or answers false to leave them here
export type b3LaneCollideFcn = (context: b3StepContext, contactCount: i32) => bool;

let b3_laneCollide: b3LaneCollideFcn? = null;

export function b3SetLaneCollide(fcn: b3LaneCollideFcn?): void {
	b3_laneCollide = fcn;
}

export function b3CollideTask<M>(startIndex: i32, endIndex: i32, workerIndex: i32, stepContext: b3StepContext): void {
	let world = stepContext.world;
	let graph = world.constraintGraph;
	let taskContext = world.taskContexts[workerIndex];
	let contactIndices = stepContext.awakeContactIndices;
	let contacts = world.contacts;
	let shapes = world.shapes;
	let fatAABBs = world.fatAABBs;
	let awakeSims = world.solverSets[b3_awakeSet].bodySims;
	let staticSims = world.solverSets[b3_staticSet].bodySims;

	B3_ASSERT(startIndex < endIndex);

	let recycleDistance = world.contactRecycleDistance;
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();
	let recycleDistanceNonTouching = b3MinFloat(recycleDistance, speculativeDistance);

	for (let i = startIndex; i < endIndex; ++i) {
		let contactIndex = contactIndices[i];
		B3_ASSERT(contactIndex < world.contacts.length);

		let contact = contacts[contactIndex];

		let shapeIdA = contact.shapeIdA;
		let shapeIdB = contact.shapeIdB;

		// Do proxies still overlap?
		let overlap = b3OverlapV(fatAABBs[shapeIdA], fatAABBs[shapeIdB]);
		if (overlap == false) {
			// This contact will be destroyed
			contact.flags |= b3_simDisjoint;
			contact.flags &= ~b3_simTouchingFlag;
			b3SetBit(taskContext.contactStateBitSet, <u32>contactIndex);
			continue;
		}

		let shapeA = shapes[shapeIdA];
		let shapeB = shapes[shapeIdB];

		// Mesh and compound contacts can allocate and share scratch, so the main lane updates them after the parts
		if (isBoolean<M>()) {
			if ((contact.flags & b3_simMeshContact) != 0 || shapeA.type == b3_compoundShape || contact.manifolds.length == 0) {
				taskContext.deferredContacts[taskContext.deferredCount] = i;
				taskContext.deferredCount += 1;
				continue;
			}
		}

		// Update contact respecting shape/body order (A,B)
		let encodedA = contact.encodedBodySimA;
		let encodedB = contact.encodedBodySimB;
		let bodyIdA = contact.edges[0].bodyId;
		let bodyIdB = contact.edges[1].bodyId;
		let simsA = b3ResolveContactBodySims(world, awakeSims, staticSims, encodedA, bodyIdA);
		let simsB = b3ResolveContactBodySims(world, awakeSims, staticSims, encodedB, bodyIdB);
		let simIndexA = b3ResolveContactBodySimIndex(world, encodedA, bodyIdA);
		let simIndexB = b3ResolveContactBodySimIndex(world, encodedB, bodyIdB);

		let wasTouching = (contact.flags & b3_simTouchingFlag) != 0;

		let transformA = simsA[simIndexA].transform;
		let transformB = simsB[simIndexB].transform;

		let isFast = ((simsA[simIndexA].flags | simsB[simIndexB].flags) & b3_isFast) != 0;

		let recycleTolerance = wasTouching ? recycleDistance : recycleDistanceNonTouching;

		// Contact recycling optimization. Please cite this library if you use this optimization.
		// This is inspired by persistent contact manifolds used in some physics engines, such as PhysX.
		// However, this allows larger relative motion and has fewer tuning parameters (just one).
		if (
			isFast == false &&
			recycleDistance > 0.0 &&
			(contact.flags & b3_relativeTransformValid) != 0 &&
			(contact.flags & b3_contactRecycleFlag) != 0
		) {
			// The scalar part of b3InvMulQuat is just the quaternion dot product.
			// cos(relative_angle/2) = scalar(conj(q1) * q2) = dot(q1, q2)
			// A small relative angle means this value is close to 1. Need to use abs or square
			// due to double cover.
			let angleA = b3DotQuat(transformA.q, contact.cachedRotationA);
			let angleB = b3DotQuat(transformB.q, contact.cachedRotationB);
			let angularDistance = b3MinFloat(angleA * angleA, angleB * angleB);

			// b3InvMulWorldTransforms in its two parts, which inline where the whole does not
			let xf: b3Transform;
			xf.q = b3InvMulQuat(transformA.q, transformB.q);
			xf.p = b3InvRotateVector(transformA.q, b3SubPos(transformB.p, transformA.p));
			let xfc = contact.cachedRelativePose;
			let maxExtentA = b3IsStaticSimIndex(encodedA) ? b3Vec3_zero : simsA[simIndexA].maxExtent;
			let maxExtentB = b3IsStaticSimIndex(encodedB) ? b3Vec3_zero : simsB[simIndexB].maxExtent;
			let maxExtent = b3Max(maxExtentA, maxExtentB);

			// Variation of Conservative Advancement
			// distance + 2 * length(modified_cross(|qr.v|, maxExtent)) < recycleTolerance.
			// 2*|qr.v| == 2*|sin(theta/2)| ~= theta for small angles.
			let distSquared = b3DistanceSquared(xf.p, xfc.p);

			if (
				angularDistance > B3_CONTACT_RECYCLE_ANGULAR_DISTANCE &&
				distSquared < recycleTolerance * recycleTolerance
			) {
				let distance = Mathf.sqrt(distSquared);
				let slack = recycleTolerance - distance;

				// qr = inv( inv(qA0) * qB0 ) * inv(qA) * qB
				//    = inv(qB0) * qA0 * inv(qA) * qB
				// Suppose A is static
				// qr = inv(qB0) * qA0 * inv(qA0) * qB
				//    = inv(qB0) * qB
				// qB = qB0 * qr
				// Therefore qr is associated with the local angular velocity of body B when A is static.
				let qr = b3InvMulQuat(xfc.q, xf.q);
				let arc = b3ModifiedCross(b3Abs(qr.v), maxExtent);

				let arcSq: f32 = 4.0 * b3LengthSquared(arc);
				if (arcSq < slack * slack) {
					let dqA = b3MulQuat(transformA.q, b3Conjugate(contact.cachedRotationA));
					let dqB = b3MulQuat(transformB.q, b3Conjugate(contact.cachedRotationB));
					let matrixA = b3MakeMatrixFromQuat(dqA);
					let matrixB = b3MakeMatrixFromQuat(dqB);

					// Minimize round-off
					let dc = b3SubPos(simsB[simIndexB].center, simsA[simIndexA].center);

					let manifoldCount = contact.manifoldCount;
					for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
						ref manifold = contact.manifolds[manifoldIndex];
						let normal = manifold.normal;

						let pointCount = manifold.pointCount;
						for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
							// Keep anchors but update separation, same as sub-stepping. This eliminates jitter.
							ref mp = manifold.points[pointIndex];
							let rA = b3MulMV(matrixA, mp.anchorA);
							let rB = b3MulMV(matrixB, mp.anchorB);
							let dp = b3Add(dc, b3Sub(rB, rA));
							mp.separation = mp.baseSeparation + b3Dot(dp, normal);
							mp.normalVelocity = 0.0;
							mp.persisted = true;
						}
					}

					// Diagnostics
					taskContext.recycledContactCount += 1;
					let bucketIndex = b3MinInt(manifoldCount, B3_CONTACT_MANIFOLD_COUNT_BUCKETS - 1);
					if (bucketIndex > 0) {
						taskContext.manifoldCounts[bucketIndex - 1] += 1;
					}

					// Contact is recycled. This also skips updating other aspects of the contact
					// such as material parameters.
					continue;
				}
			}
		}

		// Caching for contact recycling.
		contact.cachedRotationA = transformA.q;
		contact.cachedRotationB = transformB.q;
		contact.cachedRelativePose.q = b3InvMulQuat(transformA.q, transformB.q);
		contact.cachedRelativePose.p = b3InvRotateVector(transformA.q, b3SubPos(transformB.p, transformA.p));
		contact.flags |= b3_relativeTransformValid;

		// This updates solid contacts
		let touching = b3UpdateContact<M>(
			world,
			workerIndex,
			contact,
			shapeA,
			simsA[simIndexA].localCenter,
			transformA,
			shapeB,
			simsB[simIndexB].localCenter,
			transformB,
			isFast,
		);

		let bucketIndex = b3MinInt(contact.manifoldCount, B3_CONTACT_MANIFOLD_COUNT_BUCKETS - 1);
		if (bucketIndex > 0) {
			taskContext.manifoldCounts[bucketIndex - 1] += 1;
		}

		// Update the mesh contact spec
		if (touching == true && wasTouching == true && (contact.flags & b3_simMeshContact) != 0) {
			B3_ASSERT(contact.colorIndex != B3_NULL_INDEX);
			B3_ASSERT(0 <= contact.colorIndex && contact.colorIndex < B3_GRAPH_COLOR_COUNT);
			let color = graph.colors[contact.colorIndex];
			color.contacts[contact.localIndex].manifoldCount = <u16>contact.manifoldCount;
		}

		// State changes that affect island connectivity. Also affects contact events.
		if (touching == true && wasTouching == false) {
			contact.flags |= b3_simStartedTouching;
			b3SetBit(taskContext.contactStateBitSet, <u32>contactIndex);
		} else if (touching == false && wasTouching == true) {
			contact.flags |= b3_simStoppedTouching;
			b3SetBit(taskContext.contactStateBitSet, <u32>contactIndex);
		}

		if (touching) {
			let manifoldCount = contact.manifoldCount;
			for (let manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex) {
				ref manifold = contact.manifolds[manifoldIndex];
				let pointCount = manifold.pointCount;
				for (let pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
					// Cache separation
					ref mp = manifold.points[pointIndex];
					mp.baseSeparation = mp.separation;
				}
			}
		}
	}
}

function b3AddNonTouchingContact(world: b3World, contact: b3Contact): void {
	B3_ASSERT(contact.setIndex == b3_awakeSet);
	let set = world.solverSets[b3_awakeSet];
	contact.colorIndex = B3_NULL_INDEX;
	contact.localIndex = set.contactIndices.length;
	set.contactIndices.push(contact.contactId);
}

function b3RemoveNonTouchingContact(world: b3World, setIndex: i32, localIndex: i32): void {
	let set = world.solverSets[setIndex];
	let movedIndex = b3Array_RemoveSwap(set.contactIndices, localIndex);
	if (movedIndex != B3_NULL_INDEX) {
		let movedContactIndex = set.contactIndices[localIndex];
		let movedContact = world.contacts[movedContactIndex];
		B3_ASSERT(movedContact.setIndex == setIndex);
		B3_ASSERT(movedContact.colorIndex == B3_NULL_INDEX);
		B3_ASSERT(movedContact.localIndex == movedIndex);
		movedContact.localIndex = localIndex;
	}
}

function b3MakeShapeId(index1: i32, world0: u16, generation: u16): b3ShapeId {
	let id: b3ShapeId;
	id.index1 = index1;
	id.world0 = world0;
	id.generation = generation;
	return id;
}

// Narrow-phase collision
function b3Collide(context: b3StepContext): void {
	let world = context.world;

	B3_ASSERT(world.workerCount > 0);

	// Gather contacts from all the graph colors into a single array for easier parallel-for
	let touchingCount = 0;

	let graphColors = world.constraintGraph.colors;
	for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
		touchingCount += graphColors[i].convexContacts.length + graphColors[i].contacts.length;
	}

	let awakeSet = world.solverSets[b3_awakeSet];
	let nonTouchingCount = awakeSet.contactIndices.length;

	let contactCount = touchingCount + nonTouchingCount;

	if (contactCount == 0) {
		return;
	}

	if (world.collideContactIndices.length < contactCount) {
		world.collideContactIndices = new StaticArray<i32>(b3MaxInt(contactCount, 2 * world.collideContactIndices.length));
	}
	let contactIndices = world.collideContactIndices;

	let contactIndex = 0;
	for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
		let color = graphColors[i];
		let count = color.convexContacts.length;
		for (let j = 0; j < count; ++j) {
			contactIndices[contactIndex] = color.convexContacts[j];
			contactIndex += 1;
		}

		count = color.contacts.length;
		for (let j = 0; j < count; ++j) {
			contactIndices[contactIndex] = color.contacts[j].contactId;
			contactIndex += 1;
		}
	}

	B3_ASSERT(contactIndex == touchingCount);

	if (nonTouchingCount > 0) {
		let nonTouchingIndices = awakeSet.contactIndices;
		for (let i = 0; i < nonTouchingCount; ++i) {
			contactIndices[touchingCount + i] = nonTouchingIndices[i];
		}
	}

	context.awakeContactIndices = contactIndices;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	let contactIdCapacity = b3GetIdCapacity(world.contactIdPool);
	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = world.taskContexts[i];
		b3SetBitCountAndClear(taskContext.contactStateBitSet, <u32>contactIdCapacity);
		taskContext.satCallCount = 0;
		taskContext.satCacheHitCount = 0;
		taskContext.recycledContactCount = 0;
		for (let j = 0; j < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++j) {
			taskContext.manifoldCounts[j] = 0;
		}
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	let minRange = 20;
	let laneCollide = b3_laneCollide;
	if (laneCollide == null || laneCollide(context, contactCount) == false) {
		b3ParallelFor(
			world,
			(startIndex: i32, endIndex: i32, workerIndex: i32): void =>
				b3CollideTask<b3OnMain>(startIndex, endIndex, workerIndex, context),
			contactCount,
			minRange,
			"collide",
		);
	}

	context.awakeContactIndices = b3_emptyContactIndices;

	// Serially update contact state
	// todo bring this zone together with island merge

	let satMultiplier = context.dt > 0.0 ? 1 : 0;

	// Bitwise OR all contact bits
	let bitSet = world.taskContexts[0].contactStateBitSet;
	world.satCallCount = satMultiplier * world.taskContexts[0].satCallCount;
	world.satCacheHitCount = satMultiplier * world.taskContexts[0].satCacheHitCount;
	for (let j = 0; j < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++j) {
		world.manifoldCounts[j] = world.taskContexts[0].manifoldCounts[j];
	}

	for (let i = 1; i < world.workerCount; ++i) {
		b3InPlaceUnion(bitSet, world.taskContexts[i].contactStateBitSet);
		world.satCallCount += world.taskContexts[i].satCallCount;
		world.satCacheHitCount += world.taskContexts[i].satCacheHitCount;
		for (let j = 0; j < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++j) {
			world.manifoldCounts[j] += world.taskContexts[i].manifoldCounts[j];
		}
	}

	let endEventArrayIndex = world.endEventArrayIndex;

	let shapes = world.shapes;
	let worldId = world.worldId;

	// Process contact state changes. Iterate over set bits
	for (let k: u32 = 0; k < bitSet.blockCount; ++k) {
		let bits = bitSet.bits[k];
		while (bits != 0) {
			let ctz = b3CTZ64(bits);
			let contactId = <i32>(64 * k + ctz);

			let contact = world.contacts[contactId];
			B3_ASSERT(contact.setIndex == b3_awakeSet);

			let shapeA = shapes[contact.shapeIdA];
			let shapeB = shapes[contact.shapeIdB];
			let shapeIdA = b3MakeShapeId(shapeA.id + 1, worldId, shapeA.generation);
			let shapeIdB = b3MakeShapeId(shapeB.id + 1, worldId, shapeB.generation);
			let contactFullId: b3ContactId;
			contactFullId.index1 = contactId + 1;
			contactFullId.world0 = worldId;
			contactFullId.padding = 0;
			contactFullId.generation = contact.generation;
			let flags = contact.flags;

			if (flags & b3_simDisjoint) {
				// Bounding boxes no longer overlap
				b3DestroyContact(world, contact, false);
			} else if (flags & b3_simStartedTouching) {
				B3_ASSERT(contact.islandId == B3_NULL_INDEX);

				if (flags & b3_contactEnableContactEvents) {
					let event: b3ContactBeginTouchEvent;
					event.shapeIdA = shapeIdA;
					event.shapeIdB = shapeIdB;
					event.contactId = contactFullId;
					world.contactBeginEvents.push(event);
				}

				B3_ASSERT(contact.manifoldCount > 0);
				B3_ASSERT(contact.setIndex == b3_awakeSet);

				// Link first because this wakes colliding bodies and ensures the body sims
				// are in the correct place.
				contact.flags &= ~b3_simStartedTouching;
				contact.flags |= b3_contactTouchingFlag;
				b3LinkContact(world, contact);

				// Make sure these didn't change
				B3_ASSERT(contact.colorIndex == B3_NULL_INDEX);

				// Contact sim pointer may have become orphaned due to awake set growth,
				// so I just need to refresh it.

				let oldLocalIndex = contact.localIndex;

				b3AddContactToGraph(world, contact);
				b3RemoveNonTouchingContact(world, b3_awakeSet, oldLocalIndex);
			} else if (flags & b3_simStoppedTouching) {
				contact.flags &= ~b3_simStoppedTouching;
				contact.flags &= ~b3_contactTouchingFlag;

				if (contact.flags & b3_contactEnableContactEvents) {
					let event: b3ContactEndTouchEvent;
					event.shapeIdA = shapeIdA;
					event.shapeIdB = shapeIdB;
					event.contactId = contactFullId;
					world.contactEndEvents[endEventArrayIndex].push(event);
				}

				B3_ASSERT(contact.manifoldCount == 0);

				// Cache these here for the remove below
				let colorIndex = contact.colorIndex;
				let localIndex = contact.localIndex;

				b3UnlinkContact(world, contact);
				let bodyIdA = contact.edges[0].bodyId;
				let bodyIdB = contact.edges[1].bodyId;

				b3AddNonTouchingContact(world, contact);

				let isMeshContact = (contact.flags & b3_simMeshContact) != 0;
				b3RemoveContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex, isMeshContact);
			}

			// Clear the smallest set bit
			bits = bits & (bits - 1);
		}
	}
}

export function b3World_Step(worldId: b3WorldId, timeStep: f32, subStepCount: i32): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.locked = true;

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	world.bodyMoveEvents.length = 0;
	world.sensorBeginEvents.length = 0;
	world.contactBeginEvents.length = 0;
	world.contactHitEvents.length = 0;
	world.jointEvents.length = 0;

	world.profile = b3Profile();

	world.activeTaskCount = 0;
	world.taskCount = 0;

	let stepTicks = b3GetTicks();

	{
		ref c = world.maxCapacity;
		c.staticShapeCount = b3MaxInt(c.staticShapeCount, world.broadPhase.trees[b3_staticBody].proxyCount);
		c.dynamicShapeCount = b3MaxInt(c.dynamicShapeCount, world.broadPhase.trees[b3_dynamicBody].proxyCount);

		let staticBodyCount = world.solverSets[b3_staticSet].bodySims.length;
		c.staticBodyCount = b3MaxInt(c.staticBodyCount, staticBodyCount);

		// this includes kinematic bodies
		let totalBodyCount = b3GetIdCount(world.bodyIdPool);
		c.dynamicBodyCount = b3MaxInt(c.dynamicBodyCount, totalBodyCount - staticBodyCount);

		let totalContactCount = b3GetIdCount(world.contactIdPool);
		c.contactCount = b3MaxInt(c.contactCount, totalContactCount);
	}

	// Update collision pairs and create contacts
	{
		let pairTicks = b3GetTicks();
		b3UpdateBroadPhasePairs(world);
		world.profile.pairs = b3GetMilliseconds(pairTicks);
	}

	let awakeSet = world.solverSets[b3_awakeSet];

	let context = new b3StepContext(world);
	context.states = awakeSet.bodyStates;
	context.dt = timeStep;
	context.subStepCount = b3MaxInt(1, subStepCount);

	if (timeStep > 0.0) {
		context.inv_dt = 1.0 / timeStep;
		context.h = timeStep / <f32>context.subStepCount;
		context.inv_h = <f32>context.subStepCount * context.inv_dt;
	} else {
		context.inv_dt = 0.0;
		context.h = 0.0;
		context.inv_h = 0.0;
	}

	world.inv_h = context.inv_h;
	world.inv_dt = context.inv_dt;

	// Hertz values get reduced for large time steps
	let contactHertz = b3MinFloat(world.contactHertz, 0.125 * context.inv_h);
	context.contactSoftness = b3MakeSoft(contactHertz, world.contactDampingRatio, context.h);
	context.staticSoftness = b3MakeSoft(2.0 * contactHertz, 0.5 * world.contactDampingRatio, context.h);

	context.maxLinearVelocity = world.maxLinearSpeed;
	context.enableWarmStarting = world.enableWarmStarting;

	// Narrow phase : update contacts
	{
		let collideTicks = b3GetTicks();
		b3Collide(context);
		world.profile.collide = b3GetMilliseconds(collideTicks);
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (timeStep > 0.0) {
		let solveTicks = b3GetTicks();
		b3Solve(world, context);
		world.profile.solve = b3GetMilliseconds(solveTicks);
	}

	// Finish the tree task in case b3Solve didn't finish it
	if (world.userTreeTask != 0) {
		world.finishTaskFcn(world.userTreeTask, world.userTaskContext);
		world.userTreeTask = 0;
		world.activeTaskCount -= 1;
	}

	// Update sensors
	{
		let sensorTicks = b3GetTicks();
		b3OverlapSensors(world);
		world.profile.sensors = b3GetMilliseconds(sensorTicks);
	}

	world.profile.step = b3GetMilliseconds(stepTicks);

	// Make sure all tasks that were started were also finished
	B3_ASSERT(world.activeTaskCount == 0);

	// Swap end event array buffers
	world.endEventArrayIndex = 1 - world.endEventArrayIndex;
	world.sensorEndEvents[world.endEventArrayIndex].length = 0;
	world.contactEndEvents[world.endEventArrayIndex].length = 0;
	world.locked = false;
}

export function b3World_GetBounds(worldId: b3WorldId): b3AABB {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return b3AABB();
	}

	let worldBounds = b3AABB();
	let haveBounds = false;

	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		let tree = world.broadPhase.trees[i];
		if (b3DynamicTree_GetProxyCount(tree) == 0) {
			continue;
		}

		let bounds = b3DynamicTree_GetRootBounds(world.broadPhase.trees[i]);

		if (haveBounds) {
			worldBounds = b3AABB_Union(worldBounds, bounds);
		} else {
			worldBounds = bounds;
			haveBounds = true;
		}
	}

	return worldBounds;
}

export function b3World_GetBodyEvents(worldId: b3WorldId): b3BodyEvents {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return new b3BodyEvents();
	}

	let count = world.bodyMoveEvents.length;
	let events = new b3BodyEvents();
	events.moveEvents = world.bodyMoveEvents;
	events.moveCount = count;
	return events;
}

export function b3World_GetSensorEvents(worldId: b3WorldId): b3SensorEvents {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return new b3SensorEvents();
	}

	// Careful to use previous buffer
	let endEventArrayIndex = 1 - world.endEventArrayIndex;

	let beginCount = world.sensorBeginEvents.length;
	let endCount = world.sensorEndEvents[endEventArrayIndex].length;

	let events = new b3SensorEvents();
	events.beginEvents = world.sensorBeginEvents;
	events.endEvents = world.sensorEndEvents[endEventArrayIndex];
	events.beginCount = beginCount;
	events.endCount = endCount;
	return events;
}

export function b3World_GetContactEvents(worldId: b3WorldId): b3ContactEvents {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return new b3ContactEvents();
	}

	// Careful to use previous buffer
	let endEventArrayIndex = 1 - world.endEventArrayIndex;

	let beginCount = world.contactBeginEvents.length;
	let endCount = world.contactEndEvents[endEventArrayIndex].length;
	let hitCount = world.contactHitEvents.length;

	let events = new b3ContactEvents();
	events.beginEvents = world.contactBeginEvents;
	events.endEvents = world.contactEndEvents[endEventArrayIndex];
	events.hitEvents = world.contactHitEvents;
	events.beginCount = beginCount;
	events.endCount = endCount;
	events.hitCount = hitCount;

	return events;
}

export function b3World_GetJointEvents(worldId: b3WorldId): b3JointEvents {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return new b3JointEvents();
	}

	let count = world.jointEvents.length;
	let events = new b3JointEvents();
	events.jointEvents = world.jointEvents;
	events.count = count;
	return events;
}

export function b3World_IsValid(id: b3WorldId): bool {
	if (id.index1 < 1 || B3_MAX_WORLDS < <i32>id.index1) {
		return false;
	}

	let world = b3_worlds[id.index1 - 1];

	if (world.worldId != id.index1 - 1) {
		// world is not allocated
		return false;
	}

	return id.generation == world.generation;
}

export function b3Body_IsValid(id: b3BodyId): bool {
	if (B3_MAX_WORLDS <= <i32>id.world0) {
		// invalid world
		return false;
	}

	let world = b3_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	if (id.index1 < 1 || world.bodies.length < id.index1) {
		// invalid index
		return false;
	}

	let body = world.bodies[id.index1 - 1];
	if (body.setIndex == B3_NULL_INDEX) {
		// this was freed
		return false;
	}

	B3_ASSERT(body.localIndex != B3_NULL_INDEX);

	if (body.generation != id.generation) {
		// this id is orphaned
		return false;
	}

	return true;
}

export function b3Shape_IsValid(id: b3ShapeId): bool {
	if (B3_MAX_WORLDS <= <i32>id.world0) {
		return false;
	}

	let world = b3_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	let shapeId = id.index1 - 1;
	if (shapeId < 0 || world.shapes.length <= shapeId) {
		return false;
	}

	let shape = world.shapes[shapeId];
	if (shape.id == B3_NULL_INDEX) {
		// shape is free
		return false;
	}

	B3_ASSERT(shape.id == shapeId);

	return id.generation == shape.generation;
}

export function b3Joint_IsValid(id: b3JointId): bool {
	if (B3_MAX_WORLDS <= <i32>id.world0) {
		return false;
	}

	let world = b3_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	let jointId = id.index1 - 1;
	if (jointId < 0 || world.joints.length <= jointId) {
		return false;
	}

	let joint = world.joints[jointId];
	if (joint.jointId == B3_NULL_INDEX) {
		// joint is free
		return false;
	}

	B3_ASSERT(joint.jointId == jointId);

	return id.generation == joint.generation;
}

export function b3Contact_IsValid(id: b3ContactId): bool {
	if (B3_MAX_WORLDS <= <i32>id.world0) {
		return false;
	}

	let world = b3_worlds[id.world0];
	if (world.worldId != id.world0) {
		// world is free
		return false;
	}

	let contactId = id.index1 - 1;
	if (contactId < 0 || world.contacts.length <= contactId) {
		return false;
	}

	let contact = world.contacts[contactId];
	if (contact.contactId == B3_NULL_INDEX) {
		// contact is free
		return false;
	}

	B3_ASSERT(contact.contactId == contactId);

	return id.generation == contact.generation;
}

export function b3World_EnableSleeping(worldId: b3WorldId, flag: bool): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	if (flag == world.enableSleep) {
		return;
	}

	world.enableSleep = flag;

	if (flag == false) {
		let setCount = world.solverSets.length;
		for (let i = b3_firstSleepingSet; i < setCount; ++i) {
			let set = world.solverSets[i];
			if (set.bodySims.length > 0) {
				b3WakeSolverSet(world, i);
			}
		}
	}
}

export function b3World_IsSleepingEnabled(worldId: b3WorldId): bool {
	let world = b3GetWorldFromId(worldId);
	return world.enableSleep;
}

export function b3World_EnableWarmStarting(worldId: b3WorldId, flag: bool): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.enableWarmStarting = flag;
}

export function b3World_IsWarmStartingEnabled(worldId: b3WorldId): bool {
	let world = b3GetWorldFromId(worldId);
	return world.enableWarmStarting;
}

export function b3World_EnableSSE2Fallback(worldId: b3WorldId, flag: bool): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.simdWidth = flag ? 4 : b3GetSIMDWidth();
}

export function b3World_GetAwakeBodyCount(worldId: b3WorldId): i32 {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return 0;
	}
	let awakeSet = world.solverSets[b3_awakeSet];
	return awakeSet.bodySims.length;
}

export function b3World_EnableContinuous(worldId: b3WorldId, flag: bool): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.enableContinuous = flag;
}

export function b3World_IsContinuousEnabled(worldId: b3WorldId): bool {
	let world = b3GetWorldFromId(worldId);
	return world.enableContinuous;
}

export function b3World_SetRestitutionThreshold(worldId: b3WorldId, value: f32): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.restitutionThreshold = b3ClampFloat(value, 0.0, FLT_MAX);
}

export function b3World_GetRestitutionThreshold(worldId: b3WorldId): f32 {
	let world = b3GetWorldFromId(worldId);
	return world.restitutionThreshold;
}

export function b3World_SetRestitutionIterations(worldId: b3WorldId, iterations: i32): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	iterations = b3ClampInt(iterations, 0, B3_MAX_RESTITUTION_ITERATIONS);
	if (iterations == world.restitutionIterations) {
		return;
	}

	world.restitutionIterations = iterations;
}

export function b3World_GetRestitutionIterations(worldId: b3WorldId): i32 {
	let world = b3GetWorldFromId(worldId);
	return world.restitutionIterations;
}

export function b3World_EnableRestitutionPropagation(worldId: b3WorldId, flag: bool): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	if (flag == world.enableRestitutionPropagation) {
		return;
	}

	world.enableRestitutionPropagation = flag;
}

export function b3World_IsRestitutionPropagationEnabled(worldId: b3WorldId): bool {
	let world = b3GetWorldFromId(worldId);
	return world.enableRestitutionPropagation;
}

export function b3World_SetHitEventThreshold(worldId: b3WorldId, value: f32): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.hitEventThreshold = b3ClampFloat(value, 0.0, FLT_MAX);
}

export function b3World_GetHitEventThreshold(worldId: b3WorldId): f32 {
	let world = b3GetWorldFromId(worldId);
	return world.hitEventThreshold;
}

export function b3World_SetContactTuning(worldId: b3WorldId, hertz: f32, dampingRatio: f32, contactSpeed: f32): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.contactHertz = b3ClampFloat(hertz, 0.0, FLT_MAX);
	world.contactDampingRatio = b3ClampFloat(dampingRatio, 0.0, FLT_MAX);
	world.contactSpeed = b3ClampFloat(contactSpeed, 0.0, FLT_MAX);
}

export function b3World_SetContactRecycleDistance(worldId: b3WorldId, recycleDistance: f32): void {
	let world = b3GetWorldFromId(worldId);
	B3_ASSERT(world.locked == false);
	if (world.locked) {
		return;
	}

	world.contactRecycleDistance = b3ClampFloat(recycleDistance, 0.0, FLT_MAX);
}

export function b3World_GetContactRecycleDistance(worldId: b3WorldId): f32 {
	let world = b3GetWorldFromId(worldId);
	return world.contactRecycleDistance;
}

export function b3World_SetMaximumLinearSpeed(worldId: b3WorldId, maximumLinearSpeed: f32): void {
	B3_ASSERT(b3IsValidFloat(maximumLinearSpeed) && maximumLinearSpeed > 0.0);

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.maxLinearSpeed = maximumLinearSpeed;
}

export function b3World_GetMaximumLinearSpeed(worldId: b3WorldId): f32 {
	let world = b3GetWorldFromId(worldId);
	return world.maxLinearSpeed;
}

export function b3World_SetIsotropicInertiaTolerance(worldId: b3WorldId, tolerance: f32): void {
	B3_ASSERT(b3IsValidFloat(tolerance));

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.isotropicInertiaTolerance = b3ClampFloat(tolerance, 0.0, 1.0);
}

export function b3World_GetIsotropicInertiaTolerance(worldId: b3WorldId): f32 {
	let world = b3GetWorldFromId(worldId);
	return world.isotropicInertiaTolerance;
}

export function b3World_GetProfile(worldId: b3WorldId): b3Profile {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return b3Profile();
	}
	return world.profile;
}

export function b3World_GetMaxCapacity(worldId: b3WorldId): b3Capacity {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return b3Capacity();
	}
	return world.maxCapacity;
}

export function b3World_SetUserData(worldId: b3WorldId, userData: u64): void {
	let world = b3GetWorldFromId(worldId);
	world.userData = userData;
}

export function b3World_GetUserData(worldId: b3WorldId): u64 {
	let world = b3GetWorldFromId(worldId);
	return world.userData;
}

export function b3World_SetFrictionCallback(worldId: b3WorldId, callback: b3FrictionCallback?): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.frictionCallback = callback != null ? callback : b3DefaultFrictionCallback;
}

export function b3World_SetRestitutionCallback(worldId: b3WorldId, callback: b3RestitutionCallback?): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.restitutionCallback = callback != null ? callback : b3DefaultRestitutionCallback;
}

export function b3World_SetWorkerCount(worldId: b3WorldId, count: i32): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	if (count == world.workerCount) {
		return;
	}

	b3DestroyWorkerContexts(world);
	world.workerCount = b3ClampInt(count, 1, B3_MAX_WORKERS);
	b3CreateWorkerContexts(world);
}

export function b3World_GetWorkerCount(worldId: b3WorldId): i32 {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return 0;
	}

	return world.workerCount;
}

// The byte count walk is not ported (notes/containers.md), so there is nothing to log.
export function b3World_DumpMemoryStats(worldId: b3WorldId): void {}

export function b3World_GetCounters(worldId: b3WorldId): b3Counters {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return b3Counters();
	}

	let s = b3Counters();
	s.bodyCount = b3GetIdCount(world.bodyIdPool);
	s.shapeCount = b3GetIdCount(world.shapeIdPool);
	s.contactCount = b3GetIdCount(world.contactIdPool);
	s.jointCount = b3GetIdCount(world.jointIdPool);
	s.islandCount = b3GetIdCount(world.islandIdPool);

	let staticTree = world.broadPhase.trees[b3_staticBody];
	s.staticTreeHeight = b3DynamicTree_GetHeight(staticTree);

	let dynamicTree = world.broadPhase.trees[b3_dynamicBody];
	let kinematicTree = world.broadPhase.trees[b3_kinematicBody];
	s.treeHeight = b3MaxInt(b3DynamicTree_GetHeight(dynamicTree), b3DynamicTree_GetHeight(kinematicTree));

	s.satCallCount = world.satCallCount;
	s.satCacheHitCount = world.satCacheHitCount;
	for (let i = 0; i < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++i) {
		s.manifoldCounts[i] = world.manifoldCounts[i];
	}
	// No stack or arena allocator, so stackUsed, byteCount and arenaCapacity read 0.
	s.stackUsed = 0;
	s.byteCount = 0;
	s.taskCount = world.taskCount;

	s.awakeContactCount = 0;
	for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
		let color = world.constraintGraph.colors[i];
		let colorContactCount = color.convexContacts.length + color.contacts.length;
		s.colorCounts[i] = colorContactCount + color.jointSims.length;
		s.awakeContactCount += colorContactCount;
	}
	s.awakeContactCount += world.solverSets[b3_awakeSet].contactIndices.length;

	s.recycledContactCount = 0;
	s.arenaCapacity = 0;
	s.distanceIterations = 0;
	s.pushBackIterations = 0;
	s.rootIterations = 0;
	for (let i = 0; i < world.workerCount; ++i) {
		s.recycledContactCount += world.taskContexts[i].recycledContactCount;

		s.distanceIterations = b3MaxInt(s.distanceIterations, world.taskContexts[i].distanceIterations);
		s.pushBackIterations = b3MaxInt(s.pushBackIterations, world.taskContexts[i].pushBackIterations);
		s.rootIterations = b3MaxInt(s.rootIterations, world.taskContexts[i].rootIterations);
	}

	return s;
}

// C keeps each query's context on its stack. A user callback may start another query, so contexts
// are pooled by nesting depth, and the tree callbacks, which capture nothing and so allocate
// nothing, read the innermost one.
function b3AcquireQueryContext<T>(pool: Array<T>, depth: i32, create: () => T): T {
	if (depth == pool.length) {
		pool.push(create());
	}
	return pool[depth];
}

function b3NoOverlap(shapeId: b3ShapeId): bool {
	return false;
}

class WorldQueryContext {
	world: b3World? = null;
	fcn: b3OverlapResultFcn = b3NoOverlap;
	filter: b3QueryFilter = b3QueryFilter(0, 0, 0);
}

const s_queryContexts = new Array<WorldQueryContext>();
let s_queryDepth = 0;

function b3QueryTreeCallback(proxyId: i32, userData: u64): bool {
	return TreeQueryCallback(proxyId, userData, s_queryContexts[s_queryDepth - 1]);
}

function TreeQueryCallback(proxyId: i32, userData: u64, worldContext: WorldQueryContext): bool {
	let shapeId = <i32>userData;
	let world = worldContext.world!;

	let shape = world.shapes[shapeId];

	let shapeFilter = shape.filter;
	let queryFilter = worldContext.filter;

	if (b3ShouldQueryCollide(shapeFilter, queryFilter) == false) {
		return true;
	}

	let id = b3MakeShapeId(shapeId + 1, world.worldId, shape.generation);
	let result = worldContext.fcn(id);
	return result;
}

export function b3World_OverlapAABB(
	worldId: b3WorldId,
	aabb: b3AABB,
	filter: b3QueryFilter,
	fcn: b3OverlapResultFcn,
): b3TreeStats {
	let treeStats = b3TreeStats();

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return treeStats;
	}

	B3_ASSERT(b3IsValidAABB(aabb));

	let worldContext = b3AcquireQueryContext(s_queryContexts, s_queryDepth, (): WorldQueryContext => new WorldQueryContext());
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	s_queryDepth += 1;

	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		let treeResult = b3DynamicTree_Query(world.broadPhase.trees[i], aabb, filter.maskBits, false, b3QueryTreeCallback);

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	s_queryDepth -= 1;
	return treeStats;
}

class WorldOverlapContext {
	world: b3World? = null;
	fcn: b3OverlapResultFcn = b3NoOverlap;
	filter: b3QueryFilter = b3QueryFilter(0, 0, 0);
	proxy: b3ShapeProxy = new b3ShapeProxy();
	origin: b3Pos = b3Vec3();
}

const s_overlapContexts = new Array<WorldOverlapContext>();
let s_overlapDepth = 0;

function b3OverlapTreeCallback(proxyId: i32, userData: u64): bool {
	return b3TreeOverlapCallback(proxyId, userData, s_overlapContexts[s_overlapDepth - 1]);
}

function b3TreeOverlapCallback(proxyId: i32, userData: u64, worldContext: WorldOverlapContext): bool {
	let shapeId = <i32>userData;
	let world = worldContext.world!;

	let shape = world.shapes[shapeId];

	let shapeFilter = shape.filter;
	let queryFilter = worldContext.filter;

	if (b3ShouldQueryCollide(shapeFilter, queryFilter) == false) {
		return true;
	}

	// Re-center on the query origin so the overlap test stays in float precision far from the origin
	let transform = b3ToRelativeTransform(b3GetBodyTransformQuick(world, shape.bodyId), worldContext.origin);

	let overlapping = b3OverlapShape(shape, transform, worldContext.proxy);
	if (overlapping == false) {
		return true;
	}

	let id = b3MakeShapeId(shape.id + 1, world.worldId, shape.generation);
	let result = worldContext.fcn(id);
	return result;
}

export function b3World_OverlapShape(
	worldId: b3WorldId,
	origin: b3Pos,
	proxy: b3ShapeProxy,
	filter: b3QueryFilter,
	fcn: b3OverlapResultFcn,
): b3TreeStats {
	let treeStats = b3TreeStats();

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return treeStats;
	}

	B3_ASSERT(b3IsValidPosition(origin));

	// Bound the proxy in origin relative space then lift to a conservative world float box
	let aabb = b3OffsetAABB(b3MakeAABB(proxy.points, proxy.count, proxy.radius), origin);
	let worldContext = b3AcquireQueryContext(s_overlapContexts, s_overlapDepth, (): WorldOverlapContext => new WorldOverlapContext());
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.proxy = proxy;
	worldContext.origin = origin;
	s_overlapDepth += 1;

	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		let treeResult = b3DynamicTree_Query(world.broadPhase.trees[i], aabb, filter.maskBits, false, b3OverlapTreeCallback);

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	s_overlapDepth -= 1;
	return treeStats;
}

function b3NoPlanes(shapeId: b3ShapeId, planeResults: StaticArray<b3PlaneResult>, planeCount: i32): bool {
	return false;
}

class WorldMoverContext {
	world: b3World? = null;
	fcn: b3PlaneResultFcn = b3NoPlanes;
	filter: b3QueryFilter = b3QueryFilter(0, 0, 0);
	mover: b3Capsule = b3Capsule();
	origin: b3Pos = b3Vec3();

	// C's stack buffer, reused for every shape of the query
	buffer: StaticArray<b3PlaneResult> = new StaticArray<b3PlaneResult>(64);
}

const s_moverContexts = new Array<WorldMoverContext>();
let s_moverDepth = 0;

function b3MoverTreeCallback(proxyId: i32, userData: u64): bool {
	return TreeCollideCallback(proxyId, userData, s_moverContexts[s_moverDepth - 1]);
}

function TreeCollideCallback(proxyId: i32, userData: u64, worldContext: WorldMoverContext): bool {
	let shapeId = <i32>userData;
	let world = worldContext.world!;

	let shape = world.shapes[shapeId];

	let shapeFilter = shape.filter;
	let queryFilter = worldContext.filter;

	if (b3ShouldQueryCollide(shapeFilter, queryFilter) == false) {
		return true;
	}

	// Re-center on the query origin, the mover and the resulting planes are origin relative
	let bodyTransform = b3GetBodyTransformQuick(world, shape.bodyId);
	let transform = b3ToRelativeTransform(bodyTransform, worldContext.origin);

	let buffer = worldContext.buffer;
	let count = b3CollideMover(buffer, 64, shape, transform, worldContext.mover);

	if (count > 0) {
		let id = b3MakeShapeId(shape.id + 1, world.worldId, shape.generation);
		return worldContext.fcn(id, buffer, count);
	}

	return true;
}

// It is tempting to use a shape proxy for the mover, but this makes handling deep overlap difficult and the
// generality may not be worth it.
export function b3World_CollideMover(
	worldId: b3WorldId,
	origin: b3Pos,
	mover: b3Capsule,
	filter: b3QueryFilter,
	fcn: b3PlaneResultFcn,
): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	B3_ASSERT(b3IsValidPosition(origin));

	let r = b3Vec3(mover.radius, mover.radius, mover.radius);

	let relBox: b3AABB;
	relBox.lowerBound = b3Sub(b3Min(mover.center1, mover.center2), r);
	relBox.upperBound = b3Add(b3Max(mover.center1, mover.center2), r);
	let aabb = b3OffsetAABB(relBox, origin);

	let worldContext = b3AcquireQueryContext(s_moverContexts, s_moverDepth, (): WorldMoverContext => new WorldMoverContext());
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.mover = mover;
	worldContext.origin = origin;
	s_moverDepth += 1;

	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		b3DynamicTree_Query(world.broadPhase.trees[i], aabb, filter.maskBits, false, b3MoverTreeCallback);
	}

	s_moverDepth -= 1;
}

function b3NoCast(
	shapeId: b3ShapeId,
	point: b3Pos,
	normal: b3Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
): f32 {
	return 0.0;
}

class WorldRayCastContext {
	world: b3World? = null;
	fcn: b3CastResultFcn = b3NoCast;
	filter: b3QueryFilter = b3QueryFilter(0, 0, 0);
	fraction: f32 = 0;
	origin: b3Pos = b3Vec3();

	// C's b3RayResult on the stack of b3World_CastRayClosest
	result: b3RayResult = b3RayResult();
}

const s_rayContexts = new Array<WorldRayCastContext>();
let s_rayDepth = 0;

function b3AcquireRayContext(world: b3World, fcn: b3CastResultFcn, filter: b3QueryFilter, fraction: f32, origin: b3Pos): WorldRayCastContext {
	let worldContext = b3AcquireQueryContext(s_rayContexts, s_rayDepth, (): WorldRayCastContext => new WorldRayCastContext());
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.fraction = fraction;
	worldContext.origin = origin;
	s_rayDepth += 1;
	return worldContext;
}

function b3RayTreeCallback(input: b3RayCastInput, proxyId: i32, userData: u64): f32 {
	return RayCastCallback(input, proxyId, userData, s_rayContexts[s_rayDepth - 1]);
}

function RayCastCallback(input: b3RayCastInput, proxyId: i32, userData: u64, worldContext: WorldRayCastContext): f32 {
	let shapeId = <i32>userData;
	let world = worldContext.world!;

	let shape = world.shapes[shapeId];
	let shapeFilter = shape.filter;
	let queryFilter = worldContext.filter;

	if (b3ShouldQueryCollide(shapeFilter, queryFilter) == false) {
		return input.maxFraction;
	}

	let bodyTransform = b3GetBodyTransformQuick(world, shape.bodyId);
	let transform = b3ToRelativeTransform(bodyTransform, worldContext.origin);

	let localInput = input;
	localInput.origin = b3Vec3_zero;
	let output = b3RayCastShape(shape, transform, localInput);

	if (output.hit) {
		B3_ASSERT(output.fraction <= input.maxFraction);

		let id = b3MakeShapeId(shapeId + 1, world.worldId, shape.generation);
		let point = b3OffsetPos(worldContext.origin, output.point);
		let materialIndex = b3ClampInt(output.materialIndex, 0, shape.materialCount - 1);
		let userMaterialId = b3GetShapeMaterials(shape)[materialIndex].userMaterialId;

		let triangleIndex = output.triangleIndex;
		let childIndex = output.childIndex;
		let fraction = worldContext.fcn(
			id,
			point,
			output.normal,
			output.fraction,
			userMaterialId,
			triangleIndex,
			childIndex,
		);

		// The user may return -1 to skip this shape
		if (0.0 <= fraction && fraction <= 1.0) {
			worldContext.fraction = fraction;
		}

		return fraction;
	}

	return input.maxFraction;
}

export function b3World_CastRay(
	worldId: b3WorldId,
	origin: b3Pos,
	translation: b3Vec3,
	filter: b3QueryFilter,
	fcn: b3CastResultFcn,
): b3TreeStats {
	let treeStats = b3TreeStats();

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return treeStats;
	}

	B3_ASSERT(b3IsValidPosition(origin));
	B3_ASSERT(b3IsValidVec3(translation));

	// The tree traverses in float relative to the world origin. Each shape is then re-differenced at
	// full precision against the origin, so a hit stays accurate far from the origin.
	let input = b3RayCastInput(b3ToVec3(origin), translation, 1.0);

	let worldContext = b3AcquireRayContext(world, fcn, filter, 1.0, origin);

	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		let treeResult = b3DynamicTree_RayCast(world.broadPhase.trees[i], input, filter.maskBits, false, b3RayTreeCallback);
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if (worldContext.fraction == 0.0) {
			break;
		}

		input.maxFraction = worldContext.fraction;
	}

	s_rayDepth -= 1;
	return treeStats;
}

// This callback finds the closest hit. This is the most common callback used in games.
function b3RayCastClosestFcn(
	shapeId: b3ShapeId,
	point: b3Pos,
	normal: b3Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
): f32 {
	// Ignore initial overlap
	if (fraction == 0.0) {
		return -1.0;
	}

	// The innermost ray context is the one of the b3World_CastRayClosest that is calling
	ref rayResult = s_rayContexts[s_rayDepth - 1].result;

	rayResult.shapeId = shapeId;
	rayResult.point = point;
	rayResult.normal = normal;
	rayResult.fraction = fraction;
	rayResult.userMaterialId = userMaterialId;
	rayResult.triangleIndex = triangleIndex;
	rayResult.childIndex = childIndex;
	rayResult.hit = true;
	return fraction;
}

export function b3World_CastRayClosest(
	worldId: b3WorldId,
	origin: b3Pos,
	translation: b3Vec3,
	filter: b3QueryFilter,
): b3RayResult {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return b3RayResult();
	}

	B3_ASSERT(b3IsValidPosition(origin));
	B3_ASSERT(b3IsValidVec3(translation));

	// The tree traverses in float relative to the world origin. Each shape is then re-differenced at
	// full precision against its body, so a hit stays accurate far from the origin.
	let input = b3RayCastInput(b3ToVec3(origin), translation, 1.0);
	let worldContext = b3AcquireRayContext(world, b3RayCastClosestFcn, filter, 1.0, origin);
	worldContext.result = b3RayResult();

	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		let treeResult = b3DynamicTree_RayCast(world.broadPhase.trees[i], input, filter.maskBits, false, b3RayTreeCallback);
		worldContext.result.nodeVisits += treeResult.nodeVisits;
		worldContext.result.leafVisits += treeResult.leafVisits;

		if (worldContext.fraction == 0.0) {
			break;
		}

		input.maxFraction = worldContext.fraction;
	}

	s_rayDepth -= 1;
	return worldContext.result;
}

class WorldShapeCastContext {
	world: b3World? = null;
	fcn: b3CastResultFcn = b3NoCast;
	filter: b3QueryFilter = b3QueryFilter(0, 0, 0);
	fraction: f32 = 0;
	origin: b3Pos = b3Vec3();
	// origin relative input
	input: b3ShapeCastInput = new b3ShapeCastInput();
}

const s_shapeCastContexts = new Array<WorldShapeCastContext>();
let s_shapeCastDepth = 0;

function b3ShapeCastTreeCallback(input: b3BoxCastInput, proxyId: i32, userData: u64): f32 {
	return b3ShapeCastCallback(input, proxyId, userData, s_shapeCastContexts[s_shapeCastDepth - 1]);
}

function b3ShapeCastCallback(
	input: b3BoxCastInput,
	proxyId: i32,
	userData: u64,
	worldContext: WorldShapeCastContext,
): f32 {
	let shapeId = <i32>userData;
	let world = worldContext.world!;

	let shape = world.shapes[shapeId];
	let shapeFilter = shape.filter;
	let queryFilter = worldContext.filter;

	if (b3ShouldQueryCollide(shapeFilter, queryFilter) == false) {
		return input.maxFraction;
	}

	// Rebuild from the origin relative input, taking only the advancing fraction from the tree.
	// The tree box is world float and would lose the cast far from the origin.
	let localInput = b3CopyShapeCastInput(worldContext.input);
	localInput.maxFraction = input.maxFraction;

	// Re-center on the query origin so the per-shape cast stays in float precision far from the origin
	let transform = b3ToRelativeTransform(b3GetBodyTransformQuick(world, shape.bodyId), worldContext.origin);

	let output = b3ShapeCastShape(shape, transform, localInput);

	if (output.hit) {
		let id = b3MakeShapeId(shapeId + 1, world.worldId, shape.generation);
		let materialIndex = b3ClampInt(output.materialIndex, 0, shape.materialCount - 1);
		let userMaterialId = b3GetShapeMaterials(shape)[materialIndex].userMaterialId;

		let triangleIndex = output.triangleIndex;
		let childIndex = output.childIndex;
		let fraction = worldContext.fcn(
			id,
			b3OffsetPos(worldContext.origin, output.point),
			output.normal,
			output.fraction,
			userMaterialId,
			triangleIndex,
			childIndex,
		);

		// The user may return -1 to skip this shape
		if (0.0 <= fraction && fraction <= 1.0) {
			worldContext.fraction = fraction;
		}

		return fraction;
	}

	return input.maxFraction;
}

// C copies the struct onto its stack; the proxy points alias as C's pointer does. The copy is read
// only until the per shape cast returns, before any user callback, so one serves every query.
const s_shapeCastInputCopy = new b3ShapeCastInput();

function b3CopyShapeCastInput(input: b3ShapeCastInput): b3ShapeCastInput {
	let copy = s_shapeCastInputCopy;
	copy.proxy = input.proxy;
	copy.translation = input.translation;
	copy.maxFraction = input.maxFraction;
	copy.canEncroach = input.canEncroach;
	return copy;
}

export function b3World_CastShape(
	worldId: b3WorldId,
	origin: b3Pos,
	proxy: b3ShapeProxy,
	translation: b3Vec3,
	filter: b3QueryFilter,
	fcn: b3CastResultFcn,
): b3TreeStats {
	let treeStats = b3TreeStats();

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return treeStats;
	}

	B3_ASSERT(b3IsValidPosition(origin));
	B3_ASSERT(b3IsValidVec3(translation));

	let worldContext = b3AcquireQueryContext(s_shapeCastContexts, s_shapeCastDepth, (): WorldShapeCastContext => new WorldShapeCastContext());
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.fraction = 1.0;
	worldContext.origin = origin;
	worldContext.input.proxy = proxy;
	worldContext.input.translation = translation;
	worldContext.input.maxFraction = 1.0;
	worldContext.input.canEncroach = false;

	// Bound the proxy in origin relative space then lift to a conservative world float box. The tree
	// node boxes use the same directed rounding, so the swept box never clips a shape far from the
	// origin. Per shape casts re-difference at full precision against the carried origin.
	let localBox = b3MakeAABB(proxy.points, proxy.count, proxy.radius);
	let treeInput = b3BoxCastInput(b3OffsetAABB(localBox, origin), translation, 1.0);

	s_shapeCastDepth += 1;

	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		let treeResult = b3DynamicTree_BoxCast(world.broadPhase.trees[i], treeInput, filter.maskBits, false, b3ShapeCastTreeCallback);
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if (worldContext.fraction == 0.0) {
			break;
		}

		treeInput.maxFraction = worldContext.fraction;
	}

	s_shapeCastDepth -= 1;
	return treeStats;
}

class WorldMoverCastContext {
	world: b3World? = null;
	fcn: b3MoverFilterFcn? = null;
	filter: b3QueryFilter = b3QueryFilter(0, 0, 0);
	fraction: f32 = 0;
	origin: b3Pos = b3Vec3();
	// origin relative input
	input: b3ShapeCastInput = new b3ShapeCastInput();

	// C points the proxy at the mover's center1 and center2
	centers: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(2);
}

const s_moverCastContexts = new Array<WorldMoverCastContext>();
let s_moverCastDepth = 0;

function b3MoverCastTreeCallback(input: b3BoxCastInput, proxyId: i32, userData: u64): f32 {
	return MoverCastCallback(input, proxyId, userData, s_moverCastContexts[s_moverCastDepth - 1]);
}

function MoverCastCallback(
	input: b3BoxCastInput,
	proxyId: i32,
	userData: u64,
	worldContext: WorldMoverCastContext,
): f32 {
	let shapeId = <i32>userData;
	let world = worldContext.world!;

	let shape = world.shapes[shapeId];
	let shapeFilter = shape.filter;
	let queryFilter = worldContext.filter;

	if (b3ShouldQueryCollide(shapeFilter, queryFilter) == false) {
		return worldContext.fraction;
	}

	let fcn = worldContext.fcn;
	if (fcn != null) {
		let id = b3MakeShapeId(shapeId + 1, world.worldId, shape.generation);
		let shouldCollide = fcn(id);
		if (shouldCollide == false) {
			return worldContext.fraction;
		}
	}

	// Rebuild from the origin relative input, taking only the advancing fraction from the tree
	let localInput = b3CopyShapeCastInput(worldContext.input);
	localInput.maxFraction = input.maxFraction;

	// Re-center on the query origin so the per-shape cast stays in float precision far from the origin
	let transform = b3ToRelativeTransform(b3GetBodyTransformQuick(world, shape.bodyId), worldContext.origin);

	let output = b3ShapeCastShape(shape, transform, localInput);
	if (output.fraction == 0.0) {
		// Ignore overlapping shapes
		return worldContext.fraction;
	}

	worldContext.fraction = output.fraction;
	return output.fraction;
}

export function b3World_CastMover(
	worldId: b3WorldId,
	origin: b3Pos,
	mover: b3Capsule,
	translation: b3Vec3,
	filter: b3QueryFilter,
	fcn: b3MoverFilterFcn?,
): f32 {
	B3_ASSERT(b3IsValidPosition(origin));
	B3_ASSERT(b3IsValidVec3(translation));

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return 1.0;
	}

	let worldContext = b3AcquireQueryContext(s_moverCastContexts, s_moverCastDepth, (): WorldMoverCastContext => new WorldMoverCastContext());

	// C points the proxy at center1 and center2, which are adjacent fields.
	let centers = worldContext.centers;
	centers[0] = mover.center1;
	centers[1] = mover.center2;

	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.fraction = 1.0;
	worldContext.origin = origin;
	worldContext.input.proxy = b3SetProxy(worldContext.input.proxy, centers, 2, mover.radius);
	worldContext.input.translation = translation;
	worldContext.input.maxFraction = 1.0;
	worldContext.input.canEncroach = mover.radius > 0.0;

	// Bound the capsule in origin relative space then lift to a conservative world float box
	let treeInput = b3BoxCastInput(b3OffsetAABB(b3MakeAABB(centers, 2, mover.radius), origin), translation, 1.0);

	s_moverCastDepth += 1;

	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		b3DynamicTree_BoxCast(world.broadPhase.trees[i], treeInput, filter.maskBits, false, b3MoverCastTreeCallback);

		if (worldContext.fraction == 0.0) {
			break;
		}

		treeInput.maxFraction = worldContext.fraction;
	}

	s_moverCastDepth -= 1;
	return worldContext.fraction;
}

export function b3World_SetCustomFilterCallback(worldId: b3WorldId, fcn: b3CustomFilterFcn?): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}
	world.customFilterFcn = fcn;
}

export function b3World_SetPreSolveCallback(worldId: b3WorldId, fcn: b3PreSolveFcn?): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}
	world.preSolveFcn = fcn;
}

export function b3World_SetGravity(worldId: b3WorldId, gravity: b3Vec3): void {
	let world = b3GetWorldFromId(worldId);

	world.gravity = gravity;
}

export function b3World_GetGravity(worldId: b3WorldId): b3Vec3 {
	let world = b3GetWorldFromId(worldId);
	return world.gravity;
}

class ExplosionContext {
	world: b3World;
	position: b3Pos;
	radius: f32;
	falloff: f32;
	impulsePerArea: f32;

	constructor(world: b3World, position: b3Pos, radius: f32, falloff: f32, impulsePerArea: f32) {
		this.world = world;
		this.position = position;
		this.radius = radius;
		this.falloff = falloff;
		this.impulsePerArea = impulsePerArea;
	}
}

function ExplosionCallback(proxyId: i32, userData: u64, explosionContext: ExplosionContext): bool {
	let shapeId = <i32>userData;
	let world = explosionContext.world;

	let shape = world.shapes[shapeId];
	if (shape.explosionScale == 0.0) {
		return true;
	}

	let bodyId = shape.bodyId;
	B3_ASSERT(world.bodies[bodyId].type == b3_dynamicBody);

	let xf = b3GetBodyTransformQuick(world, bodyId);

	// Re-center the explosion into the shape local frame so distance and direction stay precise
	// far from the origin. Everything below runs in that near-origin frame.
	let localPosition = b3InvTransformWorldPoint(xf, explosionContext.position);

	let localPoints = new StaticArray<b3Vec3>(1);
	localPoints[0] = localPosition;

	let input = new b3DistanceInput();
	input.proxyA = b3MakeShapeProxy(shape);
	input.proxyB = new b3ShapeProxy(localPoints, 1, 0.0);
	input.transform = b3Transform_identity;
	input.useRadii = true;

	let cache = b3SimplexCache();
	let output = b3ShapeDistance(input, cache, null, 0);

	let radius = explosionContext.radius;
	let falloff = explosionContext.falloff;
	if (output.distance > radius + falloff) {
		return true;
	}

	b3WakeBody(world, bodyId);

	// Waking moves the body between sets, so read it after.
	if (world.bodies[bodyId].setIndex != b3_awakeSet) {
		return true;
	}

	// Witness point is already in the body local query frame
	let closestPoint = output.pointA;
	if (output.distance == 0.0) {
		closestPoint = b3GetShapeCentroid(shape);
	}

	let direction = b3Sub(closestPoint, localPosition);
	if (b3LengthSquared(direction) > 100.0 * FLT_EPSILON * FLT_EPSILON) {
		direction = b3Normalize(direction);
	} else {
		direction = b3Vec3(1.0, 0.0, 0.0);
	}

	let area = b3GetShapeProjectedArea(shape, direction);
	let scale: f32 = 1.0;
	if (output.distance > radius && falloff > 0.0) {
		scale = b3ClampFloat((radius + falloff - output.distance) / falloff, 0.0, 1.0);
	}

	let magnitude = explosionContext.impulsePerArea * area * scale * shape.explosionScale;
	let impulse = b3MulSV(magnitude, b3RotateVector(xf.q, direction));

	let localIndex = world.bodies[bodyId].localIndex;
	let set = world.solverSets[b3_awakeSet];
	let bodySim = set.bodySims[localIndex];
	set.bodyStates[localIndex].linearVelocity = b3MulAdd(
		set.bodyStates[localIndex].linearVelocity,
		bodySim.invMass,
		impulse,
	);

	// Lever arm from the center of mass to the closest point, rotated to world
	let r = b3RotateVector(xf.q, b3Sub(closestPoint, bodySim.localCenter));
	set.bodyStates[localIndex].angularVelocity = b3Add(
		set.bodyStates[localIndex].angularVelocity,
		b3MulMV(bodySim.invInertiaWorld, b3Cross(r, impulse)),
	);

	return true;
}

export function b3World_Explode(worldId: b3WorldId, explosionDef: b3ExplosionDef): void {
	let maskBits = explosionDef.maskBits;
	let position = explosionDef.position;
	let radius = explosionDef.radius;
	let falloff = explosionDef.falloff;
	let impulsePerArea = explosionDef.impulsePerArea;

	B3_ASSERT(b3IsValidPosition(position));
	B3_ASSERT(b3IsValidFloat(radius) && radius >= 0.0);
	B3_ASSERT(b3IsValidFloat(falloff) && falloff >= 0.0);
	B3_ASSERT(b3IsValidFloat(impulsePerArea));

	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	// Locked due to waking
	world.locked = true;

	let explosionContext = new ExplosionContext(world, position, radius, falloff, impulsePerArea);

	// The broad-phase tree is float, so translate a local query box out to world with outward rounding
	let extent = radius + falloff;
	let localBox = b3AABB(b3Vec3(-extent, -extent, -extent), b3Vec3(extent, extent, extent));
	let aabb = b3OffsetAABB(localBox, position);

	b3DynamicTree_Query(
		world.broadPhase.trees[b3_dynamicBody],
		aabb,
		maskBits,
		false,
		(proxyId: i32, userData: u64): bool => ExplosionCallback(proxyId, userData, explosionContext),
	);

	world.locked = false;
}

export function b3World_RebuildStaticTree(worldId: b3WorldId): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	let staticTree = world.broadPhase.trees[b3_staticBody];
	b3DynamicTree_Rebuild(staticTree, true);
}

export function b3World_EnableSpeculative(worldId: b3WorldId, flag: bool): void {
	let world = b3GetUnlockedWorldFromId(worldId);
	if (world == null) {
		return;
	}

	world.enableSpeculative = flag;
}

// B3_ENABLE_VALIDATION is off, so these are C's empty definitions.
export function b3ValidateConnectivity(world: b3World): void {}

export function b3ValidateSolverSets(world: b3World): void {}

export function b3ValidateContacts(world: b3World): void {}
