// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/physics_world.c

import { b3BitSet } from "./bitset";
import { b3IdPool } from "./id_pool";
import { b3AABB, b3Vec3 } from "./math_functions";
import { b3HullData, b3Manifold } from "./collision";
import {
	b3BodyMoveEvent,
	b3Capacity,
	b3ContactBeginTouchEvent,
	b3ContactEndTouchEvent,
	b3ContactHitEvent,
	b3CustomFilterFcn,
	b3FrictionCallback,
	b3JointEvent,
	b3PreSolveFcn,
	b3Profile,
	b3RestitutionCallback,
	b3SensorBeginTouchEvent,
	b3SensorEndTouchEvent,
} from "./types";
import { b3Body } from "./body";
import { b3BroadPhase } from "./broad_phase";
import { b3ConstraintGraph } from "./constraint_graph";
import { b3Contact, b3_emptyManifolds } from "./contact";
import { b3Island } from "./island";
import { b3Joint } from "./joint";
import { b3Sensor, b3SensorHit, b3SensorTaskContext } from "./sensor";
import { b3Shape } from "./shape";
import { b3SolverSet } from "./solver_set";

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
}

// One shared hull with its reference count.
export class b3HullDatabaseEntry {
	hull: b3HullData;
	referenceCount: i32;

	constructor(hull: b3HullData, referenceCount: i32) {
		this.hull = hull;
		this.referenceCount = referenceCount;
	}
}

// Replaces the verstable map keyed by hull content: buckets by b3HullData.hash, then compare content.
export class b3HullDatabase {
	buckets: Map<u64, Array<b3HullDatabaseEntry>> = new Map<u64, Array<b3HullDatabaseEntry>>();
	count: i32 = 0;
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
	hullDatabase: b3HullDatabase = new b3HullDatabase();

	// Name cache for shape and body names, keyed by name id. Replaces b3NameCache.
	names: Map<u32, string> = new Map<u32, string>();

	// This is a dense array of sensor data.
	sensors: Array<b3Sensor> = new Array<b3Sensor>();

	// Per thread storage
	taskContexts: Array<b3TaskContext> = new Array<b3TaskContext>();
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
