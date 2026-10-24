// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/solver.c

// Solver work is partitioned into fixed-size blocks that worker threads claim
// in parallel via atomic CAS on a per-block syncIndex. The port runs one worker,
// but keeps the block and stage structure so the solve order matches C.
// The solver threading model is inspired by https://github.com/bepu/bepuphysics2

import {
	B3_ASSERT,
	B3_NULL_INDEX,
	FLT_MAX,
	UINT16_MAX,
	UINT32_MAX,
	UINT8_MAX,
	b3AtomicInt,
	b3AtomicU32,
	b3GetMilliseconds,
	b3GetMillisecondsAndReset,
	b3GetTicks,
	b3Log,
	b3Ticks,
} from "./core";
import {
	B3_PI,
	b3AABB_Contains,
	b3AABB_Inflate,
	b3AABB_Union,
	b3Abs,
	b3Add,
	b3Blend2,
	b3Distance,
	b3Dot,
	b3InvMulQuat,
	b3InvRotateVector,
	b3IsValidVec3,
	b3Length,
	b3Lerp,
	b3LerpPosition,
	b3MakeMatrixFromQuat,
	b3Matrix3,
	b3Quat,
	b3MaxFloat,
	b3MaxInt,
	b3MinInt,
	b3MulAdd,
	b3MulMV,
	b3MulQuat,
	b3MulSV,
	b3NLerp,
	b3Neg,
	b3NormalizeQuat,
	b3OffsetAABB,
	b3OffsetPos,
	b3Pos,
	b3RotateVector,
	b3Solve3,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Vec3,
} from "./math_functions";
import { b3IntegrateRotation, b3ModifiedCross } from "./math_internal";
import { b3StoreRow } from "./simd";
import { b3FarthestPointOnAABB } from "./aabb";
import {
	B3_GRAPH_COLOR_COUNT,
	B3_GYROSCOPIC_ITERATIONS,
	B3_MAX_ROTATION,
	B3_MAX_TASKS,
	B3_MAX_WORKERS,
	B3_SPECULATIVE_DISTANCE,
	B3_TIME_TO_SLEEP,
} from "./constants";
import { b3GetBit, b3InPlaceUnion, b3SetBit, b3SetBitCountAndClear } from "./bitset";
import { b3CTZ64 } from "./ctz";
import { b3GetIdCapacity } from "./id_pool";
import { b3Sweep } from "./collision";
import {
	B3_DEFAULT_MASK_BITS,
	b3BodyId,
	b3BodyMoveEvent,
	b3ContactHitEvent,
	b3ContactId,
	b3JointEvent,
	b3JointId,
	b3ShapeId,
	b3_dynamicBody,
	b3_heightShape,
	b3_hullShape,
	b3_capsuleShape,
	b3_kinematicBody,
	b3_meshShape,
	b3_sphereShape,
	b3_staticBody,
} from "./types";
import { b3DynamicTree_EnlargeProxy, b3DynamicTree_Query, b3DynamicTree_Refit } from "./dynamic_tree";
import { b3DynamicTree_GetAABB } from "./collision";
import {
	b3ArrayData,
	b3Body,
	b3BodySim,
	b3BodyState,
	b3GetBodySim,
	b3GetBodySimAddress,
	b3MakeRelativeSweepAt,
	b3ShouldBodiesCollide,
	b3_allowFastRotation,
	b3_bodyTransientFlags,
	b3_dirtyMass,
	b3_enableGyroscopicTorque,
	b3_enableSleep,
	b3_enlargeBulletBounds,
	b3_hadTimeOfImpact,
	b3_isBullet,
	b3_isFast,
	b3_isSpeedCapped,
	b3_lockAngularX,
	b3_lockAngularY,
	b3_lockAngularZ,
	b3_lockLinearX,
	b3_lockLinearY,
	b3_lockLinearZ,
} from "./body";
import { b3ContactSpec } from "./contact";
import {
	b3JointSim,
	b3GetJointReaction,
	b3PrepareJoint,
	b3PrepareJoints_Overflow,
	b3SolveJoint,
	b3SolveJoints_Overflow,
	b3WarmStartJoint,
	b3WarmStartJoints_Overflow,
} from "./joint";
import { b3TaskContext, b3World, b3_awakeSet } from "./physics_world";
import { B3_OVERFLOW_INDEX, b3ConstraintGraph } from "./constraint_graph";
import {
	b3ContactConstraint,
	b3ContactConstraintWide,
	b3ContactManifoldWide,
	b3ManifoldConstraint,
	b3ApplyRestitution_Convex,
	b3ApplyRestitution_MeshWide,
	b3ApplyRestitution_Overflow,
	b3PrepareContacts_Convex,
	b3PrepareContacts_MeshWide,
	b3PrepareContacts_Overflow,
	b3PushContacts_Convex,
	b3PushContacts_MeshWide,
	b3PushContacts_Overflow,
	b3SolveContacts_Convex,
	b3SolveContacts_MeshWide,
	b3SolveContacts_Overflow,
	b3StoreImpulses_Convex,
	b3StoreImpulses_MeshWide,
	b3StoreImpulses_Overflow,
	b3WarmStartContacts_Convex,
	b3WarmStartContacts_MeshWide,
	b3WarmStartContacts_Overflow,
} from "./contact_solver";
import { b3SensorHit, b3Visitor } from "./sensor";
import {
	b3_enableCustomFiltering,
	b3_enablePreSolveEvents,
	b3_enableSensorEvents,
} from "./shape";
import { b3Shape, b3ComputeFatShapeAABB, b3ComputeShapeAABB, b3ComputeShapeMinExtent, b3GetShapeUserMaterialId, b3ShapeTimeOfImpact, b3ShouldShapesCollide } from "./shape";
import { B3_PROXY_ID, b3BroadPhase_MarkProxyMoved } from "./broad_phase";
import { b3ParallelFor } from "./parallel_for";
import { b3SplitIslandTask } from "./island";
import { b3FindNameWithDefault } from "./name_cache";
import { b3TrySleepIsland } from "./solver_set";

// Solver stages
export type b3SolverStageType = i32;
export const b3_stagePrepareJoints: b3SolverStageType = 0;
export const b3_stagePrepareWideContacts: b3SolverStageType = 1;
export const b3_stagePrepareContacts: b3SolverStageType = 2;
export const b3_stageIntegrateVelocities: b3SolverStageType = 3;
export const b3_stageWarmStart: b3SolverStageType = 4;
export const b3_stageSolve: b3SolverStageType = 5;
export const b3_stageIntegratePositions: b3SolverStageType = 6;
export const b3_stageRelax: b3SolverStageType = 7;
export const b3_stageRestitution: b3SolverStageType = 8;
export const b3_stageStoreWideImpulses: b3SolverStageType = 9;
export const b3_stageStoreImpulses: b3SolverStageType = 10;

export type b3SolverBlockType = i32;

// Block for iterating across bodies.
export const b3_bodyBlock: b3SolverBlockType = 0;

// Block for iterating across joints. For prepare.
export const b3_jointBlock: b3SolverBlockType = 1;

// Block for iterating across wide contacts. For prepare and store.
export const b3_wideContactBlock: b3SolverBlockType = 2;

// Block for iterating across contacts. For prepare and store.
export const b3_contactBlock: b3SolverBlockType = 3;

// Block for iterating across joints of a single graph color.
export const b3_graphJointBlock: b3SolverBlockType = 4;

// Block for iterating across wide contacts of a single graph color.
export const b3_graphWideContactBlock: b3SolverBlockType = 5;

// Block for iterating across contacts of a single graph color.
export const b3_graphContactBlock: b3SolverBlockType = 6;

// Block for processing overflow constraints
export const b3_overflowBlock: b3SolverBlockType = 7;

// Solver block describes a multithreaded unit of work.
export struct b3SolverBlock {
	startIndex: i32 = 0;
	count: u16 = 0;
	// b3SolverBlockType
	blockType: u8 = 0;
	colorIndex: u8 = 0;
}

// A unit of multithreaded work along with atomic synchronization. The syncIndex grows
// monotonically allowing the solver block to be re-used across sub-steps.
export struct b3SyncBlock {
	block: b3SolverBlock = b3SolverBlock();
	syncIndex: b3AtomicInt = b3AtomicInt();
}

const b3_emptySyncBlocks = new StaticArray<b3SyncBlock>(0);

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
// C slices one graph block buffer per color; here each color owns its own blocks array.
export class b3SolverStage {
	blocks: StaticArray<b3SyncBlock> = b3_emptySyncBlocks;
	type: b3SolverStageType = 0;
	blockCount: i32 = 0;
	colorIndex: u8 = 0;
	completionCount: b3AtomicInt = b3AtomicInt();
}

// Constraint softness
export struct b3Softness {
	biasRate: f32 = 0;
	massScale: f32 = 0;
	impulseScale: f32 = 0;

	constructor(biasRate: f32, massScale: f32, impulseScale: f32) {
		this.biasRate = biasRate;
		this.massScale = massScale;
		this.impulseScale = impulseScale;
	}
}

// Prepare/store run as a flat parallel-for over the whole wide-constraint
// range. Each span maps a slice of that range back to the owning color's
// contacts so workers can decode flat wide-slot indices without touching
// graph state. The spans array has one entry per active color plus a sentinel
// whose start == wideContactCount.
// contacts aliases the color's convexContacts array.
export class b3WidePrepareSpan {
	start: i32;
	count: i32;
	contacts: Array<i32>;

	constructor(start: i32, count: i32, contacts: Array<i32>) {
		this.start = start;
		this.count = count;
		this.contacts = contacts;
	}
}

// contacts aliases the color's contacts array.
export class b3ContactPrepareSpan {
	start: i32;
	count: i32;
	contacts: Array<b3ContactSpec>;

	constructor(start: i32, count: i32, contacts: Array<b3ContactSpec>) {
		this.start = start;
		this.count = count;
		this.contacts = contacts;
	}
}

// C's order pointer is the step's mesh lane order from orderBase, this color's first contact.
export class b3MeshPrepareSpan {
	start: i32;
	count: i32;
	contacts: Array<b3ContactSpec>;
	order: Int32Array;
	orderBase: i32 = 0;

	constructor(start: i32, count: i32, contacts: Array<b3ContactSpec>, order: Int32Array) {
		this.start = start;
		this.count = count;
		this.contacts = contacts;
		this.order = order;
	}
}

// joints aliases the color's jointSims array.
export class b3JointPrepareSpan {
	start: i32;
	count: i32;
	joints: Array<b3JointSim>;

	constructor(start: i32, count: i32, joints: Array<b3JointSim>) {
		this.start = start;
		this.count = count;
		this.joints = joints;
	}
}

const b3_emptyIndices = new StaticArray<i32>(0);
const b3_emptyWideConstraints = new StaticArray<b3ContactConstraintWide>(0);
const b3_emptyWideManifolds = new StaticArray<b3ContactManifoldWide>(0);
const b3_emptyWidePrepareSpans = new StaticArray<b3WidePrepareSpan>(0);
const b3_emptyContactPrepareSpans = new StaticArray<b3ContactPrepareSpan>(0);
const b3_emptyMeshPrepareSpans = new StaticArray<b3MeshPrepareSpan>(0);
const b3_emptyJointPrepareSpans = new StaticArray<b3JointPrepareSpan>(0);
const b3_emptyStages = new StaticArray<b3SolverStage>(0);
const b3_emptyManifoldConstraints = new StaticArray<b3ManifoldConstraint>(0);
const b3_emptyContactConstraints = new StaticArray<b3ContactConstraint>(0);
const b3_emptyStates = new Array<b3BodyState>();
const b3_emptySims = new Array<b3BodySim>();
const b3_emptyStarts = new Int32Array(0);

// Context for a time step. Recreated each time step.
export class b3StepContext {
	// time step
	dt: f32 = 0;

	// inverse time step (0 if dt == 0).
	inv_dt: f32 = 0;

	// sub-step
	h: f32 = 0;
	inv_h: f32 = 0;

	subStepCount: i32 = 0;

	contactSoftness: b3Softness = b3Softness();
	staticSoftness: b3Softness = b3Softness();

	maxLinearVelocity: f32 = 0;

	world: b3World;
	graph: b3ConstraintGraph;

	// shortcut to body states from awake set
	states: Array<b3BodyState> = b3_emptyStates;

	// shortcut to body sims from awake set
	sims: Array<b3BodySim> = b3_emptySims;

	// array of all shape ids for shapes that have enlarged AABBs
	enlargedShapes: StaticArray<i32> = b3_emptyIndices;
	enlargedShapeCount: i32 = 0;

	// Array of bullet bodies that need continuous collision handling
	bulletBodies: StaticArray<i32> = b3_emptyIndices;
	bulletBodyCount: b3AtomicInt = b3AtomicInt();

	anyRestitution: b3AtomicInt = b3AtomicInt();

	// Contact ids for simplified parallel-for access. Used in narrow-phase.
	// These contacts may or may not be touching. They are associated with awake bodies.
	awakeContactIndices: StaticArray<i32> = b3_emptyIndices;

	// Flat view of the wide contact constraint array used by prepare and store.
	// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
	// at wideContactCount. C interleaves each constraint with its manifold; here
	// wideManifolds[i] belongs to wideConstraints[i].
	// Per-color slices live at colors[i].wideConstraints from wideConstraintStart.
	wideConstraints: StaticArray<b3ContactConstraintWide> = b3_emptyWideConstraints;
	wideManifolds: StaticArray<b3ContactManifoldWide> = b3_emptyWideManifolds;
	widePrepareSpans: StaticArray<b3WidePrepareSpan> = b3_emptyWidePrepareSpans;
	wideContactCount: i32 = 0;

	// Similar for overflow contact constraints.
	overflowSpans: StaticArray<b3ContactPrepareSpan> = b3_emptyContactPrepareSpans;

	// Mesh group g owns wideMeshManifolds[wideMeshManifoldStarts[g] .. wideMeshManifoldStarts[g + 1]).
	wideMeshConstraints: StaticArray<b3ContactConstraintWide> = b3_emptyWideConstraints;
	wideMeshManifolds: StaticArray<b3ContactManifoldWide> = b3_emptyWideManifolds;
	wideMeshManifoldStarts: Int32Array = b3_emptyStarts;
	meshPrepareSpans: StaticArray<b3MeshPrepareSpan> = b3_emptyMeshPrepareSpans;
	jointPrepareSpans: StaticArray<b3JointPrepareSpan> = b3_emptyJointPrepareSpans;

	activeColorCount: i32 = 0;
	workerCount: i32 = 0;

	stages: StaticArray<b3SolverStage> = b3_emptyStages;
	stageCount: i32 = 0;
	enableWarmStarting: bool = false;

	// This atomic is central to multi-threaded solver task synchronization.
	// It prevents ABA problems by monotonically growing as the solver advances.
	// This means a delayed worker thread will catch up without repeating already completed
	// work (causing a race condition).
	// sync index (16-bits) | stage type (16-bits)
	atomicSyncBits: b3AtomicU32 = b3AtomicU32();

	// Race flag claimed by whichever runner reaches b3SolverTask with workerIndex 0 first.
	mainClaimed: b3AtomicInt = b3AtomicInt();

	constructor(world: b3World) {
		this.world = world;
		this.graph = world.constraintGraph;
	}
}

export function b3MakeSoft(hertz: f32, zeta: f32, h: f32): b3Softness {
	if (hertz == 0.0) {
		return b3Softness(0.0, 0.0, 0.0);
	}

	let omega: f32 = 2.0 * B3_PI * hertz;
	let a1: f32 = 2.0 * zeta + h * omega;
	let a2: f32 = h * omega * a1;
	let a3: f32 = 1.0 / (1.0 + a2);

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))

	// If z == 0
	// bias = 1/h
	// massScale = hw^2 / (1 + hw^2)
	// impulseScale = 1 / (1 + hw^2)

	// w -> inf
	// bias = 1/h
	// massScale = 1
	// impulseScale = 0

	// if w = pi / 4  * inv_h
	// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
	// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

	// In all cases:
	// massScale + impulseScale == 1

	return b3Softness(omega / a1, a2 * a3, a3);
}

// functions from solver.c follow

// these are useful for solver testing
const ITERATIONS: i32 = 1;
const RELAX_ITERATIONS: i32 = 1;

// Single threaded stand-ins for the C atomics, kept so the synchronization code reads as in C.
@inline function b3AtomicLoadInt(a: b3AtomicInt): i32 {
	return a.value;
}

@inline function b3AtomicStoreInt(a: ref b3AtomicInt, value: i32): void {
	a.value = value;
}

@inline function b3AtomicFetchAddInt(a: ref b3AtomicInt, increment: i32): i32 {
	let value = a.value;
	a.value = value + increment;
	return value;
}

@inline function b3AtomicCompareExchangeInt(a: ref b3AtomicInt, expected: i32, desired: i32): bool {
	if (a.value == expected) {
		a.value = desired;
		return true;
	}
	return false;
}

@inline function b3AtomicLoadU32(a: b3AtomicU32): u32 {
	return a.value;
}

@inline function b3AtomicStoreU32(a: ref b3AtomicU32, value: u32): void {
	a.value = value;
}

function b3Pause(): void {}

function b3Yield(): void {}

class b3WorkerContext {
	context: b3StepContext;
	workerIndex: i32;
	// Handle from enqueueTaskFcn, 0 for C's NULL
	userTask: u64 = 0;

	constructor(context: b3StepContext, workerIndex: i32) {
		this.context = context;
		this.workerIndex = workerIndex;
	}
}

// The integrations and finalize walk the state, sim, body and move event arrays through pointers as C does, with
// these field offsets. Their indices stay below the awake counts and the move event count b3Solve sized, and
// bodyId is a live body.
const FS_V = offsetof<b3BodyState>("linearVelocity");
const FS_W = offsetof<b3BodyState>("angularVelocity");
const FS_DP = offsetof<b3BodyState>("deltaPosition");
const FS_FLAGS = offsetof<b3BodyState>("flags");
const FS_DQ = offsetof<b3BodyState>("deltaRotation");
const FSIM_P = offsetof<b3BodySim>("transform") + offsetof<b3Transform>("p");
const FSIM_Q = offsetof<b3BodySim>("transform") + offsetof<b3Transform>("q");
const FSIM_TRANSFORM = offsetof<b3BodySim>("transform");
const FSIM_CENTER = offsetof<b3BodySim>("center");
const FSIM_FLAGS = offsetof<b3BodySim>("flags");
const FSIM_MINEXTENT = offsetof<b3BodySim>("minExtent");
const FSIM_MAXEXTENT = offsetof<b3BodySim>("maxExtent");
const FSIM_ROTATION0 = offsetof<b3BodySim>("rotation0");
const FSIM_CENTER0 = offsetof<b3BodySim>("center0");
const FSIM_LOCALCENTER = offsetof<b3BodySim>("localCenter");
const FSIM_FORCE = offsetof<b3BodySim>("force");
const FSIM_TORQUE = offsetof<b3BodySim>("torque");
const FSIM_INVINERTIALOCAL = offsetof<b3BodySim>("invInertiaLocal");
const FSIM_INVINERTIAWORLD = offsetof<b3BodySim>("invInertiaWorld");
const FSIM_INVMASS = offsetof<b3BodySim>("invMass");
const FSIM_INERTIA = offsetof<b3BodySim>("inertia");
const FSIM_LINEARDAMPING = offsetof<b3BodySim>("linearDamping");
const FSIM_ANGULARDAMPING = offsetof<b3BodySim>("angularDamping");
const FSIM_GRAVITYSCALE = offsetof<b3BodySim>("gravityScale");
const FSIM_BODYID = offsetof<b3BodySim>("bodyId");
const ME_USERDATA = offsetof<b3BodyMoveEvent>("userData");
const ME_TRANSFORM = offsetof<b3BodyMoveEvent>("transform");
const ME_BODYID = offsetof<b3BodyMoveEvent>("bodyId");
const ME_FELLASLEEP = offsetof<b3BodyMoveEvent>("fellAsleep");
const FB_USERDATA = offsetof<b3Body>("userData");
const FB_HEADSHAPEID = offsetof<b3Body>("headShapeId");
const FB_ISLANDID = offsetof<b3Body>("islandId");
const FB_SLEEPTHRESHOLD = offsetof<b3Body>("sleepThreshold");
const FB_SLEEPTIME = offsetof<b3Body>("sleepTime");
const FB_SLEEPVELOCITY = offsetof<b3Body>("sleepVelocity");
const FB_SAFETYFACTOR = offsetof<b3Body>("safetyFactor");
const FB_BODYMOVEINDEX = offsetof<b3Body>("bodyMoveIndex");
const FB_FLAGS = offsetof<b3Body>("flags");
const FB_TYPE = offsetof<b3Body>("type");
const FB_GENERATION = offsetof<b3Body>("generation");

// Integrate velocities, apply damping, and gyroscopic torque
// Through state and sim pointers as C; i is below the awake body count the body blocks cover.
unsafe function b3IntegrateVelocitiesTask(block: b3SolverBlock, context: b3StepContext): void {
	let states = b3ArrayData(context.states);
	let sims = b3ArrayData(context.sims);

	let gravity = context.world.gravity;
	let h = context.h;

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		let sim = sims + <usize>i * sizeof<b3BodySim>();
		let state = states + <usize>i * sizeof<b3BodyState>();

		let v = load<b3Vec3>(state + FS_V);
		let w = load<b3Vec3>(state + FS_W);
		let invMass = load<f32>(sim + FSIM_INVMASS);

		// Damping math
		// Differential equation: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		let linearDamping: f32 = 1.0 / (1.0 + h * load<f32>(sim + FSIM_LINEARDAMPING));
		let angularDamping: f32 = 1.0 / (1.0 + h * load<f32>(sim + FSIM_ANGULARDAMPING));

		// Gravity scale will be zero for kinematic bodies
		let gravityScale: f32 = invMass > 0.0 ? load<f32>(sim + FSIM_GRAVITYSCALE) : <f32>0.0;

		let linearVelocityDelta = b3Blend2(h * invMass, load<b3Vec3>(sim + FSIM_FORCE), h * gravityScale, gravity);
		v = b3MulAdd(linearVelocityDelta, linearDamping, v);

		let angularVelocityDelta = b3MulSV(h, b3MulMV(load<b3Matrix3>(sim + FSIM_INVINERTIAWORLD), load<b3Vec3>(sim + FSIM_TORQUE)));
		w = b3MulAdd(angularVelocityDelta, angularDamping, w);

		// Gyroscopic torque by solving this nonlinear equation using Newton-Raphson.
		// I * (w2 - w1) + h * cross(w2, I * w2) = 0
		// This is all done in local coordinates where the Jacobian is easier to compute.
		// This improves the simulation of long skinny bodies.
		// If the principal inertia values of I are roughly equal (isotropic) then
		// cross(w2, I * w2) ~= 0, so this can be skipped.
		if ((load<u32>(sim + FSIM_FLAGS) & b3_enableGyroscopicTorque) != 0) {
			// Get current rotation.
			let q0 = load<b3Quat>(sim + FSIM_Q);
			let q = b3MulQuat(load<b3Quat>(state + FS_DQ), q0);

			// Compute local angular velocity
			let omega1 = b3InvRotateVector(q, w);
			let omega2 = omega1;

			// Symmetric inertia tensor: 6 unique entries (column-major)
			let i00 = load<f32>(sim + FSIM_INERTIA + 0);
			let i01 = load<f32>(sim + FSIM_INERTIA + 12);
			let i02 = load<f32>(sim + FSIM_INERTIA + 24);
			let i11 = load<f32>(sim + FSIM_INERTIA + 16);
			let i12 = load<f32>(sim + FSIM_INERTIA + 28);
			let i22 = load<f32>(sim + FSIM_INERTIA + 32);

			for (let gyroIteration = 0; gyroIteration < B3_GYROSCOPIC_ITERATIONS; ++gyroIteration) {
				let w1 = omega2.x;
				let w2 = omega2.y;
				let w3 = omega2.z;

				// Iw = I * omega2 (shared between residual and Jacobian)
				let Iw1 = i00 * w1 + i01 * w2 + i02 * w3;
				let Iw2 = i01 * w1 + i11 * w2 + i12 * w3;
				let Iw3 = i02 * w1 + i12 * w2 + i22 * w3;

				// Residual: b = I * (omega2 - omega1) + h * cross(omega2, I * omega2)
				let dw = b3Sub(omega2, omega1);
				let b = b3Vec3(
					i00 * dw.x + i01 * dw.y + i02 * dw.z + h * (w2 * Iw3 - w3 * Iw2),
					i01 * dw.x + i11 * dw.y + i12 * dw.z + h * (w3 * Iw1 - w1 * Iw3),
					i02 * dw.x + i12 * dw.y + i22 * dw.z + h * (w1 * Iw2 - w2 * Iw1)
				);

				// Jacobian J = I + h * (skew(omega2) * I - skew(I * omega2))
				// Jacobian derived by Erin Catto, Ph.D. Do not attempt to do this without a Ph.D.
				// Doubled inertia terms above fold into Iw, e.g. row 2 col 1: i00*w3 - i02*w1 - Iw3.
				let J = b3Matrix3(
					b3Vec3(
						i00 + h * (w2 * i02 - w3 * i01),
						i01 + h * (w3 * i00 - w1 * i02 - Iw3),
						i02 + h * (w1 * i01 - w2 * i00 + Iw2)
					),
					b3Vec3(
						i01 + h * (w2 * i12 - w3 * i11 + Iw3),
						i11 + h * (w3 * i01 - w1 * i12),
						i12 + h * (w1 * i11 - w2 * i01 - Iw1)
					),
					b3Vec3(
						i02 + h * (w2 * i22 - w3 * i12 - Iw2),
						i12 + h * (w3 * i02 - w1 * i22 + Iw1),
						i22 + h * (w1 * i12 - w2 * i02)
					)
				);

				omega2 = b3Sub(omega2, b3Solve3(J, b));
			}

			w = b3RotateVector(q, omega2);
		}

		// The padding after each velocity holds zero
		b3StoreRow(state + FS_V, v.x, v.y, v.z, 0.0);
		b3StoreRow(state + FS_W, w.x, w.y, w.z, 0.0);
	}
}

unsafe function b3IntegratePositionsTask(block: b3SolverBlock, context: b3StepContext): void {
	let states = b3ArrayData(context.states);
	let h = context.h;
	let maxLinearSpeed = context.maxLinearVelocity;
	let maxAngularSpeed = B3_MAX_ROTATION * context.inv_dt;
	let maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	let maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		let state = load<b3BodyState>(states + <usize>i * sizeof<b3BodyState>());

		let v = state.linearVelocity;
		let w = state.angularVelocity;

		// Motion locks - these can be viewed as a constraint that come last
		v.x = (state.flags & b3_lockLinearX) != 0 ? <f32>0.0 : v.x;
		v.y = (state.flags & b3_lockLinearY) != 0 ? <f32>0.0 : v.y;
		v.z = (state.flags & b3_lockLinearZ) != 0 ? <f32>0.0 : v.z;
		w.x = (state.flags & b3_lockAngularX) != 0 ? <f32>0.0 : w.x;
		w.y = (state.flags & b3_lockAngularY) != 0 ? <f32>0.0 : w.y;
		w.z = (state.flags & b3_lockAngularZ) != 0 ? <f32>0.0 : w.z;

		// Clamp to max linear speed
		if (b3Dot(v, v) > maxLinearSpeedSquared) {
			let ratio = maxLinearSpeed / b3Length(v);
			v = b3MulSV(ratio, v);
			state.flags |= b3_isSpeedCapped;
		}

		// Clamp to max angular speed
		if (b3Dot(w, w) > maxAngularSpeedSquared && (state.flags & b3_allowFastRotation) == 0) {
			let ratio = maxAngularSpeed / b3Length(w);
			w = b3MulSV(ratio, w);
			state.flags |= b3_isSpeedCapped;
		}

		let dp = b3MulAdd(state.deltaPosition, h, v);
		let dq = b3IntegrateRotation(state.deltaRotation, b3MulSV(h, w));
		let p = states + <usize>i * sizeof<b3BodyState>();
		b3StoreRow(p + FS_V, v.x, v.y, v.z, state.padding1);
		b3StoreRow(p + FS_W, w.x, w.y, w.z, state.padding2);
		b3StoreRow(p + FS_DP, dp.x, dp.y, dp.z, reinterpret<f32>(state.flags));
		b3StoreRow(p + FS_DQ, dq.v.x, dq.v.y, dq.v.z, dq.s);
	}
}

function b3PrepareJointsTask(block: b3SolverBlock, context: b3StepContext): void {
	let spans = context.jointPrepareSpans;

	let index = block.startIndex;
	let endIndex = block.startIndex + <i32>block.count;

	// Find color for start index. Linear search but fast.
	let colorIndex = 0;
	while (spans[colorIndex + 1].start <= index) {
		colorIndex += 1;
	}

	// Loop over block
	while (index < endIndex) {
		let colorStart = spans[colorIndex].start;
		let colorEndIndex = b3MinInt(spans[colorIndex + 1].start, endIndex);
		let joints = spans[colorIndex].joints;

		// Loop over color
		for (; index < colorEndIndex; ++index) {
			B3_ASSERT(0 <= index - colorStart && index - colorStart < spans[colorIndex].count);
			let joint = joints[index - colorStart];
			b3PrepareJoint(joint, context);
		}

		// Advance to next color
		colorIndex += 1;
	}
}

function b3WarmStartJointsTask(block: b3SolverBlock, context: b3StepContext): void {
	let color = context.graph.colors[block.colorIndex];
	let joints = color.jointSims;

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		let joint = joints[i];
		b3WarmStartJoint(joint, context);
	}
}

function b3SolveJointsTask(block: b3SolverBlock, context: b3StepContext, useBias: bool, workerIndex: i32): void {
	let color = context.graph.colors[block.colorIndex];
	let joints = color.jointSims;

	B3_ASSERT(0 <= block.startIndex && block.startIndex + <i32>block.count <= color.jointSims.length);

	let jointStateBitSet = context.world.taskContexts[workerIndex].jointStateBitSet;

	let endIndex = block.startIndex + <i32>block.count;
	for (let i = block.startIndex; i < endIndex; ++i) {
		let joint = joints[i];
		b3SolveJoint(joint, context, useBias);

		if (
			useBias &&
			(joint.forceThreshold < FLT_MAX || joint.torqueThreshold < FLT_MAX) &&
			b3GetBit(jointStateBitSet, <u32>joint.jointId) == false
		) {
			// force, torque
			let reaction: FixedArray<f32, 2>;
			b3GetJointReaction(context.world, joint, context.inv_h, reaction[0], reaction[1]);

			// Check thresholds. A zero threshold means all awake joints get reported.
			if (reaction[0] >= joint.forceThreshold || reaction[1] >= joint.torqueThreshold) {
				// Flag this joint for processing.
				b3SetBit(jointStateBitSet, <u32>joint.jointId);
			}
		}
	}
}

const B2_MAX_CONTINUOUS_SENSOR_HITS: i32 = 8;

const b3_noShape = new b3Shape();

class b3ContinuousContext {
	world: b3World? = null;
	// The fast body sim's flags and body id, all the query reads of it; neither changes during the sweep.
	fastBodySimFlags: u32 = 0;
	fastBodyId: i32 = 0;
	fastShape: b3Shape = b3_noShape;
	centroid1: b3Vec3 = b3Vec3();
	centroid2: b3Vec3 = b3Vec3();
	sweep: b3Sweep = b3Sweep();
	// World base for re-centering sweeps. Keeps TOI in float precision far from the origin.
	base: b3Pos = b3Vec3();
	fraction: f32 = 0;
	// B2_MAX_CONTINUOUS_SENSOR_HITS
	sensorHits: FixedArray<b3SensorHit, 8>;
	sensorFractions: FixedArray<f32, 8>;
	sensorCount: i32 = 0;

	distanceIterations: i32 = 0;
	pushBackIterations: i32 = 0;
	rootIterations: i32 = 0;

}

function b3MakeShapeId(index1: i32, world0: u16, generation: u16): b3ShapeId {
	let id = b3ShapeId();
	id.index1 = index1;
	id.world0 = world0;
	id.generation = generation;
	return id;
}

// This is called from b3DynamicTree_Query for continuous collision
function b3ContinuousQueryCallback(proxyId: i32, userData: u64, continuousContext: b3ContinuousContext): bool {
	let shapeId = <i32>userData;

	let fastShape = continuousContext.fastShape;
	let fastBodySimFlags = continuousContext.fastBodySimFlags;
	let fastBodyId = continuousContext.fastBodyId;

	B3_ASSERT(fastShape.sensorIndex == B3_NULL_INDEX);

	// Skip same shape
	if (shapeId == fastShape.id) {
		return true;
	}

	let world = continuousContext.world!;
	let shape = world.shapes[shapeId];

	// Skip same body
	if (shape.bodyId == fastShape.bodyId) {
		return true;
	}

	// Skip sensors unless both shapes want sensor events
	let isSensor = shape.sensorIndex != B3_NULL_INDEX;
	if (isSensor && ((shape.flags & b3_enableSensorEvents) == 0 || (fastShape.flags & b3_enableSensorEvents) == 0)) {
		return true;
	}

	// Skip filtered shapes
	let canCollide = b3ShouldShapesCollide(fastShape.filter, shape.filter);
	if (canCollide == false) {
		return true;
	}

	let bodyId = shape.bodyId;

	let bodySim = b3GetBodySimAddress(world, bodyId);
	B3_ASSERT(world.bodies[bodyId].type == b3_staticBody || (fastBodySimFlags & b3_isBullet) != 0);

	// Skip bullets
	if ((world.bodies[bodyId].flags & b3_isBullet) != 0) {
		return true;
	}

	// Skip filtered bodies
	canCollide = b3ShouldBodiesCollide(world, fastBodyId, bodyId);
	if (canCollide == false) {
		return true;
	}

	// Custom user filtering
	if ((shape.flags & b3_enableCustomFiltering) != 0 || (fastShape.flags & b3_enableCustomFiltering) != 0) {
		let customFilterFcn = world.customFilterFcn;
		if (customFilterFcn != null) {
			let idA = b3MakeShapeId(shape.id + 1, world.worldId, shape.generation);
			let idB = b3MakeShapeId(fastShape.id + 1, world.worldId, fastShape.generation);
			canCollide = customFilterFcn(idA, idB);
			if (canCollide == false) {
				return true;
			}
		}
	}

	// todo does having a sweep on shapeA help with bullets?
	let sweepA = b3MakeRelativeSweepAt(bodySim, continuousContext.base);

	// Time of impact versus shape. Supports all shape types
	let output = b3ShapeTimeOfImpact(shape, fastShape, sweepA, continuousContext.sweep, continuousContext.fraction);
	if (isSensor) {
		// Only accept a sensor hit that is sooner than the current solid hit.
		if (output.fraction <= continuousContext.fraction && continuousContext.sensorCount < B2_MAX_CONTINUOUS_SENSOR_HITS) {
			let index = continuousContext.sensorCount;

			// The hit shape is a sensor
			let sensorHit = b3SensorHit(shape.id, fastShape.id);

			continuousContext.sensorHits[index] = sensorHit;
			continuousContext.sensorFractions[index] = output.fraction;
			continuousContext.sensorCount += 1;
		}
	} else if (0.0 < output.fraction && output.fraction < continuousContext.fraction) {
		let didHit = true;

		if (didHit && ((shape.flags & b3_enablePreSolveEvents) != 0 || (fastShape.flags & b3_enablePreSolveEvents) != 0)) {
			let shapeIdA = b3MakeShapeId(shape.id + 1, world.worldId, shape.generation);
			let shapeIdB = b3MakeShapeId(fastShape.id + 1, world.worldId, fastShape.generation);
			let point = b3OffsetPos(continuousContext.base, output.point);
			didHit = world.preSolveFcn!(shapeIdA, shapeIdB, point, output.normal);
		}

		if (didHit) {
			continuousContext.fraction = output.fraction;
			continuousContext.distanceIterations = b3MaxInt(continuousContext.distanceIterations, output.distanceIterations);
			continuousContext.pushBackIterations = b3MaxInt(continuousContext.pushBackIterations, output.pushBackIterations);
			continuousContext.rootIterations = b3MaxInt(continuousContext.rootIterations, output.rootIterations);
		}
	}

	// Continue query
	return true;
}

const s_continuousContext = new b3ContinuousContext();

// The tree callback of b3SolveContinuous, which does not nest
function b3ContinuousTreeCallback(proxyId: i32, userData: u64): bool {
	return b3ContinuousQueryCallback(proxyId, userData, s_continuousContext);
}

function b3IsShapeFast(shape: b3Shape, centroid1: b3Vec3, centroid2: b3Vec3, rotationChord: f32, safetyFactor: f32): bool {
	let radius: f32;
	switch (shape.type) {
		case b3_sphereShape:
			radius = shape.sphere.radius;
			break;

		case b3_capsuleShape:
			radius = 0.5 * b3Distance(shape.capsule.center1, shape.capsule.center2) + shape.capsule.radius;
			break;

		case b3_hullShape: {
			let farthestPoint = b3FarthestPointOnAABB(shape.hull!.aabb, shape.localCentroid);
			radius = b3Distance(farthestPoint, shape.localCentroid);
			break;
		}

		default:
			return true;
	}

	let minExtent = b3ComputeShapeMinExtent(shape, shape.localCentroid);
	let maxMotion = b3Distance(centroid1, centroid2) + rotationChord * radius;
	return maxMotion > safetyFactor * minExtent;
}

// Continuous collision of dynamic versus static
unsafe function b3SolveContinuous(world: b3World, bodySimIndex: i32, taskContext: b3TaskContext, dt: f32): void {
	let awakeSet = world.solverSets[b3_awakeSet];
	let fastBodySim = b3ArrayData(awakeSet.bodySims) + <usize>bodySimIndex * sizeof<b3BodySim>();
	B3_ASSERT((load<u32>(fastBodySim + FSIM_FLAGS) & b3_isFast) != 0);

	// Re-center the sweep on the fast body so the TOI and the swept query stay in float precision
	let base = load<b3Vec3>(fastBodySim + FSIM_CENTER0);

	let sweep = b3MakeRelativeSweepAt(fastBodySim, base);

	let xf1 = b3Transform();
	xf1.q = sweep.q1;
	xf1.p = b3Sub(sweep.c1, b3RotateVector(sweep.q1, sweep.localCenter));

	let xf2 = b3Transform();
	xf2.q = sweep.q2;
	xf2.p = b3Sub(sweep.c2, b3RotateVector(sweep.q2, sweep.localCenter));

	let staticTree = world.broadPhase.trees[b3_staticBody];
	let kinematicTree = world.broadPhase.trees[b3_kinematicBody];
	let dynamicTree = world.broadPhase.trees[b3_dynamicBody];
	let fastBodyId = load<i32>(fastBodySim + FSIM_BODYID);
	let fastBodySimFlags = load<u32>(fastBodySim + FSIM_FLAGS);

	// C's zeroed stack context; the fields not reset here are written before they are read
	let context = s_continuousContext;
	context.world = world;
	context.sweep = sweep;
	context.base = base;
	context.fastBodySimFlags = fastBodySimFlags;
	context.fastBodyId = fastBodyId;
	context.fraction = 1.0;
	context.sensorCount = 0;
	context.distanceIterations = 0;
	context.pushBackIterations = 0;
	context.rootIterations = 0;

	let callback = b3ContinuousTreeCallback;

	let isBullet = (fastBodySimFlags & b3_isBullet) != 0;
	let safetyFactor = world.bodies[fastBodyId].safetyFactor;
	let rotationChord: f32 = 2.0 * b3Length(b3InvMulQuat(sweep.q1, sweep.q2).v);

	let shapeId = world.bodies[fastBodyId].headShapeId;
	while (shapeId != B3_NULL_INDEX) {
		let fastShape = world.shapes[shapeId];
		shapeId = fastShape.nextShapeId;

		context.fastShape = fastShape;
		context.centroid1 = b3TransformPoint(xf1, fastShape.localCentroid);
		context.centroid2 = b3TransformPoint(xf2, fastShape.localCentroid);

		let box1 = fastShape.aabb;
		// xf2 is relative to the base, so translate the box back to world space, rounding outward
		let box2 = b3OffsetAABB(b3ComputeShapeAABB(fastShape, xf2), base);

		// Store this to avoid double computation in the case there is no impact event
		fastShape.aabb = box2;

		// No continuous collision for meshes
		if (fastShape.type == b3_meshShape || fastShape.type == b3_heightShape) {
			continue;
		}

		// No continuous collision for sensors
		if (fastShape.sensorIndex != B3_NULL_INDEX) {
			continue;
		}

		if (isBullet == false) {
			let isShapeFast = b3IsShapeFast(fastShape, context.centroid1, context.centroid2, rotationChord, safetyFactor);
			if (isShapeFast == false) {
				continue;
			}
		}

		let sweptBox = b3AABB_Union(box1, box2);
		b3DynamicTree_Query(staticTree, sweptBox, B3_DEFAULT_MASK_BITS, false, callback);

		if (isBullet) {
			b3DynamicTree_Query(kinematicTree, sweptBox, B3_DEFAULT_MASK_BITS, false, callback);
			b3DynamicTree_Query(dynamicTree, sweptBox, B3_DEFAULT_MASK_BITS, false, callback);
		}
	}

	let speculativeScalar = B3_SPECULATIVE_DISTANCE();

	if (context.fraction < 1.0) {
		// Handle time of impact event. The sweep is relative to the base, so re-add the base
		// to return the advanced pose to world space.
		let q = b3NLerp(sweep.q1, sweep.q2, context.fraction);
		let c = b3Lerp(sweep.c1, sweep.c2, context.fraction);
		let origin = b3Sub(c, b3RotateVector(q, sweep.localCenter));

		// Advance body
		let transform = b3Transform(b3OffsetPos(base, origin), q);
		let center = b3OffsetPos(base, c);
		store<b3Transform>(fastBodySim + FSIM_TRANSFORM, transform);
		store<b3Vec3>(fastBodySim + FSIM_CENTER, center);
		store<b3Quat>(fastBodySim + FSIM_ROTATION0, q);
		store<b3Vec3>(fastBodySim + FSIM_CENTER0, center);
		store<u32>(fastBodySim + FSIM_FLAGS, load<u32>(fastBodySim + FSIM_FLAGS) | b3_hadTimeOfImpact);

		// Timeloss means there is a lost gravity contribution. Other forces and torques are ignored for now.
		let stateFlags = awakeSet.bodyStates[bodySimIndex].flags;
		let v = awakeSet.bodyStates[bodySimIndex].linearVelocity;
		let timeLoss = (1.0 - context.fraction) * dt;
		let dv = b3MulSV(-timeLoss * load<f32>(fastBodySim + FSIM_GRAVITYSCALE), world.gravity);
		dv.x = (stateFlags & b3_lockLinearX) != 0 ? <f32>0.0 : dv.x;
		dv.y = (stateFlags & b3_lockLinearY) != 0 ? <f32>0.0 : dv.y;
		dv.z = (stateFlags & b3_lockLinearZ) != 0 ? <f32>0.0 : dv.z;
		awakeSet.bodyStates[bodySimIndex].linearVelocity = b3Add(v, dv);

		// The move event was written before CCD, so correct it with the impact pose
		world.bodyMoveEvents[bodySimIndex].transform = transform;

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

		shapeId = world.bodies[fastBodyId].headShapeId;
		while (shapeId != B3_NULL_INDEX) {
			let shape = world.shapes[shapeId];

			// Must recompute aabb at the interpolated transform
			let aabb = b3ComputeFatShapeAABB(shape, transform, speculativeScalar);
			shape.aabb = aabb;

			if (b3AABB_Contains(world.fatAABBs[shapeId], aabb) == false) {
				world.fatAABBs[shapeId] = b3AABB_Inflate(aabb, shape.aabbMargin);

				// Regular bodies mark the hierarchy as moved using atomic operations.
				// Bullets are handled separately at a later stage.
				if (isBullet == false) {
					b3BroadPhase_MarkProxyMoved(world.broadPhase, shape.proxyKey, world.fatAABBs[shapeId]);
				} else {
					store<u32>(fastBodySim + FSIM_FLAGS, load<u32>(fastBodySim + FSIM_FLAGS) | b3_enlargeBulletBounds);
				}
			}

			shapeId = shape.nextShapeId;
		}
	} else {
		// No time of impact event

		// Advance body
		store<b3Quat>(fastBodySim + FSIM_ROTATION0, load<b3Quat>(fastBodySim + FSIM_Q));
		store<b3Vec3>(fastBodySim + FSIM_CENTER0, load<b3Vec3>(fastBodySim + FSIM_CENTER));

		// Prepare AABBs for broad-phase
		shapeId = world.bodies[fastBodyId].headShapeId;
		while (shapeId != B3_NULL_INDEX) {
			let shape = world.shapes[shapeId];

			// shape->aabb is still valid from above

			if (b3AABB_Contains(world.fatAABBs[shapeId], shape.aabb) == false) {
				world.fatAABBs[shapeId] = b3AABB_Inflate(shape.aabb, shape.aabbMargin);

				if (isBullet == false) {
					b3BroadPhase_MarkProxyMoved(world.broadPhase, shape.proxyKey, world.fatAABBs[shapeId]);
				} else {
					store<u32>(fastBodySim + FSIM_FLAGS, load<u32>(fastBodySim + FSIM_FLAGS) | b3_enlargeBulletBounds);
				}
			}

			shapeId = shape.nextShapeId;
		}
	}

	// Push sensor hits on the the task context for serial processing.
	for (let i = 0; i < context.sensorCount; ++i) {
		// Skip any sensor hits that occurred after a solid hit
		if (context.sensorFractions[i] < context.fraction) {
			taskContext.sensorHits.push(context.sensorHits[i]);
		}
	}

	taskContext.distanceIterations = b3MaxInt(taskContext.distanceIterations, context.distanceIterations);
	taskContext.pushBackIterations = b3MaxInt(taskContext.pushBackIterations, context.pushBackIterations);
	taskContext.rootIterations = b3MaxInt(taskContext.rootIterations, context.rootIterations);
}

function b3MakeBodyMoveId(index1: i32, world0: u16, generation: u16): b3BodyId {
	let id = b3BodyId();
	id.index1 = index1;
	id.world0 = world0;
	id.generation = generation;
	return id;
}

// Implements b3ParallelForCallback
// b3MulMM( b3MulMM( r, i ), b3Transpose( r ) ) with each b3MulMV sum in C's order, stored into dst
@inline unsafe function b3StoreWorldInertia(dst: usize, r: b3Matrix3, i: b3Matrix3): void {
	let axx = r.cx.x * i.cx.x + r.cy.x * i.cx.y + r.cz.x * i.cx.z;
	let axy = r.cx.y * i.cx.x + r.cy.y * i.cx.y + r.cz.y * i.cx.z;
	let axz = r.cx.z * i.cx.x + r.cy.z * i.cx.y + r.cz.z * i.cx.z;
	let ayx = r.cx.x * i.cy.x + r.cy.x * i.cy.y + r.cz.x * i.cy.z;
	let ayy = r.cx.y * i.cy.x + r.cy.y * i.cy.y + r.cz.y * i.cy.z;
	let ayz = r.cx.z * i.cy.x + r.cy.z * i.cy.y + r.cz.z * i.cy.z;
	let azx = r.cx.x * i.cz.x + r.cy.x * i.cz.y + r.cz.x * i.cz.z;
	let azy = r.cx.y * i.cz.x + r.cy.y * i.cz.y + r.cz.y * i.cz.z;
	let azz = r.cx.z * i.cz.x + r.cy.z * i.cz.y + r.cz.z * i.cz.z;
	b3StoreRow(
		dst,
		axx * r.cx.x + ayx * r.cy.x + azx * r.cz.x,
		axy * r.cx.x + ayy * r.cy.x + azy * r.cz.x,
		axz * r.cx.x + ayz * r.cy.x + azz * r.cz.x,
		axx * r.cx.y + ayx * r.cy.y + azx * r.cz.y
	);
	b3StoreRow(
		dst + 16,
		axy * r.cx.y + ayy * r.cy.y + azy * r.cz.y,
		axz * r.cx.y + ayz * r.cy.y + azz * r.cz.y,
		axx * r.cx.z + ayx * r.cy.z + azx * r.cz.z,
		axy * r.cx.z + ayy * r.cy.z + azy * r.cz.z
	);
	store<f32>(dst + 32, axz * r.cx.z + ayz * r.cy.z + azz * r.cz.z);
}

unsafe function b3FinalizeBodiesTask(startIndex: i32, endIndex: i32, workerIndex: i32, stepContext: b3StepContext): void {
	let world = stepContext.world;
	let bodies = b3ArrayData(world.bodies);
	let sims = b3ArrayData(stepContext.sims);
	let states = b3ArrayData(stepContext.states);

	B3_ASSERT(endIndex <= world.bodyMoveEvents.length);

	let enableSleep = world.enableSleep;
	let enableContinuous = world.enableContinuous;
	let timeStep = stepContext.dt;
	let invTimeStep = stepContext.inv_dt;
	let worldId = world.worldId;

	// The body move event array has should already have the correct size
	let moveEvents = b3ArrayData(world.bodyMoveEvents);

	let taskContext = world.taskContexts[workerIndex];
	let awakeIslandBitSet = taskContext.awakeIslandBitSet;

	let speculativeScalar = B3_SPECULATIVE_DISTANCE();

	for (let simIndex = startIndex; simIndex < endIndex; ++simIndex) {
		let state = states + <usize>simIndex * sizeof<b3BodyState>();
		let sim = sims + <usize>simIndex * sizeof<b3BodySim>();

		let v = load<b3Vec3>(state + FS_V);
		let w = load<b3Vec3>(state + FS_W);
		let deltaPosition = load<b3Vec3>(state + FS_DP);
		let deltaRotation = load<b3Quat>(state + FS_DQ);
		let q = load<b3Quat>(sim + FSIM_Q);
		let localOmega = b3InvRotateVector(q, w);
		let localDeltaRotation = b3InvRotateVector(q, deltaRotation.v);
		let bodyId = load<i32>(sim + FSIM_BODYID);

		if (b3IsValidVec3(v) == false || b3IsValidVec3(w) == false) {
			let name = b3FindNameWithDefault(world.names, world.bodies[bodyId].nameId, "NULL");
			b3Log("unstable: " + name);
		}

		B3_ASSERT(b3IsValidVec3(v));
		B3_ASSERT(b3IsValidVec3(w));

		// The rows below write whole 16 byte runs; each lane past the field it means rewrites the next field's value
		let center = b3OffsetPos(load<b3Vec3>(sim + FSIM_CENTER), deltaPosition);
		b3StoreRow(sim + FSIM_CENTER, center.x, center.y, center.z, reinterpret<f32>(load<u32>(sim + FSIM_FLAGS)));
		q = b3NormalizeQuat(b3MulQuat(deltaRotation, q));
		b3StoreRow(sim + FSIM_Q, q.v.x, q.v.y, q.v.z, q.s);

		// Use the velocity of the farthest point on the body to account for rotation.
		let maxExtent = load<b3Vec3>(sim + FSIM_MAXEXTENT);
		let velocityArc = b3ModifiedCross(b3Abs(localOmega), maxExtent);
		let maxVelocity = b3Length(v) + b3Length(velocityArc);

		// Sleep needs to observe position correction as well as true velocity.
		// q = [sin(theta/2) * v, cos(theta/2)]
		// for small angles abs(theta) ~= 2 * length(sin(theta/2) * v)
		let rotationArc = b3ModifiedCross(b3Abs(localDeltaRotation), maxExtent);
		let maxDeltaPosition: f32 = b3Length(deltaPosition) + 2.0 * b3Length(rotationArc);

		// Position correction is not as important for sleep as true velocity.
		let positionSleepFactor: f32 = 0.5;
		let sleepVelocity = b3MaxFloat(maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition);

		// Clear the transient flags (fast, speed capped, had TOI). These flags are conditionally set
		// as part of the code below.
		let stateFlags = load<u32>(state + FS_FLAGS);

		// reset state deltas, and the body state flag knows about speed capping (used for debug draw)
		b3StoreRow(state + FS_DP, 0.0, 0.0, 0.0, reinterpret<f32>(stateFlags & ~b3_bodyTransientFlags));
		b3StoreRow(state + FS_DQ, 0.0, 0.0, 0.0, 1.0);

		let localCenter = load<b3Vec3>(sim + FSIM_LOCALCENTER);
		let p = b3OffsetPos(center, b3Neg(b3RotateVector(q, localCenter)));
		b3StoreRow(sim + FSIM_P, p.x, p.y, p.z, q.v.x);

		// cache miss here, however I need the shape list below
		let body = bodies + <usize>bodyId * sizeof<b3Body>();
		store<i32>(body + FB_BODYMOVEINDEX, simIndex);
		store<f32>(body + FB_SLEEPVELOCITY, sleepVelocity);

		let moveEvent = moveEvents + <usize>simIndex * sizeof<b3BodyMoveEvent>();
		let moveId = b3MakeBodyMoveId(bodyId + 1, worldId, load<u16>(body + FB_GENERATION));
		store<u64>(moveEvent + ME_USERDATA, load<u64>(body + FB_USERDATA));
		b3StoreRow(moveEvent + ME_TRANSFORM, p.x, p.y, p.z, q.v.x);
		b3StoreRow(moveEvent + ME_TRANSFORM + offsetof<b3Transform>("q"), q.v.x, q.v.y, q.v.z, q.s);
		store<b3BodyId>(moveEvent + ME_BODYID, moveId);
		store<bool>(moveEvent + ME_FELLASLEEP, false);

		// reset applied force and torque
		b3StoreRow(sim + FSIM_FORCE, 0.0, 0.0, 0.0, 0.0);
		store<f32>(sim + FSIM_TORQUE + 4, 0.0);
		store<f32>(sim + FSIM_TORQUE + 8, 0.0);

		// If you hit this then it means you deferred mass computation but never called b3Body_ApplyMassFromShapes
		// or b3Body_SetMassData.
		B3_ASSERT((load<u32>(body + FB_FLAGS) & b3_dirtyMass) == 0);

		let bodyFlags = (load<u32>(body + FB_FLAGS) & ~b3_bodyTransientFlags) | (stateFlags & b3_isSpeedCapped);
		store<u32>(body + FB_FLAGS, bodyFlags);
		let simFlags = load<u32>(sim + FSIM_FLAGS) & ~(b3_isFast | b3_bodyTransientFlags);

		if (enableSleep == false || (bodyFlags & b3_enableSleep) == 0 || sleepVelocity > load<f32>(body + FB_SLEEPTHRESHOLD)) {
			// Body is not sleepy
			store<f32>(body + FB_SLEEPTIME, 0.0);

			let safetyFactor = load<f32>(body + FB_SAFETYFACTOR);
			let maxMotion = b3MaxFloat(maxDeltaPosition, maxVelocity * timeStep);
			if (
				load<i32>(body + FB_TYPE) == b3_dynamicBody &&
				enableContinuous &&
				maxMotion > safetyFactor * load<f32>(sim + FSIM_MINEXTENT)
			) {
				// This flag is used for debug draw and contact recycling.
				simFlags |= b3_isFast;
				store<u32>(sim + FSIM_FLAGS, simFlags);

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if ((simFlags & b3_isBullet) != 0) {
					let bulletIndex = b3AtomicFetchAddInt(stepContext.bulletBodyCount, 1);
					stepContext.bulletBodies[bulletIndex] = simIndex;
				} else {
					b3SolveContinuous(world, simIndex, taskContext, timeStep);
				}
			} else {
				// Body is safe to advance
				store<u32>(sim + FSIM_FLAGS, simFlags);
				b3StoreRow(sim + FSIM_CENTER0, center.x, center.y, center.z, localCenter.x);
				b3StoreRow(sim + FSIM_ROTATION0, q.v.x, q.v.y, q.v.z, q.s);
			}
		} else {
			// Body is safe to advance and is falling asleep
			store<u32>(sim + FSIM_FLAGS, simFlags);
			b3StoreRow(sim + FSIM_CENTER0, center.x, center.y, center.z, localCenter.x);
			b3StoreRow(sim + FSIM_ROTATION0, q.v.x, q.v.y, q.v.z, q.s);
			store<f32>(body + FB_SLEEPTIME, load<f32>(body + FB_SLEEPTIME) + timeStep);
		}

		// Update world space inverse inertia tensor. The sweep may have moved the body.
		let rotationMatrix = b3MakeMatrixFromQuat(load<b3Quat>(sim + FSIM_Q));
		b3StoreWorldInertia(sim + FSIM_INVINERTIAWORLD, rotationMatrix, load<b3Matrix3>(sim + FSIM_INVINERTIALOCAL));

		// Any single body in an island can keep it awake
		let sleepTime = load<f32>(body + FB_SLEEPTIME);
		let islandId = load<i32>(body + FB_ISLANDID);
		let island = world.islands[islandId];
		if (sleepTime < B3_TIME_TO_SLEEP) {
			// keep island awake
			let islandIndex = island.localIndex;
			b3SetBit(awakeIslandBitSet, <u32>islandIndex);
		} else if (island.constraintRemoveCount > 0) {
			// Body wants to sleep but its island needs splitting first. Track the sleepiest candidate.
			// Break sleep time ties using the island id to ensure determinism. The cross worker reduction
			// breaks ties the same way.
			if (
				sleepTime > taskContext.splitSleepTime ||
				(sleepTime == taskContext.splitSleepTime && islandId > taskContext.splitIslandId)
			) {
				// pick the sleepiest candidate
				taskContext.splitIslandId = islandId;
				taskContext.splitSleepTime = sleepTime;
			}
		}

		// Update shapes AABBs
		// For fast non-bullet bodies the AABB has already been updated in b3SolveContinuous
		// For fast bullet bodies the AABB will be updated at a later stage
		if ((load<u32>(sim + FSIM_FLAGS) & b3_isFast) == 0) {
			let transform = load<b3Transform>(sim + FSIM_TRANSFORM);
			let shapeId = load<i32>(body + FB_HEADSHAPEID);
			while (shapeId != B3_NULL_INDEX) {
				let shape = world.shapes[shapeId];

				let aabb = b3ComputeFatShapeAABB(shape, transform, speculativeScalar);
				shape.aabb = aabb;

				if (b3AABB_Contains(world.fatAABBs[shapeId], aabb) == false) {
					world.fatAABBs[shapeId] = b3AABB_Inflate(aabb, shape.aabbMargin);

					// Mark the hierarchy as moved using atomic operations.
					b3BroadPhase_MarkProxyMoved(world.broadPhase, shape.proxyKey, world.fatAABBs[shapeId]);
				}

				shapeId = shape.nextShapeId;
			}
		}
	}
}

struct b3BlockDim {
	// number of items per block (except last block)
	size: i32 = 0;

	// total number of blocks
	count: i32 = 0;
}

// A block is a range of tasks, a start index and count as a sub-array. Each worker receives at
// most M blocks of work. The workers may receive less blocks if there is not sufficient work.
// Each block of work has a minimum number of elements (block size). This in turn may limit the
// number of blocks. If there are many elements then the block size is increased so there are
// still at most M blocks of work per worker. M is a tunable number that has two goals:
// 1. keep M small to reduce overhead
// 2. keep M large enough for other workers to be able to steal work
// The block size is a power of two to make math efficient.
function b3ComputeBlockCount(itemCount: i32, minSize: i32, maxBlockCount: i32): b3BlockDim {
	let dim = b3BlockDim();
	if (itemCount == 0) {
		return dim;
	}

	if (itemCount <= minSize * maxBlockCount) {
		dim.size = minSize;
	} else {
		dim.size = (itemCount + maxBlockCount - 1) / maxBlockCount;
	}

	dim.count = (itemCount + dim.size - 1) / dim.size;

	B3_ASSERT(dim.count >= 1);
	B3_ASSERT(dim.size * dim.count >= itemCount);

	return dim;
}

// Initialize solver blocks for a contiguous range of items. Computes block size internally
// from the same parameters used by b3ComputeBlockCount. The atomic claim counter is zeroed
// so workers can CAS (0, 1) on the first stage that owns these blocks.
// C passes a pointer into the block buffer; here the buffer and the offset.
function b3InitBlocks(
	blocks: StaticArray<b3SyncBlock>,
	offset: i32,
	dim: b3BlockDim,
	itemCount: i32,
	blockType: u8,
	colorIndex: u8
): void {
	if (dim.count == 0) {
		return;
	}

	B3_ASSERT(itemCount >= dim.count);

	// Compute the number of elements per block
	let blockSize = dim.size;

	// Simulation too big
	B3_ASSERT(blockSize <= UINT16_MAX);

	for (let i = 0; i < dim.count; ++i) {
		ref b = blocks[offset + i];
		b.block.startIndex = i * blockSize;
		b.block.count = <u16>blockSize;
		b.block.blockType = blockType;
		b.block.colorIndex = colorIndex;
		b3AtomicStoreInt(b.syncIndex, 0);
	}

	// The last block may not be full
	blocks[offset + dim.count - 1].block.count = <u16>(itemCount - (dim.count - 1) * blockSize);
}

// Returns the next stage index, as C returns stage + 1.
function b3InitStage(
	stages: StaticArray<b3SolverStage>,
	stageIndex: i32,
	type: b3SolverStageType,
	blocks: StaticArray<b3SyncBlock>,
	blockCount: i32,
	colorIndex: u8
): i32 {
	let stage = stages[stageIndex];
	stage.type = type;
	stage.blocks = blocks;
	stage.blockCount = blockCount;
	stage.colorIndex = colorIndex;
	b3AtomicStoreInt(stage.completionCount, 0);
	return stageIndex + 1;
}

// Initialize one stage per color for each iteration. Used for warm start, solve, relax, and restitution.
// All iterations of a given color share the same b3SyncBlock array so the per-block syncIndex
// grows monotonically across stages within that color.
function b3InitColorStages(
	stages: StaticArray<b3SolverStage>,
	stageIndex: i32,
	type: b3SolverStageType,
	iterations: i32,
	activeColorCount: i32,
	colorBlocks: StaticArray<StaticArray<b3SyncBlock>>,
	colorBlockCounts: StaticArray<i32>,
	activeColorIndices: StaticArray<i32>
): i32 {
	for (let j = 0; j < iterations; ++j) {
		for (let i = 0; i < activeColorCount; ++i) {
			stageIndex = b3InitStage(stages, stageIndex, type, colorBlocks[i], colorBlockCounts[i], <u8>activeColorIndices[i]);
		}
	}
	return stageIndex;
}

function b3ExecuteBlock(stage: b3SolverStage, context: b3StepContext, block: b3SolverBlock, workerIndex: i32): void {
	let stageType = stage.type;
	let blockType = <b3SolverBlockType>block.blockType;

	switch (stageType) {
		case b3_stagePrepareJoints:
			b3PrepareJointsTask(block, context);
			break;

		case b3_stagePrepareWideContacts:
			b3PrepareContacts_Convex(block, context);
			break;

		case b3_stagePrepareContacts:
			b3PrepareContacts_MeshWide(block, context);
			break;

		case b3_stageIntegrateVelocities:
			b3IntegrateVelocitiesTask(block, context);
			break;

		case b3_stageWarmStart:
			if (blockType == b3_graphJointBlock) {
				b3WarmStartJointsTask(block, context);
			} else if (blockType == b3_graphWideContactBlock) {
				b3WarmStartContacts_Convex(block, context);
			} else {
				b3WarmStartContacts_MeshWide(block, context);
			}
			break;

		case b3_stageSolve:
			if (blockType == b3_graphJointBlock) {
				let useBias = true;
				b3SolveJointsTask(block, context, useBias, workerIndex);
			} else if (blockType == b3_graphWideContactBlock) {
				b3PushContacts_Convex(block, context);
			} else {
				b3PushContacts_MeshWide(block, context);
			}
			break;

		case b3_stageIntegratePositions:
			b3IntegratePositionsTask(block, context);
			break;

		case b3_stageRelax:
			if (blockType == b3_graphJointBlock) {
				let useBias = false;
				b3SolveJointsTask(block, context, useBias, workerIndex);
			} else if (blockType == b3_graphWideContactBlock) {
				b3SolveContacts_Convex(block, context);
			} else {
				b3SolveContacts_MeshWide(block, context);
			}
			break;

		case b3_stageRestitution:
			if (blockType == b3_graphWideContactBlock) {
				b3ApplyRestitution_Convex(block, context);
			} else if (blockType == b3_graphContactBlock) {
				b3ApplyRestitution_MeshWide(block, context);
			}
			break;

		case b3_stageStoreWideImpulses:
			b3StoreImpulses_Convex(block, context, workerIndex);
			break;

		case b3_stageStoreImpulses:
			b3StoreImpulses_MeshWide(block, context, workerIndex);
			break;
	}
}

// This staggers the worker start indices so they avoid touching the same solver blocks
@inline function GetWorkerStartIndex(workerIndex: i32, blockCount: i32, workerCount: i32): i32 {
	if (blockCount <= workerCount) {
		return workerIndex < blockCount ? workerIndex : B3_NULL_INDEX;
	}

	let blocksPerWorker = blockCount / workerCount;
	let remainder = blockCount - blocksPerWorker * workerCount;
	return blocksPerWorker * workerIndex + b3MinInt(remainder, workerIndex);
}

// Execute a stage, which is an array of solver blocks, each controlled with an atomic sync index.
// Each worker starts at its home index and sweeps the ring, CAS-claiming any unclaimed blocks.
function b3ExecuteStage(
	stage: b3SolverStage,
	context: b3StepContext,
	previousSyncIndex: i32,
	syncIndex: i32,
	workerIndex: i32
): void {
	let completedCount = 0;
	let blocks = stage.blocks;
	let blockCount = stage.blockCount;

	let startIndex = GetWorkerStartIndex(workerIndex, blockCount, context.workerCount);
	if (startIndex == B3_NULL_INDEX) {
		return;
	}

	B3_ASSERT(0 <= startIndex && startIndex < blockCount);

	let blockIndex = startIndex;
	for (let i = 0; i < blockCount; ++i) {
		if (b3AtomicCompareExchangeInt(blocks[blockIndex].syncIndex, previousSyncIndex, syncIndex)) {
			B3_ASSERT(completedCount < blockCount);

			// Pass the descriptor by value -- the wrapping b3SyncBlock holds the atomic
			// syncIndex but we only copy .block, so the struct copy never aliases the CAS target.
			b3ExecuteBlock(stage, context, blocks[blockIndex].block, workerIndex);
			completedCount += 1;
		}

		blockIndex += 1;
		if (blockIndex >= blockCount) {
			blockIndex = 0;
		}
	}

	b3AtomicFetchAddInt(stage.completionCount, completedCount);
}

// Execute a stage on worker 0 (main thread).
function b3ExecuteMainStage(stage: b3SolverStage, context: b3StepContext, syncBits: u32): void {
	let blockCount = stage.blockCount;
	if (blockCount == 0) {
		return;
	}

	const workerIndex = 0;

	if (blockCount == 1) {
		b3ExecuteBlock(stage, context, stage.blocks[0].block, workerIndex);
	} else {
		b3AtomicStoreU32(context.atomicSyncBits, syncBits);

		let syncIndex = <i32>((syncBits >> 16) & 0xffff);
		B3_ASSERT(syncIndex > 0);
		let previousSyncIndex = syncIndex - 1;

		b3ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

		// Spin waiting for thieves to finish
		while (b3AtomicLoadInt(stage.completionCount) != blockCount) {
			b3Pause();
		}

		b3AtomicStoreInt(stage.completionCount, 0);
	}
}

@inline function b3SyncBits(syncIndex: i32, stageIndex: i32): u32 {
	return (<u32>syncIndex << 16) | <u32>stageIndex;
}

// Parallel solver task
function b3SolverTask(workerContext: b3WorkerContext): void {
	let workerIndex = workerContext.workerIndex;
	let context = workerContext.context;
	let activeColorCount = context.activeColorCount;
	let stages = context.stages;
	let world = context.world;

	if (workerIndex == 0) {
		// The orchestrator slot is a race. The calling thread of b3World_Step also enters here
		// as worker 0, so progress is guaranteed even if the user's task system schedules tasks
		// out of order, has fewer threads than workerCount, or runs the task synchronously
		// inside enqueueTaskFcn. Whoever wins the CAS becomes the orchestrator; the loser
		// returns and lets the spinner-only path handle workers >0.
		if (b3AtomicCompareExchangeInt(context.mainClaimed, 0, 1) == false) {
			return;
		}

		// Main thread synchronizes the workers and does work itself.
		//
		// This needs to be a task for the main thread because the user's task system may execute
		// the tasks serially and this is the first task. This single task is able to fully
		// complete all work even if all other workers are blocked.

		// Stages are re-used by loops so that I don't need more stages for large substep counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		let ticks = b3Ticks();
		ticks.value = b3GetTicks();

		let bodySyncIndex = 1;
		let stageIndex = 0;

		// Prepare joint constraints
		let jointSyncIndex = 1;
		let syncBits = b3SyncBits(jointSyncIndex, stageIndex);
		B3_ASSERT(stages[stageIndex].type == b3_stagePrepareJoints);
		b3ExecuteMainStage(stages[stageIndex], context, syncBits);
		stageIndex += 1;
		jointSyncIndex += 1;

		// Prepare convex contact constraints
		let convexSyncIndex = 1;
		syncBits = b3SyncBits(convexSyncIndex, stageIndex);
		B3_ASSERT(stages[stageIndex].type == b3_stagePrepareWideContacts);
		b3ExecuteMainStage(stages[stageIndex], context, syncBits);
		stageIndex += 1;
		convexSyncIndex += 1;

		// Prepare mesh contact constraints
		let meshSyncIndex = 1;
		syncBits = b3SyncBits(meshSyncIndex, stageIndex);
		B3_ASSERT(stages[stageIndex].type == b3_stagePrepareContacts);
		b3ExecuteMainStage(stages[stageIndex], context, syncBits);
		stageIndex += 1;
		meshSyncIndex += 1;

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		b3PrepareJoints_Overflow(context);
		b3PrepareContacts_Overflow(context);

		world.profile.prepareConstraints += b3GetMillisecondsAndReset(ticks.value);

		let graphSyncIndex = 1;
		let subStepCount = context.subStepCount;
		for (let subStepIndex = 0; subStepIndex < subStepCount; ++subStepIndex) {
			// stageIndex restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			let iterationStageIndex = stageIndex;

			// Integrate velocities
			syncBits = b3SyncBits(bodySyncIndex, iterationStageIndex);
			B3_ASSERT(stages[iterationStageIndex].type == b3_stageIntegrateVelocities);
			b3ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
			iterationStageIndex += 1;
			bodySyncIndex += 1;

			world.profile.integrateVelocities += b3GetMillisecondsAndReset(ticks.value);

			// Warm start constraints
			b3WarmStartJoints_Overflow(context);
			b3WarmStartContacts_Overflow(context);

			for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
				syncBits = b3SyncBits(graphSyncIndex, iterationStageIndex);
				B3_ASSERT(stages[iterationStageIndex].type == b3_stageWarmStart);
				b3ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
				iterationStageIndex += 1;
			}
			graphSyncIndex += 1;

			world.profile.warmStart += b3GetMillisecondsAndReset(ticks.value);

			// Solve constraints
			let useBias = true;
			for (let j = 0; j < ITERATIONS; ++j) {
				// Overflow constraints have lower priority. Typically these are dynamic-vs-dynamic.
				b3SolveJoints_Overflow(context, useBias);
				b3PushContacts_Overflow(context);

				for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
					syncBits = b3SyncBits(graphSyncIndex, iterationStageIndex);
					B3_ASSERT(stages[iterationStageIndex].type == b3_stageSolve);
					b3ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			world.profile.solveImpulses += b3GetMillisecondsAndReset(ticks.value);

			// Integrate positions
			B3_ASSERT(stages[iterationStageIndex].type == b3_stageIntegratePositions);
			syncBits = b3SyncBits(bodySyncIndex, iterationStageIndex);
			b3ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
			iterationStageIndex += 1;
			bodySyncIndex += 1;

			world.profile.integratePositions += b3GetMillisecondsAndReset(ticks.value);

			// Relax constraints
			useBias = false;
			for (let j = 0; j < RELAX_ITERATIONS; ++j) {
				b3SolveJoints_Overflow(context, useBias);
				b3SolveContacts_Overflow(context);

				for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
					syncBits = b3SyncBits(graphSyncIndex, iterationStageIndex);
					B3_ASSERT(stages[iterationStageIndex].type == b3_stageRelax);
					b3ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			world.profile.relaxImpulses += b3GetMillisecondsAndReset(ticks.value);
		}

		// Advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + ITERATIONS * activeColorCount + 1 + RELAX_ITERATIONS * activeColorCount;

		let restitutionIterations = context.world.restitutionIterations;
		if (restitutionIterations > 0 && b3AtomicLoadInt(context.anyRestitution) != 0) {
			for (let j = 0; j < restitutionIterations; ++j) {
				b3ApplyRestitution_Overflow(context);

				let iterationStageIndex = stageIndex;
				for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
					syncBits = b3SyncBits(graphSyncIndex, iterationStageIndex);
					B3_ASSERT(stages[iterationStageIndex].type == b3_stageRestitution);
					b3ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
					iterationStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			world.profile.restitution += b3GetMillisecondsAndReset(ticks.value);
		}

		stageIndex += activeColorCount;

		// Store impulses
		b3StoreImpulses_Overflow(context);

		syncBits = b3SyncBits(convexSyncIndex, stageIndex);
		B3_ASSERT(stages[stageIndex].type == b3_stageStoreWideImpulses);
		b3ExecuteMainStage(stages[stageIndex], context, syncBits);
		stageIndex += 1;

		syncBits = b3SyncBits(meshSyncIndex, stageIndex);
		B3_ASSERT(stages[stageIndex].type == b3_stageStoreImpulses);
		b3ExecuteMainStage(stages[stageIndex], context, syncBits);
		stageIndex += 1;

		world.profile.storeImpulses += b3GetMillisecondsAndReset(ticks.value);

		// Signal workers to finish
		b3AtomicStoreU32(context.atomicSyncBits, UINT32_MAX);

		B3_ASSERT(stageIndex == context.stageCount);
		return;
	}

	// Worker spins and waits for work
	let lastSyncBits: u32 = 0;
	while (true) {
		// Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
		// parallel simulation with graph coloring.
		// todo improve this spinner
		let syncBits: u32 = b3AtomicLoadU32(context.atomicSyncBits);
		let spinCount = 0;
		while (syncBits == lastSyncBits) {
			if (spinCount > 5) {
				b3Yield();
				spinCount = 0;
			} else {
				b3Pause();
				b3Pause();
				spinCount += 1;
			}
			syncBits = b3AtomicLoadU32(context.atomicSyncBits);
		}

		if (syncBits == UINT32_MAX) {
			// sentinel hit
			break;
		}

		let stageIndex = <i32>(syncBits & 0xffff);
		B3_ASSERT(stageIndex < context.stageCount);

		let syncIndex = <i32>((syncBits >> 16) & 0xffff);
		B3_ASSERT(syncIndex > 0);

		let previousSyncIndex = syncIndex - 1;

		let stage = stages[stageIndex];
		b3ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

		lastSyncBits = syncBits;
	}
}

function b3BulletBodyTask(startIndex: i32, endIndex: i32, workerIndex: i32, stepContext: b3StepContext): void {
	let taskContext = stepContext.world.taskContexts[workerIndex];

	B3_ASSERT(startIndex <= endIndex);

	for (let i = startIndex; i < endIndex; ++i) {
		let simIndex = stepContext.bulletBodies[i];
		b3SolveContinuous(stepContext.world, simIndex, taskContext, stepContext.dt);
	}
}

// Sentinel span arrays; never read.
const b3_sentinelContactIds = new Array<i32>();
const b3_sentinelContactSpecs = new Array<b3ContactSpec>();
const b3_sentinelJointSims = new Array<b3JointSim>();
const b3_sentinelOrder = new Int32Array(0);

// C takes the per step buffers from the world's stack allocator; these grow only and every reader stays
// within the counts the step writes, so reusing them across steps and worlds keeps results.
class b3SolverScratch {
	bulletBodies: StaticArray<i32> = b3_emptyIndices;
	activeColorIndices: StaticArray<i32> = new StaticArray<i32>(B3_GRAPH_COLOR_COUNT);
	colorWideContactCounts: StaticArray<i32> = new StaticArray<i32>(B3_GRAPH_COLOR_COUNT);
	colorMeshContactCounts: StaticArray<i32> = new StaticArray<i32>(B3_GRAPH_COLOR_COUNT);
	colorMeshGroupCounts: StaticArray<i32> = new StaticArray<i32>(B3_GRAPH_COLOR_COUNT);
	colorJointCounts: StaticArray<i32> = new StaticArray<i32>(B3_GRAPH_COLOR_COUNT);
	graphWideContactDims: StaticArray<b3BlockDim> = new StaticArray<b3BlockDim>(B3_GRAPH_COLOR_COUNT);
	graphMeshContactDims: StaticArray<b3BlockDim> = new StaticArray<b3BlockDim>(B3_GRAPH_COLOR_COUNT);
	graphJointDims: StaticArray<b3BlockDim> = new StaticArray<b3BlockDim>(B3_GRAPH_COLOR_COUNT);
	graphBlockCounts: StaticArray<i32> = new StaticArray<i32>(B3_GRAPH_COLOR_COUNT);
	bucketStarts: StaticArray<i32> = b3_emptyIndices;
	wideConstraints: StaticArray<b3ContactConstraintWide> = b3_emptyWideConstraints;
	wideManifolds: StaticArray<b3ContactManifoldWide> = b3_emptyWideManifolds;
	wideMeshConstraints: StaticArray<b3ContactConstraintWide> = b3_emptyWideConstraints;
	wideMeshManifolds: StaticArray<b3ContactManifoldWide> = b3_emptyWideManifolds;
	meshLaneOrder: Int32Array = b3_sentinelOrder;
	meshManifoldStarts: Int32Array = b3_sentinelOrder;
	contactConstraints: StaticArray<b3ContactConstraint> = b3_emptyContactConstraints;
	manifoldConstraints: StaticArray<b3ManifoldConstraint> = b3_emptyManifoldConstraints;
	widePrepareSpans: StaticArray<b3WidePrepareSpan> = new StaticArray<b3WidePrepareSpan>(B3_GRAPH_COLOR_COUNT + 1);
	meshPrepareSpans: StaticArray<b3MeshPrepareSpan> = new StaticArray<b3MeshPrepareSpan>(B3_GRAPH_COLOR_COUNT + 1);
	jointPrepareSpans: StaticArray<b3JointPrepareSpan> = new StaticArray<b3JointPrepareSpan>(B3_GRAPH_COLOR_COUNT + 1);
	overflowSpans: StaticArray<b3ContactPrepareSpan> = new StaticArray<b3ContactPrepareSpan>(2);
	stages: StaticArray<b3SolverStage> = b3_emptyStages;
	bodyBlocks: StaticArray<b3SyncBlock> = b3_emptySyncBlocks;
	convexBlocks: StaticArray<b3SyncBlock> = b3_emptySyncBlocks;
	meshBlocks: StaticArray<b3SyncBlock> = b3_emptySyncBlocks;
	jointBlocks: StaticArray<b3SyncBlock> = b3_emptySyncBlocks;
	graphColorBlocks: StaticArray<StaticArray<b3SyncBlock>> = new StaticArray<StaticArray<b3SyncBlock>>(B3_GRAPH_COLOR_COUNT);
	workerContexts: StaticArray<b3WorkerContext> = new StaticArray<b3WorkerContext>(0);
	workerNames: StaticArray<string> = new StaticArray<string>(0);
	callerContext: b3WorkerContext? = null;

	constructor() {
		for (let i = 0; i <= B3_GRAPH_COLOR_COUNT; ++i) {
			this.widePrepareSpans[i] = new b3WidePrepareSpan(0, 0, b3_sentinelContactIds);
			this.meshPrepareSpans[i] = new b3MeshPrepareSpan(0, 0, b3_sentinelContactSpecs, b3_sentinelOrder);
			this.jointPrepareSpans[i] = new b3JointPrepareSpan(0, 0, b3_sentinelJointSims);
		}
		this.overflowSpans[0] = new b3ContactPrepareSpan(0, 0, b3_sentinelContactSpecs);
		this.overflowSpans[1] = new b3ContactPrepareSpan(0, 0, b3_sentinelContactSpecs);
		for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
			this.graphColorBlocks[i] = b3_emptySyncBlocks;
		}
	}
}

const b3_solverScratch = new b3SolverScratch();

function b3SetWidePrepareSpan(span: b3WidePrepareSpan, start: i32, count: i32, contacts: Array<i32>): void {
	span.start = start;
	span.count = count;
	span.contacts = contacts;
}

function b3SetContactPrepareSpan(span: b3ContactPrepareSpan, start: i32, count: i32, contacts: Array<b3ContactSpec>): void {
	span.start = start;
	span.count = count;
	span.contacts = contacts;
}

function b3SetMeshPrepareSpan(
	span: b3MeshPrepareSpan,
	start: i32,
	count: i32,
	contacts: Array<b3ContactSpec>,
	order: Int32Array,
	orderBase: i32
): void {
	span.start = start;
	span.count = count;
	span.contacts = contacts;
	span.order = order;
	span.orderBase = orderBase;
}

function b3SetJointPrepareSpan(span: b3JointPrepareSpan, start: i32, count: i32, joints: Array<b3JointSim>): void {
	span.start = start;
	span.count = count;
	span.joints = joints;
}

@inline function b3GrowCapacity(count: i32): i32 {
	return count + (count >> 2) + 16;
}

function b3ReserveIndices(a: StaticArray<i32>, count: i32): StaticArray<i32> {
	return a.length >= count ? a : new StaticArray<i32>(b3GrowCapacity(count));
}

function b3ReserveInt32Array(a: Int32Array, count: i32): Int32Array {
	return a.length >= count ? a : new Int32Array(b3GrowCapacity(count));
}

function b3ReserveSyncBlocks(a: StaticArray<b3SyncBlock>, count: i32): StaticArray<b3SyncBlock> {
	return a.length >= count ? a : new StaticArray<b3SyncBlock>(b3GrowCapacity(count));
}

function b3ReserveWideConstraints(
	a: StaticArray<b3ContactConstraintWide>,
	count: i32
): StaticArray<b3ContactConstraintWide> {
	return a.length >= count ? a : new StaticArray<b3ContactConstraintWide>(b3GrowCapacity(count));
}

function b3ReserveWideManifolds(a: StaticArray<b3ContactManifoldWide>, count: i32): StaticArray<b3ContactManifoldWide> {
	return a.length >= count ? a : new StaticArray<b3ContactManifoldWide>(b3GrowCapacity(count));
}

function b3ReserveContactConstraints(a: StaticArray<b3ContactConstraint>, count: i32): StaticArray<b3ContactConstraint> {
	return a.length >= count ? a : new StaticArray<b3ContactConstraint>(b3GrowCapacity(count));
}

function b3ReserveManifoldConstraints(
	a: StaticArray<b3ManifoldConstraint>,
	count: i32
): StaticArray<b3ManifoldConstraint> {
	return a.length >= count ? a : new StaticArray<b3ManifoldConstraint>(b3GrowCapacity(count));
}

function b3ReserveStages(a: StaticArray<b3SolverStage>, count: i32): StaticArray<b3SolverStage> {
	if (a.length >= count) {
		return a;
	}
	let stages = new StaticArray<b3SolverStage>(b3GrowCapacity(count));
	for (let i = 0; i < stages.length; ++i) {
		stages[i] = i < a.length ? a[i] : new b3SolverStage();
	}
	return stages;
}

function b3MakeJointEventId(index1: i32, world0: u16, generation: u16): b3JointId {
	let id = b3JointId();
	id.index1 = index1;
	id.world0 = world0;
	id.generation = generation;
	return id;
}

// Solve with graph coloring
export function b3Solve(world: b3World, stepContext: b3StepContext): void {
	let simdShift = world.simdWidth == 8 ? 3 : 2;

	// Only count steps that advance the simulation
	world.stepIndex += 1;

	let awakeSet = world.solverSets[b3_awakeSet];
	let awakeBodyCount = awakeSet.bodySims.length;
	if (awakeBodyCount == 0) {
		return;
	}

	// Solve constraints using graph coloring
	{
		let setupTicks = b3Ticks();
		setupTicks.value = b3GetTicks();

		// Prepare buffers for continuous collision (fast bodies)
		b3AtomicStoreInt(stepContext.bulletBodyCount, 0);
		let scratch = b3_solverScratch;
		scratch.bulletBodies = b3ReserveIndices(scratch.bulletBodies, awakeBodyCount);
		stepContext.bulletBodies = scratch.bulletBodies;

		let graph = world.constraintGraph;
		let colors = graph.colors;

		stepContext.sims = awakeSet.bodySims;
		stepContext.states = awakeSet.bodyStates;

		// count contacts, joints, and colors
		let activeColorCount = 0;
		for (let i = 0; i < B3_GRAPH_COLOR_COUNT - 1; ++i) {
			let perColorContactCount = colors[i].convexContacts.length + colors[i].contacts.length;
			let perColorJointCount = colors[i].jointSims.length;
			let occupancyCount = perColorContactCount + perColorJointCount;
			activeColorCount += occupancyCount > 0 ? 1 : 0;
		}

		// prepare for move events
		world.bodyMoveEvents.length = awakeBodyCount;

		let workerCount = world.workerCount;

		// Target 4 blocks per worker to allow work stealing
		const maxBlockCount = 4 * workerCount;

		// Body blocks are for parallel iteration over bodies directly (integration, update transforms)
		let minBodiesPerBlock = 32;
		let bodyDim = b3ComputeBlockCount(awakeBodyCount, minBodiesPerBlock, maxBlockCount);

		const minContactsPerBlock = 4;
		const minJointsPerBlock = 4;

		// Configure blocks for tasks parallel-for each active graph color
		// The blocks are a mix of convex contact, mesh contact, and joint blocks
		let activeColorIndices = scratch.activeColorIndices;
		let colorWideContactCounts = scratch.colorWideContactCounts;
		let colorMeshContactCounts = scratch.colorMeshContactCounts;
		let colorMeshGroupCounts = scratch.colorMeshGroupCounts;
		let colorJointCounts = scratch.colorJointCounts;
		let graphWideContactDims = scratch.graphWideContactDims;
		let graphMeshContactDims = scratch.graphMeshContactDims;
		let graphJointDims = scratch.graphJointDims;
		let graphBlockCount = 0;

		// c is the active color index
		let wideContactCount = 0;
		let meshContactCount = 0;
		let meshGroupCount = 0;
		let jointCount = 0;
		let c = 0;
		for (let i = 0; i < B3_GRAPH_COLOR_COUNT - 1; ++i) {
			let color = colors[i];
			let colorConvexContactCount = color.convexContacts.length;
			let colorMeshContactCount = color.contacts.length;
			let colorJointCount = color.jointSims.length;

			if (colorConvexContactCount + colorMeshContactCount + colorJointCount == 0) {
				continue;
			}

			activeColorIndices[c] = i;

			// Ceiling for wide constraint count
			let colorWideConstraintCount = colorConvexContactCount > 0 ? ((colorConvexContactCount - 1) >> simdShift) + 1 : 0;
			wideContactCount += colorWideConstraintCount;
			colorWideContactCounts[c] = colorWideConstraintCount;

			colorMeshContactCounts[c] = colorMeshContactCount;
			meshContactCount += colorMeshContactCount;

			let colorMeshGroupCount = colorMeshContactCount > 0 ? ((colorMeshContactCount - 1) >> simdShift) + 1 : 0;
			colorMeshGroupCounts[c] = colorMeshGroupCount;
			meshGroupCount += colorMeshGroupCount;

			colorJointCounts[c] = colorJointCount;
			jointCount += colorJointCount;

			// Solver block dimensions
			graphWideContactDims[c] = b3ComputeBlockCount(colorWideConstraintCount, minContactsPerBlock, maxBlockCount);
			graphMeshContactDims[c] = b3ComputeBlockCount(colorMeshGroupCount, minContactsPerBlock, maxBlockCount);
			graphJointDims[c] = b3ComputeBlockCount(colorJointCount, minJointsPerBlock, maxBlockCount);
			graphBlockCount += graphWideContactDims[c].count + graphMeshContactDims[c].count + graphJointDims[c].count;

			c += 1;
		}
		activeColorCount = c;

		// Prepare and store run as one flat parallel-for over the entire wide constraint range,
		// partitioned into uniformly sized blocks. Color info is consulted inside the task via
		// a small span array, so blocks do not need to honor color boundaries here.
		let convexPrepareDim = b3ComputeBlockCount(wideContactCount, minContactsPerBlock, maxBlockCount);
		let meshPrepareDim = b3ComputeBlockCount(meshGroupCount, minContactsPerBlock, maxBlockCount);
		let jointPrepareDim = b3ComputeBlockCount(jointCount, minJointsPerBlock, maxBlockCount);

		// C's interleaved stream is split into parallel constraint and manifold arrays.
		scratch.wideConstraints = b3ReserveWideConstraints(scratch.wideConstraints, wideContactCount);
		scratch.wideManifolds = b3ReserveWideManifolds(scratch.wideManifolds, wideContactCount);
		let wideConstraints = scratch.wideConstraints;
		let wideManifolds = scratch.wideManifolds;

		scratch.meshLaneOrder = b3ReserveInt32Array(scratch.meshLaneOrder, meshContactCount);
		scratch.meshManifoldStarts = b3ReserveInt32Array(scratch.meshManifoldStarts, meshGroupCount + 1);
		let meshLaneOrder = scratch.meshLaneOrder;
		let meshManifoldStarts = scratch.meshManifoldStarts;

		// Sort mesh contacts by manifold count so that contacts with more manifolds are solved
		// together keeping the SIMD lanes fuller and less ragged.
		let meshSlotCount = 0;
		{
			let maxManifoldCount = 0;
			for (let i = 0; i < activeColorCount; ++i) {
				let color = colors[activeColorIndices[i]];
				let specs = color.contacts;
				let colorContactCount = colorMeshContactCounts[i];
				for (let j = 0; j < colorContactCount; ++j) {
					maxManifoldCount = b3MaxInt(maxManifoldCount, <i32>specs[j].manifoldCount);
				}
			}

			scratch.bucketStarts = b3ReserveIndices(scratch.bucketStarts, maxManifoldCount + 1);
			let bucketStarts = scratch.bucketStarts;

			let orderBase = 0;
			let groupBase = 0;
			for (let i = 0; i < activeColorCount; ++i) {
				let color = colors[activeColorIndices[i]];
				let colorContactCount = colorMeshContactCounts[i];
				if (colorContactCount == 0) {
					continue;
				}

				let specs = color.contacts;

				for (let k = 0; k <= maxManifoldCount; ++k) {
					bucketStarts[k] = 0;
				}

				for (let j = 0; j < colorContactCount; ++j) {
					bucketStarts[<i32>specs[j].manifoldCount] += 1;
				}

				let start = 0;
				for (let k = maxManifoldCount; k >= 0; --k) {
					let count = bucketStarts[k];
					bucketStarts[k] = start;
					start += count;
				}

				// C's order pointer is meshLaneOrder + orderBase
				for (let j = 0; j < colorContactCount; ++j) {
					let bucket = <i32>specs[j].manifoldCount;
					meshLaneOrder[orderBase + bucketStarts[bucket]] = j;
					bucketStarts[bucket] += 1;
				}

				let colorMeshGroupCount = colorMeshGroupCounts[i];
				for (let g = 0; g < colorMeshGroupCount; ++g) {
					let slotCount = <i32>specs[meshLaneOrder[orderBase + (g << simdShift)]].manifoldCount;
					meshManifoldStarts[groupBase + g] = meshSlotCount;
					meshSlotCount += slotCount;
				}

				orderBase += colorContactCount;
				groupBase += colorMeshGroupCount;
			}

			B3_ASSERT(orderBase == meshContactCount);
			B3_ASSERT(groupBase == meshGroupCount);
			meshManifoldStarts[meshGroupCount] = meshSlotCount;
		}

		scratch.wideMeshConstraints = b3ReserveWideConstraints(scratch.wideMeshConstraints, meshGroupCount);
		scratch.wideMeshManifolds = b3ReserveWideManifolds(scratch.wideMeshManifolds, meshSlotCount);
		let wideMeshConstraints = scratch.wideMeshConstraints;
		let wideMeshManifolds = scratch.wideMeshManifolds;

		let overflow = colors[B3_OVERFLOW_INDEX];
		let overflowCount = overflow.contacts.length;
		let overflowManifoldCount = 0;
		for (let i = 0; i < overflowCount; ++i) {
			overflow.contacts[i].manifoldStart = overflowManifoldCount;
			overflowManifoldCount += <i32>overflow.contacts[i].manifoldCount;
		}

		scratch.contactConstraints = b3ReserveContactConstraints(scratch.contactConstraints, overflowCount);
		scratch.manifoldConstraints = b3ReserveManifoldConstraints(scratch.manifoldConstraints, overflowManifoldCount);
		overflow.contactConstraints = scratch.contactConstraints;
		overflow.manifoldConstraints = scratch.manifoldConstraints;

		// Build the span table for the flat prepare/store parallel-for while I slice the
		// wide constraint buffer across colors. One entry per active color plus a sentinel
		// at wideContactCount.
		let widePrepareSpans = scratch.widePrepareSpans;
		let meshPrepareSpans = scratch.meshPrepareSpans;
		let jointPrepareSpans = scratch.jointPrepareSpans;

		// Distribute transient constraints to each graph color and prepare spans
		{
			let wideBase = 0;
			let contactBase = 0;
			let meshGroupBase = 0;
			let jointBase = 0;
			for (let i = 0; i < activeColorCount; ++i) {
				let j = activeColorIndices[i];
				let color = colors[j];

				let colorConvexContactCount = color.convexContacts.length;
				b3SetWidePrepareSpan(widePrepareSpans[i], wideBase, colorConvexContactCount, color.convexContacts);

				if (colorConvexContactCount == 0) {
					color.wideConstraints = b3_emptyWideConstraints;
					color.wideManifolds = b3_emptyWideManifolds;
					color.wideConstraintStart = 0;
					color.wideConstraintCount = 0;
				} else {
					color.wideConstraints = wideConstraints;
					color.wideManifolds = wideManifolds;
					color.wideConstraintStart = wideBase;

					let colorContactCountW = ((colorConvexContactCount - 1) >> simdShift) + 1;
					color.wideConstraintCount = colorContactCountW;
					wideBase += colorContactCountW;
				}

				let colorContactCount = color.contacts.length;
				b3SetMeshPrepareSpan(meshPrepareSpans[i], meshGroupBase, colorContactCount, color.contacts, meshLaneOrder, contactBase);

				if (colorContactCount == 0) {
					color.wideMeshConstraints = b3_emptyWideConstraints;
					color.wideMeshManifolds = b3_emptyWideManifolds;
					color.wideMeshConstraintStart = 0;
					color.wideMeshManifoldStarts = b3_sentinelOrder;
					color.wideMeshConstraintCount = 0;
				} else {
					let colorMeshGroupCount = colorMeshGroupCounts[i];
					color.wideMeshConstraints = wideMeshConstraints;
					color.wideMeshManifolds = wideMeshManifolds;
					color.wideMeshConstraintStart = meshGroupBase;
					color.wideMeshManifoldStarts = meshManifoldStarts.subarray(meshGroupBase);
					color.wideMeshConstraintCount = colorMeshGroupCount;
					meshGroupBase += colorMeshGroupCount;
					contactBase += colorContactCount;
				}

				b3SetJointPrepareSpan(jointPrepareSpans[i], jointBase, color.jointSims.length, color.jointSims);
				jointBase += color.jointSims.length;
			}

			// Sentinels
			b3SetWidePrepareSpan(widePrepareSpans[activeColorCount], wideContactCount, 0, b3_sentinelContactIds);
			B3_ASSERT(wideBase == wideContactCount);

			b3SetMeshPrepareSpan(meshPrepareSpans[activeColorCount], meshGroupCount, 0, b3_sentinelContactSpecs, b3_sentinelOrder, 0);
			B3_ASSERT(contactBase == meshContactCount);
			B3_ASSERT(meshGroupBase == meshGroupCount);

			b3SetJointPrepareSpan(jointPrepareSpans[activeColorCount], jointCount, 0, b3_sentinelJointSims);
			B3_ASSERT(jointBase == jointCount);
		}

		//// Special span for overflow to allow for function re-use
		let overflowSpans = scratch.overflowSpans;
		b3SetContactPrepareSpan(overflowSpans[0], 0, overflow.contacts.length, overflow.contacts);
		b3SetContactPrepareSpan(overflowSpans[1], overflow.contacts.length, 0, b3_sentinelContactSpecs);

		let stageCount = 0;

		// b3_stagePrepareJoints
		stageCount += 1;
		// b3_stagePrepareWideContacts
		stageCount += 1;
		// b3_stagePrepareContacts
		stageCount += 1;
		// b3_stageIntegrateVelocities
		stageCount += 1;
		// b3_stageWarmStart
		stageCount += activeColorCount;
		// b3_stageSolve
		stageCount += ITERATIONS * activeColorCount;
		// b3_stageIntegratePositions
		stageCount += 1;
		// b3_stageRelax
		stageCount += RELAX_ITERATIONS * activeColorCount;
		// b3_stageRestitution
		stageCount += activeColorCount;
		// b3_stageStoreWideImpulses
		stageCount += 1;
		// b3_stageStoreImpulses
		stageCount += 1;

		scratch.stages = b3ReserveStages(scratch.stages, stageCount);
		scratch.bodyBlocks = b3ReserveSyncBlocks(scratch.bodyBlocks, bodyDim.count);
		scratch.convexBlocks = b3ReserveSyncBlocks(scratch.convexBlocks, convexPrepareDim.count);
		scratch.meshBlocks = b3ReserveSyncBlocks(scratch.meshBlocks, meshPrepareDim.count);
		scratch.jointBlocks = b3ReserveSyncBlocks(scratch.jointBlocks, jointPrepareDim.count);
		let stages = scratch.stages;
		let bodyBlocks = scratch.bodyBlocks;
		let convexBlocks = scratch.convexBlocks;
		let meshBlocks = scratch.meshBlocks;
		let jointBlocks = scratch.jointBlocks;

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
		// Note: cannot split islands in parallel with FinalizeBodies
		let splitIslandTask: u64 = 0;
		if (world.splitIslandId != B3_NULL_INDEX) {
			if (world.taskCount < B3_MAX_TASKS) {
				splitIslandTask = world.enqueueTaskFcn(
					(taskContext: u64): void => b3SplitIslandTask(world),
					0,
					world.userTaskContext,
					"split"
				);
				world.taskCount += 1;
				world.activeTaskCount += splitIslandTask == 0 ? 0 : 1;
			} else {
				b3SplitIslandTask(world);
			}
		}

		// Prepare body blocks
		b3InitBlocks(bodyBlocks, 0, bodyDim, awakeBodyCount, <u8>b3_bodyBlock, <u8>UINT8_MAX);

		// Prepare blocks as a single flat parallel-for over the whole constraint range.
		// The task walks spans to decode flat slot indices back to per-color arrays.
		b3InitBlocks(convexBlocks, 0, convexPrepareDim, wideContactCount, <u8>b3_wideContactBlock, <u8>UINT8_MAX);
		b3InitBlocks(meshBlocks, 0, meshPrepareDim, meshGroupCount, <u8>b3_contactBlock, <u8>UINT8_MAX);
		b3InitBlocks(jointBlocks, 0, jointPrepareDim, jointCount, <u8>b3_jointBlock, <u8>UINT8_MAX);

		// Prepare graph work blocks. Each color gets joint blocks followed by contact blocks.
		// C slices one graph block buffer per color; here each color owns its blocks.
		let graphColorBlocks = scratch.graphColorBlocks;
		let graphBlockCounts = scratch.graphBlockCounts;
		let graphBlockTotal = 0;
		for (let i = 0; i < activeColorCount; ++i) {
			let colorBlockCount = graphJointDims[i].count + graphWideContactDims[i].count + graphMeshContactDims[i].count;
			let colorBlocks = b3ReserveSyncBlocks(graphColorBlocks[i], colorBlockCount);
			graphColorBlocks[i] = colorBlocks;

			let colorIndex = <u8>activeColorIndices[i];
			let offset = 0;
			b3InitBlocks(colorBlocks, offset, graphJointDims[i], colorJointCounts[i], <u8>b3_graphJointBlock, colorIndex);
			offset += graphJointDims[i].count;

			b3InitBlocks(
				colorBlocks,
				offset,
				graphWideContactDims[i],
				colorWideContactCounts[i],
				<u8>b3_graphWideContactBlock,
				colorIndex
			);
			offset += graphWideContactDims[i].count;

			b3InitBlocks(colorBlocks, offset, graphMeshContactDims[i], colorMeshGroupCounts[i], <u8>b3_graphContactBlock, colorIndex);
			offset += graphMeshContactDims[i].count;

			graphBlockCounts[i] = colorBlockCount;
			graphBlockTotal += offset;
		}

		B3_ASSERT(graphBlockTotal == graphBlockCount);

		let stage = 0;
		stage = b3InitStage(stages, stage, b3_stagePrepareJoints, jointBlocks, jointPrepareDim.count, <u8>UINT8_MAX);
		stage = b3InitStage(stages, stage, b3_stagePrepareWideContacts, convexBlocks, convexPrepareDim.count, <u8>UINT8_MAX);
		stage = b3InitStage(stages, stage, b3_stagePrepareContacts, meshBlocks, meshPrepareDim.count, <u8>UINT8_MAX);
		stage = b3InitStage(stages, stage, b3_stageIntegrateVelocities, bodyBlocks, bodyDim.count, <u8>UINT8_MAX);
		stage = b3InitColorStages(
			stages,
			stage,
			b3_stageWarmStart,
			1,
			activeColorCount,
			graphColorBlocks,
			graphBlockCounts,
			activeColorIndices
		);
		stage = b3InitColorStages(
			stages,
			stage,
			b3_stageSolve,
			ITERATIONS,
			activeColorCount,
			graphColorBlocks,
			graphBlockCounts,
			activeColorIndices
		);
		stage = b3InitStage(stages, stage, b3_stageIntegratePositions, bodyBlocks, bodyDim.count, <u8>UINT8_MAX);
		stage = b3InitColorStages(
			stages,
			stage,
			b3_stageRelax,
			RELAX_ITERATIONS,
			activeColorCount,
			graphColorBlocks,
			graphBlockCounts,
			activeColorIndices
		);
		stage = b3InitColorStages(
			stages,
			stage,
			b3_stageRestitution,
			1,
			activeColorCount,
			graphColorBlocks,
			graphBlockCounts,
			activeColorIndices
		);
		stage = b3InitStage(stages, stage, b3_stageStoreWideImpulses, convexBlocks, convexPrepareDim.count, <u8>UINT8_MAX);
		stage = b3InitStage(stages, stage, b3_stageStoreImpulses, meshBlocks, meshPrepareDim.count, <u8>UINT8_MAX);

		B3_ASSERT(stage == stageCount);

		B3_ASSERT(workerCount <= B3_MAX_WORKERS);
		if (scratch.workerContexts.length < workerCount) {
			let contexts = new StaticArray<b3WorkerContext>(workerCount);
			let names = new StaticArray<string>(workerCount);
			for (let i = 0; i < workerCount; ++i) {
				contexts[i] = new b3WorkerContext(stepContext, i);
				names[i] = "solve[" + i.toString() + "]";
			}
			scratch.workerContexts = contexts;
			scratch.workerNames = names;
			scratch.callerContext = new b3WorkerContext(stepContext, 0);
		}
		let workerContext = scratch.workerContexts;

		stepContext.graph = graph;
		stepContext.activeColorCount = activeColorCount;
		stepContext.workerCount = workerCount;
		stepContext.stageCount = stageCount;
		stepContext.stages = stages;
		stepContext.wideConstraints = wideConstraints;
		stepContext.wideManifolds = wideManifolds;
		stepContext.widePrepareSpans = widePrepareSpans;
		stepContext.wideContactCount = wideContactCount;
		stepContext.overflowSpans = overflowSpans;
		stepContext.wideMeshConstraints = wideMeshConstraints;
		stepContext.wideMeshManifolds = wideMeshManifolds;
		stepContext.wideMeshManifoldStarts = meshManifoldStarts;
		stepContext.meshPrepareSpans = meshPrepareSpans;
		stepContext.jointPrepareSpans = jointPrepareSpans;
		b3AtomicStoreU32(stepContext.atomicSyncBits, 0);
		b3AtomicStoreInt(stepContext.mainClaimed, 0);

		world.profile.solverSetup = b3GetMillisecondsAndReset(setupTicks.value);

		let constraintTicks = b3Ticks();
		constraintTicks.value = b3GetTicks();

		let jointIdCapacity = b3GetIdCapacity(world.jointIdPool);
		let contactIdCapacity = b3GetIdCapacity(world.contactIdPool);
		for (let i = 0; i < workerCount; ++i) {
			let taskContext = world.taskContexts[i];
			b3SetBitCountAndClear(taskContext.jointStateBitSet, <u32>jointIdCapacity);
			b3SetBitCountAndClear(taskContext.hitEventBitSet, <u32>contactIdCapacity);
			taskContext.hasHitEvents = false;

			let worker = workerContext[i];
			worker.context = stepContext;

			if (world.taskCount < B3_MAX_TASKS) {
				worker.userTask = world.enqueueTaskFcn(
					(taskContext: u64): void => b3SolverTask(worker),
					0,
					world.userTaskContext,
					scratch.workerNames[i]
				);
				world.taskCount += 1;
				world.activeTaskCount += worker.userTask == 0 ? 0 : 1;
			} else {
				worker.userTask = 0;
				b3SolverTask(worker);
			}
		}

		// The calling thread of b3World_Step also enters b3SolverTask as worker 0 and races for the
		// orchestrator slot via the CAS inside. This guarantees progress even when the user's task
		// system can't run the queued worker 0 promptly: it might schedule out of order, have fewer
		// threads than workerCount, or invert priority by parking the calling thread in finishTaskFcn.
		// Whoever wins the CAS becomes the orchestrator; the loser returns and lets the spinner-only
		// path handle workers >0.
		let callerContext = scratch.callerContext!;
		callerContext.context = stepContext;
		callerContext.userTask = 0;
		b3SolverTask(callerContext);

		// Finish constraint solve
		for (let i = 0; i < workerCount; ++i) {
			if (workerContext[i].userTask != 0) {
				world.finishTaskFcn(workerContext[i].userTask, world.userTaskContext);
				world.activeTaskCount -= 1;
			}
		}

		// Finish island split
		if (splitIslandTask != 0) {
			world.finishTaskFcn(splitIslandTask, world.userTaskContext);
			world.activeTaskCount -= 1;
		}
		world.splitIslandId = B3_NULL_INDEX;

		world.profile.constraints = b3GetMillisecondsAndReset(constraintTicks.value);

		let transformTicks = b3GetTicks();

		// Prepare the island bit set used in body finalization.
		let awakeIslandCount = awakeSet.islandSims.length;
		for (let i = 0; i < world.workerCount; ++i) {
			let taskContext = world.taskContexts[i];
			taskContext.sensorHits.length = 0;
			b3SetBitCountAndClear(taskContext.awakeIslandBitSet, <u32>awakeIslandCount);
			taskContext.splitIslandId = B3_NULL_INDEX;
			taskContext.splitSleepTime = 0.0;
		}

		// Finish the user tree task that was queued earlier in the time step. This must be complete before touching the
		// broad-phase.
		if (world.userTreeTask != 0) {
			world.finishTaskFcn(world.userTreeTask, world.userTaskContext);
			world.userTreeTask = 0;
			world.activeTaskCount -= 1;
		}

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		b3ParallelFor(
			world,
			(startIndex: i32, endIndex: i32, workerIndex: i32): void =>
				b3FinalizeBodiesTask(startIndex, endIndex, workerIndex, stepContext),
			awakeBodyCount,
			8,
			"ccd"
		);

		overflow.manifoldConstraints = b3_emptyManifoldConstraints;
		overflow.contactConstraints = b3_emptyContactConstraints;

		world.profile.transforms = b3GetMilliseconds(transformTicks);
	}

	// Report joint events
	{
		let jointEventTicks = b3GetTicks();

		// Gather bits for all joints that have force/torque events
		let jointStateBitSet = world.taskContexts[0].jointStateBitSet;
		for (let i = 1; i < world.workerCount; ++i) {
			b3InPlaceUnion(jointStateBitSet, world.taskContexts[i].jointStateBitSet);
		}

		{
			let wordCount = jointStateBitSet.blockCount;
			let bits = jointStateBitSet.bits;

			let jointArray = world.joints;
			let worldIndex0 = world.worldId;

			for (let k: u32 = 0; k < wordCount; ++k) {
				let word = bits[<i32>k];
				while (word != 0) {
					let ctz = b3CTZ64(word);
					let jointId = <i32>(64 * k + ctz);

					B3_ASSERT(jointId < jointArray.length);

					B3_ASSERT(jointArray[jointId].setIndex == b3_awakeSet);

					let event = b3JointEvent();
					event.jointId = b3MakeJointEventId(jointId + 1, worldIndex0, jointArray[jointId].generation);
					event.userData = jointArray[jointId].userData;

					world.jointEvents.push(event);

					// Clear the smallest set bit
					word = word & (word - 1);
				}
			}
		}

		world.profile.jointEvents = b3GetMilliseconds(jointEventTicks);
	}

	// Report hit events
	{
		let hitTicks = b3GetTicks();

		B3_ASSERT(world.contactHitEvents.length == 0);

		// Fast path: if no worker flagged any hit-event candidates during b2StoreImpulsesTask, skip entirely.
		let anyHitEvents = false;
		for (let i = 0; i < world.workerCount; ++i) {
			if (world.taskContexts[i].hasHitEvents) {
				anyHitEvents = true;
				break;
			}
		}

		if (anyHitEvents) {
			// Union per-worker bits into worker 0's bit set.
			let hitEventBitSet = world.taskContexts[0].hitEventBitSet;
			for (let i = 1; i < world.workerCount; ++i) {
				if (world.taskContexts[i].hasHitEvents) {
					b3InPlaceUnion(hitEventBitSet, world.taskContexts[i].hitEventBitSet);
				}
			}

			let threshold = world.hitEventThreshold;
			let contactArray = world.contacts;
			let worldId = world.worldId;

			let wordCount = hitEventBitSet.blockCount;
			let bits = hitEventBitSet.bits;
			for (let k: u32 = 0; k < wordCount; ++k) {
				let word = bits[<i32>k];
				while (word != 0) {
					let ctz = b3CTZ64(word);
					let contactId = <i32>(64 * k + ctz);

					let contact = contactArray[contactId];
					B3_ASSERT(contact.setIndex == b3_awakeSet && contact.colorIndex != B3_NULL_INDEX);

					let shapeA = world.shapes[contact.shapeIdA];
					let shapeB = world.shapes[contact.shapeIdB];
					let simA = b3GetBodySim(world, shapeA.bodyId);
					let simB = b3GetBodySim(world, shapeB.bodyId);
					let midCenter = b3LerpPosition(simA.center, simB.center, 0.5);

					let event = b3ContactHitEvent();
					event.approachSpeed = threshold;

					let found = false;
					let triangleIndex = 0;
					let manifoldCount = contact.manifoldCount;
					for (let i = 0; i < manifoldCount; ++i) {
						ref manifold = contact.manifolds[i];
						let pointCount = manifold.pointCount;
						for (let p = 0; p < pointCount; ++p) {
							ref mp = manifold.points[p];
							let approachSpeed = -mp.normalVelocity;

							// Need to check total impulse because the point may be speculative and not colliding
							if (approachSpeed > event.approachSpeed && mp.totalNormalImpulse > 0.0) {
								event.approachSpeed = approachSpeed;
								event.point = b3OffsetPos(midCenter, b3Lerp(mp.anchorA, mp.anchorB, 0.5));
								event.normal = manifold.normal;
								triangleIndex = mp.triangleIndex;
								found = true;
							}
						}
					}

					if (found == true) {
						event.shapeIdA = b3MakeShapeId(shapeA.id + 1, worldId, shapeA.generation);
						event.shapeIdB = b3MakeShapeId(shapeB.id + 1, worldId, shapeB.generation);

						let contactIdOut = b3ContactId();
						contactIdOut.index1 = contact.contactId + 1;
						contactIdOut.world0 = worldId;
						contactIdOut.padding = 0;
						contactIdOut.generation = contact.generation;
						event.contactId = contactIdOut;

						// shapeB is never a compound today (asserted in b3CreateContact), so the
						// childIndex argument is irrelevant for it. shapeA carries the compound.
						event.userMaterialIdA = b3GetShapeUserMaterialId(shapeA, contact.childIndex, triangleIndex);
						event.userMaterialIdB = b3GetShapeUserMaterialId(shapeB, 0, triangleIndex);

						world.contactHitEvents.push(event);
					}

					// Clear the smallest set bit
					word = word & (word - 1);
				}
			}
		}

		world.profile.hitEvents = b3GetMilliseconds(hitTicks);
	}

	{
		let refitTicks = b3GetTicks();

		b3DynamicTree_Refit(world.broadPhase.trees[b3_kinematicBody]);
		b3DynamicTree_Refit(world.broadPhase.trees[b3_dynamicBody]);

		world.profile.refit = b3GetMilliseconds(refitTicks);
	}

	let bulletBodyCount = b3AtomicLoadInt(stepContext.bulletBodyCount);
	if (bulletBodyCount > 0) {
		let bulletTicks = b3GetTicks();

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		let minRange = 8;
		b3ParallelFor(
			world,
			(startIndex: i32, endIndex: i32, workerIndex: i32): void =>
				b3BulletBodyTask(startIndex, endIndex, workerIndex, stepContext),
			bulletBodyCount,
			minRange,
			"bullets"
		);

		// Serially enlarge broad-phase proxies for bullet shapes
		let broadPhase = world.broadPhase;
		let dynamicTree = broadPhase.trees[b3_dynamicBody];

		// Fast array access is important here
		let bodyArray = world.bodies;
		let bodySimArray = awakeSet.bodySims;
		let shapeArray = world.shapes;

		// Serially enlarge broad-phase proxies for bullet shapes
		let bulletBodySimIndices = stepContext.bulletBodies;

		// This loop has non-deterministic order but it shouldn't affect the result
		for (let i = 0; i < bulletBodyCount; ++i) {
			let bulletSimIndex = bulletBodySimIndices[i];
			if ((bodySimArray[bulletSimIndex].flags & b3_enlargeBulletBounds) == 0) {
				continue;
			}

			// Clear flag
			bodySimArray[bulletSimIndex].flags &= ~b3_enlargeBulletBounds;

			let bodyId = bodySimArray[bulletSimIndex].bodyId;
			B3_ASSERT(0 <= bodyId && bodyId < world.bodies.length);

			let shapeId = bodyArray[bodyId].headShapeId;
			while (shapeId != B3_NULL_INDEX) {
				let shape = shapeArray[shapeId];

				let proxyKey = shape.proxyKey;
				let proxyId = B3_PROXY_ID(proxyKey);

				let treeAABB = b3DynamicTree_GetAABB(dynamicTree, proxyId);
				let shapeFatAABB = world.fatAABBs[shapeId];

				if (b3AABB_Contains(treeAABB, shapeFatAABB) == false) {
					b3DynamicTree_EnlargeProxy(dynamicTree, proxyId, shapeFatAABB);
				}

				shapeId = shape.nextShapeId;
			}
		}

		world.profile.bullets = b3GetMilliseconds(bulletTicks);
	}

	stepContext.bulletBodies = b3_emptyIndices;
	b3AtomicStoreInt(stepContext.bulletBodyCount, 0);

	// Report sensor hits. This may include bullets sensor hits.
	{
		let sensorHitTicks = b3GetTicks();

		let workerCount = world.workerCount;
		B3_ASSERT(workerCount == world.taskContexts.length);

		for (let i = 0; i < workerCount; ++i) {
			let taskContext = world.taskContexts[i];
			let hitCount = taskContext.sensorHits.length;
			let hits = taskContext.sensorHits;

			for (let j = 0; j < hitCount; ++j) {
				let hit = hits[j];
				let sensorShape = world.shapes[hit.sensorId];
				let visitor = world.shapes[hit.visitorId];

				let sensor = world.sensors[sensorShape.sensorIndex];
				let shapeRef = b3Visitor(hit.visitorId, visitor.generation);
				sensor.hits.push(shapeRef);
			}
		}

		world.profile.sensorHits = b3GetMilliseconds(sensorHitTicks);
	}

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	// todo_erin figure out how to do this in parallel with tree refit
	if (world.enableSleep == true) {
		let sleepTicks = b3GetTicks();

		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		B3_ASSERT(world.splitIslandId == B3_NULL_INDEX);
		let splitSleepTimer: f32 = 0.0;
		for (let i = 0; i < world.workerCount; ++i) {
			let taskContext = world.taskContexts[i];
			if (taskContext.splitIslandId != B3_NULL_INDEX && taskContext.splitSleepTime >= splitSleepTimer) {
				B3_ASSERT(taskContext.splitSleepTime > 0.0);

				// Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
				if (taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < world.splitIslandId) {
					continue;
				}

				world.splitIslandId = taskContext.splitIslandId;
				splitSleepTimer = taskContext.splitSleepTime;
			}
		}

		let awakeIslandBitSet = world.taskContexts[0].awakeIslandBitSet;
		for (let i = 1; i < world.workerCount; ++i) {
			b3InPlaceUnion(awakeIslandBitSet, world.taskContexts[i].awakeIslandBitSet);
		}

		// Need to process in reverse because this moves islands to sleeping solver sets.
		let islands = awakeSet.islandSims;
		let count = islands.length;
		for (let islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1) {
			if (b3GetBit(awakeIslandBitSet, <u32>islandIndex) == true) {
				// this island is still awake
				continue;
			}

			let islandId = islands[islandIndex].islandId;

			b3TrySleepIsland(world, islandId);
		}

		world.profile.sleepIslands = b3GetMilliseconds(sleepTicks);
	}
}
