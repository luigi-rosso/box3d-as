// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/solver.c

// Solver work is partitioned into fixed-size blocks that worker threads claim
// in parallel via atomic CAS on a per-block syncIndex. The port runs one worker,
// but keeps the block and stage structure so the solve order matches C.
// The solver threading model is inspired by https://github.com/bepu/bepuphysics2

import { b3AtomicInt, b3AtomicU32 } from "./core";
import { B3_PI } from "./math_functions";
import { b3BodySim, b3BodyState } from "./body";
import { b3ContactSpec } from "./contact";
import { b3JointSim } from "./joint";
import { b3World } from "./physics_world";
import { b3ConstraintGraph } from "./constraint_graph";
import { b3ContactConstraintWide, b3ContactManifoldWide } from "./contact_solver";

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

// order is a view of the step's mesh lane order at this color's first contact.
export class b3MeshPrepareSpan {
	start: i32;
	count: i32;
	contacts: Array<b3ContactSpec>;
	order: Int32Array;

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
	states: Array<b3BodyState> = new Array<b3BodyState>();

	// shortcut to body sims from awake set
	sims: Array<b3BodySim> = new Array<b3BodySim>();

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
	wideMeshManifoldStarts: Int32Array = new Int32Array(0);
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
