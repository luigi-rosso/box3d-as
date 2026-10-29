// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Box3D's task callbacks on AnimaScript lanes. Lanes share no memory, so a task's inputs are serialized into a
// blob, its outputs into a region, and the solver's colored stages swap the velocities of the bodies two lanes
// share through one region per lane. notes/threading.md has the design and the costs.

import { Jobs, Job } from "rive/jobs";
import { Worker } from "rive/workers";
import { Blob, SharedRegion, Signal } from "rive/shared";
import { Lanes } from "rive/lanes";

import { b3GetTicks, b3GetLengthUnitsPerMeter, b3SetLengthUnitsPerMeter } from "./core";
import {
	B3_LANE_TRACE_BYTES,
	B3_MAX_LANES,
	b3_laneBackendJobs,
	b3_laneBackendParallel,
	b3_laneBackendWorkers,
	b3GrowI32,
	b3GrowU32,
	b3GrowU8,
	b3HeaderF32,
	b3HeaderI32,
	b3LaneAwait,
	b3LaneNow,
	b3LaneTraceArm,
	b3LaneTraceGather,
	b3LaneTraceOpen,
	b3LaneTraceSend,
	b3LaneWriter,
	b3MakeLaneTaskContext,
	b3ReadF32,
	b3ReadI32,
	b3ReadU32,
	b3SetLaneClock,
	b3SplitStart,
} from "./lane_common";
import { b3World_EnableParallel } from "./parallel_step";
import {
	b3LaneCollideApply,
	b3LaneCollideBegin,
	b3LaneCollideMain,
	b3LaneCollideSection,
	b3LaneCollidePartition,
	b3LaneCollideReplica,
	b3LaneCollideSupported,
} from "./lane_collide";
import { b3LaneTreeApply, b3LaneTreeJob, b3LaneTreeMain, b3LaneTreePack } from "./lane_tree";
import {
	b3SpanBegin,
	b3SpanEnd,
	b3TraceRow,
	b3TraceRowAt,
	b3_traceCollide,
	b3_traceCopy,
	b3_traceIntegratePositions,
	b3_traceIntegrateVelocities,
	b3_tracePrepare,
	b3_traceRelax,
	b3_traceRestitution,
	b3_traceSolve,
	b3_traceStore,
	b3_traceWarmStart,
} from "./trace";
import { b3EnqueueTaskCallback, b3FinishTaskCallback, b3TaskCallback } from "./types";
import { B3_GRAPH_COLOR_COUNT } from "./constants";
import { b3Vec3 } from "./math_functions";
import { b3Manifold, b3ManifoldPoint } from "./collision";
import { b3GetBit, b3SetBit, b3SetBitCountAndClear } from "./bitset";
import { b3GetIdCapacity } from "./id_pool";
import { b3ArrayData, b3BodySim, b3BodyState } from "./body";
import { b3Contact, b3ContactSpec } from "./contact";
import { b3ContactConstraintWide, b3ContactManifoldWide } from "./contact_solver";
import { B3_OVERFLOW_INDEX } from "./constraint_graph";
import {
	b3ExecuteBlock,
	b3MeshPrepareSpan,
	b3SetLaneSolve,
	b3Softness,
	b3SolverBlock,
	b3SolverStage,
	b3StepContext,
	b3WidePrepareSpan,
	b3_graphContactBlock,
	b3_graphWideContactBlock,
	b3_stageIntegratePositions,
	b3_stageIntegrateVelocities,
	b3_stagePrepareContacts,
	b3_stagePrepareWideContacts,
	b3_stageRelax,
	b3_stageRestitution,
	b3_stageSolve,
	b3_stageStoreImpulses,
	b3_stageStoreWideImpulses,
	b3_stageWarmStart,
} from "./solver";
import { b3CollideTask, b3SetLaneCollide, b3World, b3GetWorldFromId } from "./physics_world";
import { b3WorldId } from "./types";

export { B3_MAX_LANES, b3_laneBackendJobs, b3_laneBackendParallel, b3_laneBackendWorkers } from "./lane_common";

// Which tasks b3World_EnableLanes moves to lanes
export const b3_laneCollide: i32 = 1;
export const b3_laneSolve: i32 = 2;
export const b3_laneTree: i32 = 4;

// Worker commands, in the control region's command word
const CMD_SOLVE: u32 = 1;
const CMD_COLLIDE: u32 = 2;
const CMD_QUIT: u32 = 3;

// Control region words, by byte offset. A command's serial goes out before its sections are packed, so the lanes
// wake meanwhile; each lane's section blob follows with its ready word set to the serial.
const C_SERIAL: u32 = 0;
const C_COMMAND: u32 = 4;
const C_LANES: u32 = 12;
const C_SNAPSHOT: u32 = 16;
const C_OUTPUTS: u32 = 64;
const C_INPUTS: u32 = 96;
const C_READY: u32 = 128;
const C_PAIRS: u32 = 192;
const C_TRACES: u32 = C_PAIRS + 4 * 64;
const C_BYTES: u32 = C_TRACES + 4 * <u32>B3_MAX_LANES;

// Pair region words, by byte offset. Lane p appends what it sends lane q in a step to pair p * lanes + q, with
// each exchange's offset and count in a table, so a pair's lock has one writer and one reader and is never
// contended, and a lane waits only for the exchanges that bring it data. Pair p * lanes + p holds lane p's done
// word.
const X_EPOCH: u32 = 0;
const X_DONE: u32 = 12;
// The id of the blob a worker's collide answered
const X_RESULT: u32 = 8;
const X_TABLE: u32 = 16;
const X_TABLE_ENTRIES: i32 = 1024;
const X_DATA: u32 = X_TABLE + 8 * <u32>X_TABLE_ENTRIES;

// Epochs of one step's exchanges start at the step serial times this; the last one marks a partition done
const EPOCH_STRIDE: u32 = 4096;

// Common solve header words, after the section table
const H_AWAKE: i32 = 0;
const H_COLORS: i32 = 1;
const H_SUBSTEPS: i32 = 2;
const H_DT: i32 = 3;
const H_INV_DT: i32 = 4;
const H_H: i32 = 5;
const H_INV_H: i32 = 6;
const H_SOFT: i32 = 7;
const H_STATIC_SOFT: i32 = 10;
const H_MAX_LINEAR_VELOCITY: i32 = 13;
const H_WARM_STARTING: i32 = 14;
const H_RESTITUTION_ITERATIONS: i32 = 15;
const H_RESTITUTION_THRESHOLD: i32 = 16;
const H_PROPAGATE: i32 = 17;
const H_CONTACT_SPEED: i32 = 18;
const H_HIT_THRESHOLD: i32 = 19;
const H_GRAVITY: i32 = 20;
const H_CONTACT_CAPACITY: i32 = 23;
const H_ANY_RESTITUTION: i32 = 24;
const H_EPOCH_BASE: i32 = 25;
const H_LENGTH_UNITS: i32 = 26;
const H_LANES: i32 = 27;
const H_NEIGHBORS: i32 = 28;
const H_COLOR_INDICES: i32 = H_NEIGHBORS + B3_MAX_LANES;
// Per receiving lane and active color, the lanes that send it velocities
const H_NEEDS: i32 = H_COLOR_INDICES + B3_GRAPH_COLOR_COUNT;
const H_WORDS: i32 = H_NEEDS + B3_MAX_LANES * B3_GRAPH_COLOR_COUNT;

// Per contact record words before its manifolds
const R_WORDS: i32 = 11;
// Per body record words before its sim and state
const B_WORDS: i32 = 2;
// Exchange entry: awake index, then linear and angular velocity bits
const E_WORDS: i32 = 7;
// What the solver writes into a manifold: friction, twist and rolling impulses, then per point the normal and
// total normal impulses and the normal velocity
const I_MANIFOLD_WORDS: i32 = 7;
const I_POINT_WORDS: i32 = 3;

// Partition timings at the head of each output, ms
export const T_SETUP: i32 = 0;
export const T_PREPARE: i32 = 1;
export const T_INTEGRATE_VELOCITIES: i32 = 2;
export const T_WARM_START: i32 = 3;
export const T_SOLVE: i32 = 4;
export const T_INTEGRATE_POSITIONS: i32 = 5;
export const T_RELAX: i32 = 6;
export const T_RESTITUTION: i32 = 7;
export const T_STORE: i32 = 8;
export const T_EXCHANGE: i32 = 9;
export const T_FIRST_WAIT: i32 = 10;
export const T_OUTPUT: i32 = 11;
export const T_COUNT: i32 = 12;

// Output head: byte length, partition, wait spins in millions, then nothing else; the clock is the main lane's
const O_WORDS: i32 = 4;

// Largest block the u16 block count holds, rounded down to a power of two
const BLOCK_LIMIT: i32 = 32768;

// Atomic loads a worker spins through before it parks on the wake signal: briefly after a command, longer after a
// wake, which the main lane sends before it serializes the step so the wake hides behind that work
const IDLE_SPINS: i32 = 20000;
const WOKEN_SPINS: i32 = 2000000;

// What one partition runs: its slice of each color's constraints and its bodies, over some step context. The main
// lane's partition runs over the world's own arrays; the other lanes' over a replica's compact ones.
export class b3LanePartition {
	context: b3StepContext;
	stage: b3SolverStage = new b3SolverStage();
	activeColorCount: i32 = 0;
	colorIndices: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	// Per active color: the first wide constraint relative to the color, the count, and the first flat index
	wideFirst: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	wideCount: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	wideFlat: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	groupFirst: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	groupCount: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	groupFlat: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	// Consecutive awake index runs of the bodies, the body blocks of the integration stages
	runs: Int32Array = new Int32Array(0);
	runCount: i32 = 0;
	// Per color, the shared bodies this partition's constraints write
	sharedStarts: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT + 1);
	shared: Int32Array = new Int32Array(0);
	// By awake index, the lanes that touch each body this partition touches
	maskOf: Uint32Array = new Uint32Array(0);
	sendBuffer: Uint32Array = new Uint32Array(0);
	receiveBuffer: Uint32Array = new Uint32Array(0);
	times: Float64Array = new Float64Array(T_COUNT);
	partition: i32 = 0;
	lanes: i32 = 1;
	neighbors: u32 = 0;
	epoch: u32 = 0;
	epochBase: u32 = 0;
	// Per receiving lane and active color, the lanes that send it velocities; and per receiving lane, the entries
	// appended to its pair so far this step
	needs: Uint32Array = new Uint32Array(B3_MAX_LANES * B3_GRAPH_COLOR_COUNT);
	appended: Int32Array = new Int32Array(B3_MAX_LANES);
	spins: u64 = 0;
	exchangesWaited: i32 = 0;
	trace: b3TraceRow? = null;

	constructor(context: b3StepContext) {
		this.context = context;
	}
}

// A lane's copy of what its partition's stages read and write. A worker keeps it from step to step; a job builds
// it per run.
export class b3LaneSolveReplica {
	world: b3World = new b3World();
	part: b3LanePartition;
	dummy: b3Contact = new b3Contact();
	header: Uint32Array = new Uint32Array(H_WORDS);
	input: Uint8Array = new Uint8Array(0);
	output: Uint8Array = new Uint8Array(0);
	wideConstraints: StaticArray<b3ContactConstraintWide> = new StaticArray<b3ContactConstraintWide>(0);
	wideManifolds: StaticArray<b3ContactManifoldWide> = new StaticArray<b3ContactManifoldWide>(0);
	meshConstraints: StaticArray<b3ContactConstraintWide> = new StaticArray<b3ContactConstraintWide>(0);
	meshManifolds: StaticArray<b3ContactManifoldWide> = new StaticArray<b3ContactManifoldWide>(0);
	meshStarts: Int32Array = new Int32Array(0);
	meshOrder: Int32Array = new Int32Array(0);
	widePrepareSpans: StaticArray<b3WidePrepareSpan> = new StaticArray<b3WidePrepareSpan>(B3_GRAPH_COLOR_COUNT + 1);
	meshPrepareSpans: StaticArray<b3MeshPrepareSpan> = new StaticArray<b3MeshPrepareSpan>(B3_GRAPH_COLOR_COUNT + 1);
	colorWideLanes: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	colorGroupLanes: Int32Array = new Int32Array(B3_GRAPH_COLOR_COUNT);
	// Byte offset of each contact record in the section, in section order
	records: Int32Array = new Int32Array(0);
	contactCount: i32 = 0;
	// The partition's bodies, sorted by awake index, and the lanes that touch each by awake index
	bodies: Int32Array = new Int32Array(0);
	bodyCount: i32 = 0;

	constructor() {
		this.part = new b3LanePartition(new b3StepContext(this.world));
		for (let i = 0; i <= B3_GRAPH_COLOR_COUNT; ++i) {
			this.widePrepareSpans[i] = new b3WidePrepareSpan(0, 0, new Array<i32>());
			this.meshPrepareSpans[i] = new b3MeshPrepareSpan(0, 0, new Array<b3ContactSpec>(), this.meshOrder);
		}
		this.world.taskContexts.push(b3MakeLaneTaskContext());
	}
}

function b3ReplicaContact(r: b3LaneSolveReplica, contactId: i32): b3Contact {
	let contacts = r.world.contacts;
	while (contacts.length <= contactId) {
		contacts.push(r.dummy);
	}
	let contact = contacts[contactId];
	if (contact == r.dummy) {
		contact = new b3Contact();
		contacts[contactId] = contact;
	}
	return contact;
}

// Runs of consecutive awake indices among sorted bodies, the integration stages' body blocks
function b3SetRuns(part: b3LanePartition, bodies: Int32Array, bodyCount: i32): void {
	part.runs = b3GrowI32(part.runs, 2 * bodyCount);
	let runCount = 0;
	for (let k = 0; k < bodyCount; ) {
		let start = bodies[k];
		let n = 1;
		while (k + n < bodyCount && bodies[k + n] == start + n && n < BLOCK_LIMIT) {
			n += 1;
		}
		part.runs[2 * runCount] = start;
		part.runs[2 * runCount + 1] = n;
		runCount += 1;
		k += n;
	}
	part.runCount = runCount;
}

// Reads a section into the replica and points its step context at compact arrays. Each color's slice of
// constraints keeps the lanes the main lane's layout gives it, so the kernels compute the same bits.
unsafe function b3LaneSolveSetup(r: b3LaneSolveReplica): void {
	let world = r.world;
	let part = r.part;
	let context = part.context;
	let h = r.header;
	let base = r.input.dataStart;

	let awakeCount = b3HeaderI32(h, H_AWAKE);
	let activeColorCount = b3HeaderI32(h, H_COLORS);
	context.subStepCount = b3HeaderI32(h, H_SUBSTEPS);
	context.dt = b3HeaderF32(h, H_DT);
	context.inv_dt = b3HeaderF32(h, H_INV_DT);
	context.h = b3HeaderF32(h, H_H);
	context.inv_h = b3HeaderF32(h, H_INV_H);
	context.contactSoftness = b3Softness(b3HeaderF32(h, H_SOFT), b3HeaderF32(h, H_SOFT + 1), b3HeaderF32(h, H_SOFT + 2));
	context.staticSoftness = b3Softness(
		b3HeaderF32(h, H_STATIC_SOFT),
		b3HeaderF32(h, H_STATIC_SOFT + 1),
		b3HeaderF32(h, H_STATIC_SOFT + 2)
	);
	context.maxLinearVelocity = b3HeaderF32(h, H_MAX_LINEAR_VELOCITY);
	context.enableWarmStarting = b3HeaderI32(h, H_WARM_STARTING) != 0;
	world.enableWarmStarting = context.enableWarmStarting;
	world.restitutionIterations = b3HeaderI32(h, H_RESTITUTION_ITERATIONS);
	world.restitutionThreshold = b3HeaderF32(h, H_RESTITUTION_THRESHOLD);
	world.enableRestitutionPropagation = b3HeaderI32(h, H_PROPAGATE) != 0;
	world.contactSpeed = b3HeaderF32(h, H_CONTACT_SPEED);
	world.hitEventThreshold = b3HeaderF32(h, H_HIT_THRESHOLD);
	world.gravity = b3Vec3(b3HeaderF32(h, H_GRAVITY), b3HeaderF32(h, H_GRAVITY + 1), b3HeaderF32(h, H_GRAVITY + 2));
	b3SetLengthUnitsPerMeter(b3HeaderF32(h, H_LENGTH_UNITS));
	context.activeColorCount = activeColorCount;
	context.anyRestitution.value = b3HeaderI32(h, H_ANY_RESTITUTION);
	part.activeColorCount = activeColorCount;
	part.lanes = b3HeaderI32(h, H_LANES);
	part.neighbors = <u32>b3HeaderI32(h, H_NEIGHBORS + part.partition);
	part.epochBase = <u32>b3HeaderI32(h, H_EPOCH_BASE);
	for (let k = 0; k < B3_MAX_LANES * B3_GRAPH_COLOR_COUNT; ++k) {
		part.needs[k] = <u32>b3HeaderI32(h, H_NEEDS + k);
	}

	let taskContext = world.taskContexts[0];
	b3SetBitCountAndClear(taskContext.hitEventBitSet, <u32>b3HeaderI32(h, H_CONTACT_CAPACITY));
	taskContext.hasHitEvents = false;

	// Section: counts, then the color table
	let contactCount = b3ReadI32(base, 0);
	let bodyCount = b3ReadI32(base, 4);
	r.contactCount = contactCount;
	r.bodyCount = bodyCount;
	let at = 8;
	let wideTotal = 0;
	let groupTotal = 0;
	let meshLaneTotal = 0;
	for (let i = 0; i < activeColorCount; ++i) {
		part.colorIndices[i] = b3HeaderI32(h, H_COLOR_INDICES + i);
		let wideCount = b3ReadI32(base, at);
		let groups = b3ReadI32(base, at + 8);
		part.wideFirst[i] = 0;
		part.wideCount[i] = wideCount;
		part.wideFlat[i] = wideTotal;
		part.groupFirst[i] = 0;
		part.groupCount[i] = groups;
		part.groupFlat[i] = groupTotal;
		r.colorWideLanes[i] = b3ReadI32(base, at + 4);
		r.colorGroupLanes[i] = b3ReadI32(base, at + 12);
		at += 16;
		wideTotal += wideCount;
		groupTotal += groups;
		meshLaneTotal += r.colorGroupLanes[i];
	}

	// Contacts into the replica's contact slots
	r.records = b3GrowI32(r.records, contactCount);
	let manifoldSize = <i32>sizeof<b3Manifold>();
	for (let k = 0; k < contactCount; ++k) {
		r.records[k] = at;
		let contactId = b3ReadI32(base, at);
		let manifoldCount = b3ReadI32(base, at + 16);
		let contact = b3ReplicaContact(r, contactId);
		contact.contactId = contactId;
		contact.flags = b3ReadU32(base, at + 4);
		contact.encodedBodySimA = b3ReadI32(base, at + 8);
		contact.encodedBodySimB = b3ReadI32(base, at + 12);
		contact.friction = b3ReadF32(base, at + 20);
		contact.restitution = b3ReadF32(base, at + 24);
		contact.rollingResistance = b3ReadF32(base, at + 28);
		contact.tangentVelocity = b3Vec3(b3ReadF32(base, at + 32), b3ReadF32(base, at + 36), b3ReadF32(base, at + 40));
		if (contact.manifolds.length < manifoldCount) {
			contact.manifolds = new StaticArray<b3Manifold>(manifoldCount);
		}
		contact.manifoldCount = manifoldCount;
		at += R_WORDS * 4;
		let bytes = manifoldCount * manifoldSize;
		memory.copy(changetype<usize>(contact.manifolds), base + <usize>at, <usize>bytes);
		at += bytes;
	}

	// Bodies into their awake slots
	if (context.sims.length < awakeCount) {
		context.sims.length = awakeCount;
	}
	if (context.states.length < awakeCount) {
		context.states.length = awakeCount;
	}
	part.maskOf = b3GrowU32(part.maskOf, awakeCount);
	r.bodies = b3GrowI32(r.bodies, bodyCount);
	let sims = b3ArrayData(context.sims);
	let states = b3ArrayData(context.states);
	let simSize = <i32>sizeof<b3BodySim>();
	let stateSize = <i32>sizeof<b3BodyState>();
	for (let k = 0; k < bodyCount; ++k) {
		let index = b3ReadI32(base, at);
		r.bodies[k] = index;
		part.maskOf[index] = b3ReadU32(base, at + 4);
		at += B_WORDS * 4;
		memory.copy(sims + <usize>index * <usize>simSize, base + <usize>at, <usize>simSize);
		at += simSize;
		memory.copy(states + <usize>index * <usize>stateSize, base + <usize>at, <usize>stateSize);
		at += stateSize;
	}
	b3SetRuns(part, r.bodies, bodyCount);

	// Compact constraint arrays, and the spans and color slices over them
	if (r.wideConstraints.length < wideTotal) {
		let capacity = wideTotal + (wideTotal >> 2) + 4;
		r.wideConstraints = new StaticArray<b3ContactConstraintWide>(capacity);
		r.wideManifolds = new StaticArray<b3ContactManifoldWide>(capacity);
	}
	if (r.meshConstraints.length < groupTotal) {
		r.meshConstraints = new StaticArray<b3ContactConstraintWide>(groupTotal + (groupTotal >> 2) + 4);
	}
	r.meshStarts = b3GrowI32(r.meshStarts, groupTotal + 1);
	if (r.meshOrder.length < meshLaneTotal) {
		r.meshOrder = new Int32Array(meshLaneTotal + (meshLaneTotal >> 2) + 16);
		for (let i = 0; i < r.meshOrder.length; ++i) {
			r.meshOrder[i] = i;
		}
	}

	let graph = world.constraintGraph;
	let slotCount = 0;
	let record = 0;
	for (let i = 0; i < activeColorCount; ++i) {
		let color = graph.colors[part.colorIndices[i]];
		let wideLanes = r.colorWideLanes[i];

		let span = r.widePrepareSpans[i];
		span.start = part.wideFlat[i];
		span.count = wideLanes;
		let ids = span.contacts;
		ids.length = wideLanes;
		for (let k = 0; k < wideLanes; ++k) {
			ids[k] = b3ReadI32(base, r.records[record + k]);
		}
		record += wideLanes;

		color.wideConstraints = r.wideConstraints;
		color.wideManifolds = r.wideManifolds;
		color.wideConstraintStart = part.wideFlat[i];
		color.wideConstraintCount = part.wideCount[i];

		let groups = part.groupCount[i];
		let groupBase = part.groupFlat[i];
		let groupLanes = r.colorGroupLanes[i];
		let meshSpan = r.meshPrepareSpans[i];
		meshSpan.start = groupBase;
		meshSpan.count = groupLanes;
		// The lanes arrive in lane order, so the order is the identity within each color
		meshSpan.order = r.meshOrder;
		meshSpan.orderBase = 0;
		let specs = meshSpan.contacts;
		specs.length = groupLanes;
		for (let k = 0; k < groupLanes; ++k) {
			let spec = b3ContactSpec();
			spec.contactId = b3ReadI32(base, r.records[record + k]);
			spec.manifoldCount = <u16>b3ReadI32(base, r.records[record + k] + 16);
			specs[k] = spec;
		}
		// Lanes arrive sorted by manifold count, so a group's first lane has its slot count
		for (let g = 0; g < groups; ++g) {
			r.meshStarts[groupBase + g] = slotCount;
			slotCount += <i32>specs[4 * g].manifoldCount;
		}
		record += groupLanes;
		color.wideMeshConstraints = r.meshConstraints;
		color.wideMeshConstraintStart = groupBase;
		color.wideMeshConstraintCount = groups;
	}
	r.meshStarts[groupTotal] = slotCount;
	if (r.meshManifolds.length < slotCount) {
		r.meshManifolds = new StaticArray<b3ContactManifoldWide>(slotCount + (slotCount >> 2) + 4);
	}
	for (let i = 0; i < activeColorCount; ++i) {
		let color = graph.colors[part.colorIndices[i]];
		color.wideMeshManifolds = r.meshManifolds;
		color.wideMeshManifoldStarts = r.meshStarts.subarray(color.wideMeshConstraintStart);
	}

	let sentinel = r.widePrepareSpans[activeColorCount];
	sentinel.start = wideTotal;
	sentinel.count = 0;
	let meshSentinel = r.meshPrepareSpans[activeColorCount];
	meshSentinel.start = groupTotal;
	meshSentinel.count = 0;

	context.wideConstraints = r.wideConstraints;
	context.wideManifolds = r.wideManifolds;
	context.widePrepareSpans = r.widePrepareSpans;
	context.wideContactCount = wideTotal;
	context.wideMeshConstraints = r.meshConstraints;
	context.wideMeshManifolds = r.meshManifolds;
	context.wideMeshManifoldStarts = r.meshStarts;
	context.meshPrepareSpans = r.meshPrepareSpans;
	context.graph = graph;
	context.workerCount = 1;

	// Per color, the bodies this partition's constraints write that another lane also touches
	part.shared = b3GrowI32(part.shared, 2 * contactCount);
	let sharedCount = 0;
	record = 0;
	for (let i = 0; i < activeColorCount; ++i) {
		part.sharedStarts[i] = sharedCount;
		let lanes = r.colorWideLanes[i] + r.colorGroupLanes[i];
		for (let k = 0; k < lanes; ++k) {
			let rec = r.records[record + k];
			let a = b3ReadI32(base, rec + 8);
			let b = b3ReadI32(base, rec + 12);
			if (a >= 0 && popcnt(part.maskOf[a]) > 1) {
				part.shared[sharedCount++] = a;
			}
			if (b >= 0 && popcnt(part.maskOf[b]) > 1) {
				part.shared[sharedCount++] = b;
			}
		}
		record += lanes;
	}
	part.sharedStarts[activeColorCount] = sharedCount;
	part.sendBuffer = b3GrowU32(part.sendBuffer, E_WORDS * (sharedCount + 1));
}

// Runs a stage over [start, start + count) in blocks the u16 block count can hold
function b3RunBlocks(part: b3LanePartition, type: i32, blockType: i32, colorIndex: i32, start: i32, count: i32): void {
	let stage = part.stage;
	stage.type = type;
	let block = b3SolverBlock();
	block.blockType = <u8>blockType;
	block.colorIndex = <u8>colorIndex;
	let end = start + count;
	for (let i = start; i < end; i += BLOCK_LIMIT) {
		block.startIndex = i;
		block.count = <u16>min(BLOCK_LIMIT, end - i);
		b3ExecuteBlock(stage, part.context, block, 0);
	}
}

function b3RunBodies(part: b3LanePartition, type: i32): void {
	for (let k = 0; k < part.runCount; ++k) {
		b3RunBlocks(part, type, 0, 0, part.runs[2 * k], part.runs[2 * k + 1]);
	}
}

// One color of a colored stage, then the exchange of the shared bodies it wrote
function b3RunColor(part: b3LanePartition, exchanges: Array<SharedRegion>, type: i32, i: i32): void {
	let colorIndex = part.colorIndices[i];
	b3RunBlocks(part, type, b3_graphWideContactBlock, colorIndex, part.wideFirst[i], part.wideCount[i]);
	b3RunBlocks(part, type, b3_graphContactBlock, colorIndex, part.groupFirst[i], part.groupCount[i]);
	b3LaneExchange(part, exchanges, i);
}

@inline unsafe function b3CopyVelocities(dst: usize, src: usize): void {
	let v = offsetof<b3BodyState>("linearVelocity");
	let w = offsetof<b3BodyState>("angularVelocity");
	store<u32>(dst + v, load<u32>(src));
	store<u32>(dst + v + 4, load<u32>(src + 4));
	store<u32>(dst + v + 8, load<u32>(src + 8));
	store<u32>(dst + w, load<u32>(src + 12));
	store<u32>(dst + w + 4, load<u32>(src + 16));
	store<u32>(dst + w + 8, load<u32>(src + 20));
}

// Publishes this partition's writes to shared bodies, to each lane that touches them, then waits for the lanes that
// send it velocities in this color and applies them. Within a step nothing in a pair is written twice, and the
// next step starts only once every lane is done.
unsafe function b3LaneExchange(part: b3LanePartition, exchanges: Array<SharedRegion>, colorSlot: i32): void {
	part.epoch += 1;
	if (part.neighbors == 0) {
		return;
	}

	let ticks = b3LaneNow();
	let epoch = part.epoch;
	let tableOffset = X_TABLE + 8 * (epoch - part.epochBase);
	let states = b3ArrayData(part.context.states);
	let stateSize = sizeof<b3BodyState>();
	let v = offsetof<b3BodyState>("linearVelocity");
	let w = offsetof<b3BodyState>("angularVelocity");
	let lanes = part.lanes;
	let p = part.partition;
	let pBit: u32 = 1 << <u32>p;
	let maskOf = part.maskOf.dataStart;
	let needs = part.needs;

	let start = part.sharedStarts[colorSlot];
	let end = part.sharedStarts[colorSlot + 1];
	let send = part.sendBuffer;
	let out = send.dataStart;
	for (let q = 0; q < lanes; ++q) {
		if ((needs[q * B3_GRAPH_COLOR_COUNT + colorSlot] & pBit) == 0) {
			continue;
		}
		let bit: u32 = 1 << <u32>q;
		let count = 0;
		for (let k = start; k < end; ++k) {
			let index = part.shared[k];
			if ((load<u32>(maskOf + <usize>index * 4) & bit) == 0) {
				continue;
			}
			let state = states + <usize>index * stateSize;
			let e = out + <usize>(E_WORDS * count) * 4;
			store<i32>(e, index);
			store<u32>(e + 4, load<u32>(state + v));
			store<u32>(e + 8, load<u32>(state + v + 4));
			store<u32>(e + 12, load<u32>(state + v + 8));
			store<u32>(e + 16, load<u32>(state + w));
			store<u32>(e + 20, load<u32>(state + w + 4));
			store<u32>(e + 24, load<u32>(state + w + 8));
			count += 1;
		}
		let pair = exchanges[p * lanes + q];
		let offset = part.appended[q];
		let words = E_WORDS * count;
		let dataOffset = X_DATA + <u32>(offset * E_WORDS * 4);
		pair.exclusive((view) => {
			view.writeFrom(dataOffset, send, 0, words);
		});
		part.appended[q] = offset + count;
		pair.atomicStore(tableOffset, <u32>offset);
		pair.atomicStore(tableOffset + 4, <u32>count);
		pair.atomicStore(X_EPOCH, epoch);
	}

	let senders = needs[p * B3_GRAPH_COLOR_COUNT + colorSlot];
	for (let q = 0; q < lanes; ++q) {
		if ((senders & (1 << <u32>q)) == 0) {
			continue;
		}
		let other = exchanges[q * lanes + p];
		let waitTicks = b3LaneNow();
		part.spins += b3LaneAwait(other, X_EPOCH, epoch);
		if (part.exchangesWaited == 0) {
			part.times[T_FIRST_WAIT] += b3LaneNow() - waitTicks;
		}
		part.exchangesWaited += 1;
		let offset = other.atomicLoad(tableOffset);
		let received = <i32>other.atomicLoad(tableOffset + 4);
		let words = E_WORDS * received;
		part.receiveBuffer = b3GrowU32(part.receiveBuffer, words);
		let receive = part.receiveBuffer;
		let dataOffset = X_DATA + offset * <u32>(E_WORDS * 4);
		other.exclusive((view) => {
			view.readInto(dataOffset, receive, 0, words);
		});
		let input = receive.dataStart;
		for (let k = 0; k < received; ++k) {
			let e = input + <usize>(E_WORDS * k) * 4;
			b3CopyVelocities(states + <usize>load<i32>(e) * stateSize, e + 4);
		}
	}
	part.times[T_EXCHANGE] += b3LaneNow() - ticks;
}

// b3SolverTask's stage order without joints or the overflow color
function b3LaneSolveStages(part: b3LanePartition, exchanges: Array<SharedRegion>): void {
	let times = part.times;
	let trace = part.trace;
	let colorCount = part.activeColorCount;
	part.epoch = part.epochBase;
	part.appended.fill(0);
	let ticks = b3LaneNow();
	let traceBegin = b3SpanBegin(trace);

	for (let i = 0; i < colorCount; ++i) {
		b3RunBlocks(part, b3_stagePrepareWideContacts, 0, 0, part.wideFlat[i], part.wideCount[i]);
		b3RunBlocks(part, b3_stagePrepareContacts, 0, 0, part.groupFlat[i], part.groupCount[i]);
	}
	times[T_PREPARE] += b3LaneNow() - ticks;
	b3SpanEnd(trace, b3_tracePrepare, traceBegin);

	let subStepCount = part.context.subStepCount;
	for (let subStep = 0; subStep < subStepCount; ++subStep) {
		ticks = b3LaneNow();
		traceBegin = b3SpanBegin(trace);
		b3RunBodies(part, b3_stageIntegrateVelocities);
		times[T_INTEGRATE_VELOCITIES] += b3LaneNow() - ticks;
		b3SpanEnd(trace, b3_traceIntegrateVelocities, traceBegin);

		ticks = b3LaneNow();
		traceBegin = b3SpanBegin(trace);
		for (let i = 0; i < colorCount; ++i) {
			b3RunColor(part, exchanges, b3_stageWarmStart, i);
		}
		times[T_WARM_START] += b3LaneNow() - ticks;
		b3SpanEnd(trace, b3_traceWarmStart, traceBegin);

		ticks = b3LaneNow();
		traceBegin = b3SpanBegin(trace);
		for (let i = 0; i < colorCount; ++i) {
			b3RunColor(part, exchanges, b3_stageSolve, i);
		}
		times[T_SOLVE] += b3LaneNow() - ticks;
		b3SpanEnd(trace, b3_traceSolve, traceBegin);

		ticks = b3LaneNow();
		traceBegin = b3SpanBegin(trace);
		b3RunBodies(part, b3_stageIntegratePositions);
		times[T_INTEGRATE_POSITIONS] += b3LaneNow() - ticks;
		b3SpanEnd(trace, b3_traceIntegratePositions, traceBegin);

		ticks = b3LaneNow();
		traceBegin = b3SpanBegin(trace);
		for (let i = 0; i < colorCount; ++i) {
			b3RunColor(part, exchanges, b3_stageRelax, i);
		}
		times[T_RELAX] += b3LaneNow() - ticks;
		b3SpanEnd(trace, b3_traceRelax, traceBegin);
	}

	let world = part.context.world;
	let restitutionIterations = world.restitutionIterations;
	if (restitutionIterations > 0 && part.context.anyRestitution.value != 0) {
		ticks = b3LaneNow();
		traceBegin = b3SpanBegin(trace);
		for (let j = 0; j < restitutionIterations; ++j) {
			for (let i = 0; i < colorCount; ++i) {
				b3RunColor(part, exchanges, b3_stageRestitution, i);
			}
		}
		times[T_RESTITUTION] += b3LaneNow() - ticks;
		b3SpanEnd(trace, b3_traceRestitution, traceBegin);
	}

	ticks = b3LaneNow();
	traceBegin = b3SpanBegin(trace);
	for (let i = 0; i < colorCount; ++i) {
		b3RunBlocks(part, b3_stageStoreWideImpulses, 0, 0, part.wideFlat[i], part.wideCount[i]);
		b3RunBlocks(part, b3_stageStoreImpulses, 0, 0, part.groupFlat[i], part.groupCount[i]);
	}
	times[T_STORE] += b3LaneNow() - ticks;
	b3SpanEnd(trace, b3_traceStore, traceBegin);
}

@inline function b3ImpulseBytes(manifolds: StaticArray<b3Manifold>, manifoldCount: i32): i32 {
	let words = 1;
	for (let m = 0; m < manifoldCount; ++m) {
		words += I_MANIFOLD_WORDS + I_POINT_WORDS * manifolds[m].pointCount;
	}
	return 4 * words;
}

// Output: byte length, partition, wait spins, then per contact its hit bit and what the solver wrote into its
// manifolds, then the states of the bodies whose lowest touching lane is this one
unsafe function b3LaneSolveOutput(r: b3LaneSolveReplica, output: SharedRegion): void {
	let part = r.part;
	let stateSize = <i32>sizeof<b3BodyState>();
	let contacts = r.world.contacts;
	let base = r.input.dataStart;
	let lane = part.partition;
	let bytes = 4 * O_WORDS;
	for (let k = 0; k < r.contactCount; ++k) {
		let contact = contacts[b3ReadI32(base, r.records[k])];
		bytes += b3ImpulseBytes(contact.manifolds, contact.manifoldCount);
	}
	bytes += stateSize * r.bodyCount;
	r.output = b3GrowU8(r.output, bytes);

	let out = r.output.dataStart;
	let at = out + <usize>(4 * O_WORDS);
	let hits = r.world.taskContexts[0].hitEventBitSet;
	let pointSize = sizeof<b3ManifoldPoint>();
	for (let k = 0; k < r.contactCount; ++k) {
		let contactId = b3ReadI32(base, r.records[k]);
		let contact = contacts[contactId];
		store<u32>(at, b3GetBit(hits, <u32>contactId) ? 1 : 0);
		at += 4;
		let manifolds = changetype<usize>(contact.manifolds);
		for (let m = 0; m < contact.manifoldCount; ++m) {
			let manifold = manifolds + <usize>m * sizeof<b3Manifold>();
			memory.copy(at, manifold + offsetof<b3Manifold>("twistImpulse"), 28);
			at += 28;
			let pointCount = load<i32>(manifold + offsetof<b3Manifold>("pointCount"));
			for (let j = 0; j < pointCount; ++j) {
				let point = manifold + offsetof<b3Manifold>("points") + <usize>j * pointSize;
				store<u32>(at, load<u32>(point + offsetof<b3ManifoldPoint>("normalImpulse")));
				store<u32>(at + 4, load<u32>(point + offsetof<b3ManifoldPoint>("totalNormalImpulse")));
				store<u32>(at + 8, load<u32>(point + offsetof<b3ManifoldPoint>("normalVelocity")));
				at += 12;
			}
		}
	}
	let states = b3ArrayData(part.context.states);
	for (let k = 0; k < r.bodyCount; ++k) {
		let index = r.bodies[k];
		if (<i32>ctz(part.maskOf[index]) != lane) {
			continue;
		}
		memory.copy(at, states + <usize>index * <usize>stateSize, <usize>stateSize);
		at += <usize>stateSize;
	}

	let length = <i32>(at - out);
	store<u32>(out, <u32>length);
	store<u32>(out + 4, <u32>lane);
	store<u32>(out + 8, <u32>(part.spins / 1000));
	store<u32>(out + 12, <u32>part.exchangesWaited);
	let buffer = r.output;
	output.exclusive((view) => {
		view.writeFrom(0, buffer, 0, length);
	});
}

// Everything a lane's partition does in a step, the same on a job and a worker
export unsafe function b3LaneSolvePartition(
	r: b3LaneSolveReplica,
	input: Blob,
	output: SharedRegion,
	exchanges: Array<SharedRegion>,
	partition: i32,
	traceRegion: SharedRegion
): void {
	let part = r.part;
	part.partition = partition;
	part.spins = 0;
	part.exchangesWaited = 0;
	for (let i = 0; i < T_COUNT; ++i) {
		part.times[i] = 0;
	}
	let trace = b3LaneTraceOpen(traceRegion);
	part.trace = trace;
	let traceBegin = b3SpanBegin(trace);
	let ticks = b3LaneNow();
	// The section blob: the common header, then this partition's section
	input.readInto(0, r.header);
	let length = <i32>input.byteLength - 4 * H_WORDS;
	r.input = b3GrowU8(r.input, length);
	input.readInto(<u32>(4 * H_WORDS), r.input, 0, length);
	b3LaneSolveSetup(r);
	part.times[T_SETUP] = b3LaneNow() - ticks;
	b3SpanEnd(trace, b3_traceCopy, traceBegin);
	b3LaneSolveStages(part, exchanges);
	ticks = b3LaneNow();
	traceBegin = b3SpanBegin(trace);
	b3LaneSolveOutput(r, output);
	part.times[T_OUTPUT] = b3LaneNow() - ticks;
	b3SpanEnd(trace, b3_traceCopy, traceBegin);
	b3LaneTraceSend(traceRegion, trace);
	exchanges[partition * part.lanes + partition].atomicStore(X_DONE, part.epochBase | (EPOCH_STRIDE - 1));
}

@job export function b3LaneSolveJob(
	input: Blob,
	output: SharedRegion,
	exchanges: Array<SharedRegion>,
	partition: i32,
	trace: SharedRegion,
): i32 {
	let replica = new b3LaneSolveReplica();
	b3LaneSolvePartition(replica, input, output, exchanges, partition, trace);
	return partition;
}

@job export function b3LaneCollideJob(snapshot: Blob, input: Blob, partition: i32, trace: SharedRegion): Blob {
	let replica = new b3LaneCollideReplica();
	return b3LaneCollidePartition(replica, snapshot, input, partition, trace);
}

// A worker per extra lane. It spins on the control region's serial between commands, briefly, then sleeps.
@worker export async function b3LaneWorker(control: SharedRegion, wake: Signal, partition: i32): Promise<void> {
	let replica = new b3LaneSolveReplica();
	let collideReplica = new b3LaneCollideReplica();
	let exchanges = new Array<SharedRegion>();
	// The blob the last collide answered, held until the main lane has taken its own hold, by the next command
	let result: Blob? = null;
	// The serial starts at 0 and every command waits for every partition, so a late start misses nothing
	let seen: u32 = 0;
	let spins = IDLE_SPINS;
	while (true) {
		let serial = control.atomicLoad(C_SERIAL);
		for (let spin = 0; serial == seen && spin < spins; ++spin) {
			serial = control.atomicLoad(C_SERIAL);
		}
		if (serial == seen) {
			// The sequence is read before the last look at the serial, so a command posted after it wakes this
			let wakeSeq = wake.seq;
			if (control.atomicLoad(C_SERIAL) == seen) {
				await wake.next(wakeSeq);
				if (Lanes.stopping()) {
					return;
				}
			}
			spins = WOKEN_SPINS;
			continue;
		}
		spins = IDLE_SPINS;
		seen = serial;
		let command = control.atomicLoad(C_COMMAND);
		if (result != null) {
			result.release();
			result = null;
		}
		if (command == CMD_QUIT) {
			return;
		}

		let lanes = <i32>control.atomicLoad(C_LANES);
		exchanges.length = 0;
		for (let q = 0; q < lanes * lanes; ++q) {
			exchanges.push(new SharedRegion(control.atomicLoad(C_PAIRS + 4 * <u32>q)));
		}
		let ready = C_READY + 4 * <u32>partition;
		while (control.atomicLoad(ready) != serial) {}
		let input = new Blob(control.atomicLoad(C_INPUTS + 4 * <u32>partition));
		let trace = new SharedRegion(control.atomicLoad(C_TRACES + 4 * <u32>partition));
		if (command == CMD_SOLVE) {
			let output = new SharedRegion(control.atomicLoad(C_OUTPUTS + 4 * <u32>partition));
			b3LaneSolvePartition(replica, input, output, exchanges, partition, trace);
		} else if (command == CMD_COLLIDE) {
			let snapshot = new Blob(control.atomicLoad(C_SNAPSHOT));
			let answer = b3LaneCollidePartition(collideReplica, snapshot, input, partition, trace);
			result = answer;
			let mine = exchanges[partition * lanes + partition];
			mine.atomicStore(X_RESULT, answer.id);
			mine.atomicStore(X_DONE, serial * EPOCH_STRIDE + (EPOCH_STRIDE - 1));
		}
	}
}

// Main lane side

export class b3LaneStats {
	steps: i32 = 0;
	solves: i32 = 0;
	fallbacks: i32 = 0;
	pack: f64 = 0;
	packMasks: f64 = 0;
	packNeeds: f64 = 0;
	packContacts: f64 = 0;
	packBodies: f64 = 0;
	blob: f64 = 0;
	partition0: f64 = 0;
	wait: f64 = 0;
	apply: f64 = 0;
	bytesIn: f64 = 0;
	bytesOut: f64 = 0;
	sharedBodies: f64 = 0;
	bodies: f64 = 0;
	exchanges: f64 = 0;
	laneSpins: f64 = 0;
	mainSpins: f64 = 0;
	times: Float64Array = new Float64Array(T_COUNT);
}

export class b3LaneTasks {
	lanes: i32 = 1;
	backend: i32 = b3_laneBackendWorkers;
	world: b3World;
	control: SharedRegion = SharedRegion.create(C_BYTES);
	wake: Signal = Signal.create();
	exchanges: Array<SharedRegion> = new Array<SharedRegion>();
	outputs: Array<SharedRegion> = new Array<SharedRegion>();
	// Each lane's trace spans, then the tree job's
	traces: Array<SharedRegion> = new Array<SharedRegion>();
	workers: Array<Worker> = new Array<Worker>();
	jobs: Array<Job<i32>> = new Array<Job<i32>>();
	writer: b3LaneWriter = new b3LaneWriter();
	head: b3LaneWriter = new b3LaneWriter();
	// The section blobs of the command in flight, released once it is done
	inputs: Array<Blob> = new Array<Blob>();
	part: b3LanePartition;
	staging: Uint8Array = new Uint8Array(0);
	masks: Uint32Array = new Uint32Array(0);
	mainBodies: Int32Array = new Int32Array(0);
	neighbors: StaticArray<u32> = new StaticArray<u32>(B3_MAX_LANES);
	// Per lane past the main one, contact ids and owned body indices in section order
	packContacts: Int32Array = new Int32Array(0);
	contactStarts: Int32Array = new Int32Array(B3_MAX_LANES + 1);
	packOwned: Int32Array = new Int32Array(0);
	ownedStarts: Int32Array = new Int32Array(B3_MAX_LANES + 1);
	outputBytes: Int32Array = new Int32Array(B3_MAX_LANES);
	needs: Uint32Array = new Uint32Array(B3_MAX_LANES * B3_GRAPH_COLOR_COUNT);
	pairEntries: Int32Array = new Int32Array(B3_MAX_LANES * B3_MAX_LANES);
	collide: b3LaneCollideMain = new b3LaneCollideMain();
	// With one lane, runs partition 0 through the serialized replica path on the main lane, which measures the
	// marshalling alone and checks it without any other lane
	replicaOnMain: bool = false;
	mainReplica: b3LaneSolveReplica? = null;
	collideJobs: Array<Job<Blob>> = new Array<Job<Blob>>();
	tree: b3LaneTreeMain = new b3LaneTreeMain();
	treeJob: Job<Blob>? = null;
	treeInput: Blob? = null;
	// The world's task callbacks before the lanes took them over
	enqueueTask: b3EnqueueTaskCallback;
	finishTask: b3FinishTaskCallback;
	serial: u32 = 0;
	stats: b3LaneStats = new b3LaneStats();

	constructor(world: b3World, lanes: i32, backend: i32) {
		this.world = world;
		this.enqueueTask = world.enqueueTaskFcn;
		this.finishTask = world.finishTaskFcn;
		this.lanes = lanes;
		this.backend = backend;
		this.part = new b3LanePartition(new b3StepContext(world));
		for (let p = 0; p < lanes; ++p) {
			this.outputs.push(SharedRegion.create(1 << 16));
		}
		for (let p = 0; p <= lanes; ++p) {
			this.traces.push(SharedRegion.create(B3_LANE_TRACE_BYTES));
		}
		for (let q = 0; q < lanes * lanes; ++q) {
			this.exchanges.push(SharedRegion.create(X_DATA));
		}
		this.publishRegions();
		if (backend == b3_laneBackendWorkers) {
			for (let p = 1; p < lanes; ++p) {
				this.workers.push(Worker.spawn(b3LaneWorker, this.control, this.wake, p));
			}
		}
	}

	publishRegions(): void {
		let control = this.control;
		for (let p = 0; p < this.lanes; ++p) {
			control.atomicStore(C_OUTPUTS + 4 * <u32>p, this.outputs[p].id);
			control.atomicStore(C_TRACES + 4 * <u32>p, this.traces[p].id);
		}
		for (let q = 0; q < this.lanes * this.lanes; ++q) {
			control.atomicStore(C_PAIRS + 4 * <u32>q, this.exchanges[q].id);
		}
		control.atomicStore(C_LANES, <u32>this.lanes);
	}

	stop(): void {
		if (this.backend == b3_laneBackendWorkers) {
			this.control.atomicStore(C_COMMAND, CMD_QUIT);
			this.serial += 1;
			this.control.atomicStore(C_SERIAL, this.serial);
			this.wake.notify();
		}
	}

	// Jobs are all joined between steps, so their shared memory can go; workers may still read it as they quit
	release(): void {
		if (this.backend != b3_laneBackendJobs) {
			return;
		}
		let regions = [this.exchanges, this.outputs, this.traces];
		for (let i = 0; i < regions.length; ++i) {
			for (let k = 0; k < regions[i].length; ++k) {
				regions[i][k].release();
			}
		}
		let snapshot = this.collide.snapshot.blob;
		if (snapshot != null) {
			snapshot.release();
		}
		this.control.release();
		this.wake.release();
	}
}

@inline unsafe function b3MarkBody(masks: Uint32Array, encoded: i32, bit: u32): void {
	if (encoded >= 0) {
		store<u32>(masks.dataStart + <usize>encoded * 4, load<u32>(masks.dataStart + <usize>encoded * 4) | bit);
	}
}

// A shared body that lane q's constraint writes in active color i: every other lane touching it receives it
@inline function b3NeedBody(
	needs: Uint32Array,
	pairEntries: Int32Array,
	masks: Uint32Array,
	encoded: i32,
	q: i32,
	i: i32,
	lanes: i32
): void {
	if (encoded < 0) {
		return;
	}
	let mask = masks[encoded];
	if ((mask & (mask - 1)) == 0) {
		return;
	}
	let qBit: u32 = 1 << <u32>q;
	let receivers = mask & ~qBit;
	while (receivers != 0) {
		let p = <i32>ctz(receivers);
		receivers &= receivers - 1;
		needs[p * B3_GRAPH_COLOR_COUNT + i] |= qBit;
		pairEntries[q * lanes + p] += 1;
	}
}

unsafe function b3WriteContact(w: b3LaneWriter, contact: b3Contact): void {
	let bytes = contact.manifoldCount * <i32>sizeof<b3Manifold>();
	let at = w.reserve(R_WORDS * 4 + bytes);
	store<i32>(at, contact.contactId);
	store<u32>(at + 4, contact.flags);
	store<i32>(at + 8, contact.encodedBodySimA);
	store<i32>(at + 12, contact.encodedBodySimB);
	store<i32>(at + 16, contact.manifoldCount);
	store<f32>(at + 20, contact.friction);
	store<f32>(at + 24, contact.restitution);
	store<f32>(at + 28, contact.rollingResistance);
	store<f32>(at + 32, contact.tangentVelocity.x);
	store<f32>(at + 36, contact.tangentVelocity.y);
	store<f32>(at + 40, contact.tangentVelocity.z);
	memory.copy(at + <usize>(R_WORDS * 4), changetype<usize>(contact.manifolds), <usize>bytes);
}

// Calls fn on each contact lane p takes of active color i: convex lanes, then mesh lanes in lane order
@inline function b3ColorLaneRange(count: i32, lanesTotal: i32, p: i32, lanes: i32): i32 {
	return min(4 * b3SplitStart(count, p, lanes), lanesTotal);
}

// Sets up the main lane's partition 0 over the world's own step context
function b3SetMainPartition(t: b3LaneTasks, context: b3StepContext, activeColorCount: i32): void {
	let part = t.part;
	let lanes = t.lanes;
	let colors = context.world.constraintGraph.colors;
	part.context = context;
	part.partition = 0;
	part.lanes = lanes;
	part.activeColorCount = activeColorCount;
	part.neighbors = t.neighbors[0];
	part.epochBase = t.serial * EPOCH_STRIDE;
	part.needs = t.needs;
	part.spins = 0;
	part.exchangesWaited = 0;
	for (let i = 0; i < T_COUNT; ++i) {
		part.times[i] = 0;
	}

	let contacts = context.world.contacts;
	let masks = t.masks;
	part.maskOf = masks;
	part.shared = b3GrowI32(part.shared, 2 * contacts.length + 16);
	let sharedCount = 0;
	for (let i = 0; i < activeColorCount; ++i) {
		let color = colors[part.colorIndices[i]];
		let wideEnd = b3SplitStart(color.wideConstraintCount, 1, lanes);
		let groupEnd = b3SplitStart(color.wideMeshConstraintCount, 1, lanes);
		part.wideFirst[i] = 0;
		part.wideCount[i] = wideEnd;
		part.wideFlat[i] = color.wideConstraintStart;
		part.groupFirst[i] = 0;
		part.groupCount[i] = groupEnd;
		part.groupFlat[i] = color.wideMeshConstraintStart;

		part.sharedStarts[i] = sharedCount;
		let last = min(4 * wideEnd, color.convexContacts.length);
		for (let k = 0; k < last; ++k) {
			let contact = contacts[color.convexContacts[k]];
			let a = contact.encodedBodySimA;
			let b = contact.encodedBodySimB;
			if (a >= 0 && popcnt(masks[a]) > 1) {
				part.shared[sharedCount++] = a;
			}
			if (b >= 0 && popcnt(masks[b]) > 1) {
				part.shared[sharedCount++] = b;
			}
		}
		let span = context.meshPrepareSpans[i];
		last = min(4 * groupEnd, color.contacts.length);
		for (let k = 0; k < last; ++k) {
			let contact = contacts[color.contacts[span.order[span.orderBase + k]].contactId];
			let a = contact.encodedBodySimA;
			let b = contact.encodedBodySimB;
			if (a >= 0 && popcnt(masks[a]) > 1) {
				part.shared[sharedCount++] = a;
			}
			if (b >= 0 && popcnt(masks[b]) > 1) {
				part.shared[sharedCount++] = b;
			}
		}
	}
	part.sharedStarts[activeColorCount] = sharedCount;
	part.sendBuffer = b3GrowU32(part.sendBuffer, E_WORDS * (sharedCount + 1));

	let awakeCount = context.sims.length;
	t.mainBodies = b3GrowI32(t.mainBodies, awakeCount);
	let bodyCount = 0;
	for (let index = 0; index < awakeCount; ++index) {
		if ((masks[index] & 1) != 0) {
			t.mainBodies[bodyCount++] = index;
		}
	}
	b3SetRuns(part, t.mainBodies, bodyCount);
}

// Serializes the other lanes' shares of the step's constraints, starts them, runs partition 0 here over the
// world's own arrays and applies what the others wrote. Answers false, leaving the solver to the task callbacks,
// for what it does not cover.
unsafe function b3LaneSolve(t: b3LaneTasks, context: b3StepContext): bool {
	let world = context.world;
	let colors = world.constraintGraph.colors;
	let overflow = colors[B3_OVERFLOW_INDEX];
	let stats = t.stats;
	if (world.workerCount != 1 || overflow.contacts.length > 0 || overflow.jointSims.length > 0) {
		stats.fallbacks += 1;
		return false;
	}
	let part = t.part;
	let activeColorCount = 0;
	for (let i = 0; i < B3_OVERFLOW_INDEX; ++i) {
		let color = colors[i];
		if (color.jointSims.length > 0) {
			stats.fallbacks += 1;
			return false;
		}
		if (color.convexContacts.length + color.contacts.length > 0) {
			part.colorIndices[activeColorCount++] = i;
		}
	}
	if (activeColorCount != context.activeColorCount) {
		stats.fallbacks += 1;
		return false;
	}

	// Workers parked between steps wake while the step is serialized below
	if (t.backend == b3_laneBackendWorkers) {
		t.wake.notify();
	}

	// What the skipped task loop of b3Solve clears; partition 0's stores set hit bits here directly
	let taskContext = world.taskContexts[0];
	b3SetBitCountAndClear(taskContext.jointStateBitSet, <u32>b3GetIdCapacity(world.jointIdPool));
	b3SetBitCountAndClear(taskContext.hitEventBitSet, <u32>b3GetIdCapacity(world.contactIdPool));
	taskContext.hasHitEvents = false;

	let trace = b3TraceRowAt(0);
	let traceBegin = b3SpanBegin(trace);
	let ticks = b3LaneNow();
	let lanes = t.lanes;
	let awakeCount = context.sims.length;
	let contacts = world.contacts;
	t.masks = b3GrowU32(t.masks, awakeCount);
	let masks = t.masks;
	masks.fill(0, 0, awakeCount);

	// Which lanes touch each body
	let anyRestitution = false;
	for (let i = 0; i < activeColorCount; ++i) {
		let color = colors[part.colorIndices[i]];
		let span = context.meshPrepareSpans[i];
		for (let p = 0; p < lanes; ++p) {
			let bit: u32 = 1 << <u32>p;
			let first = b3ColorLaneRange(color.wideConstraintCount, color.convexContacts.length, p, lanes);
			let last = b3ColorLaneRange(color.wideConstraintCount, color.convexContacts.length, p + 1, lanes);
			for (let k = first; k < last; ++k) {
				let contact = contacts[color.convexContacts[k]];
				b3MarkBody(masks, contact.encodedBodySimA, bit);
				b3MarkBody(masks, contact.encodedBodySimB, bit);
				anyRestitution = anyRestitution || contact.restitution > 0.0;
			}
			first = b3ColorLaneRange(color.wideMeshConstraintCount, color.contacts.length, p, lanes);
			last = b3ColorLaneRange(color.wideMeshConstraintCount, color.contacts.length, p + 1, lanes);
			for (let k = first; k < last; ++k) {
				let contact = contacts[color.contacts[span.order[span.orderBase + k]].contactId];
				b3MarkBody(masks, contact.encodedBodySimA, bit);
				b3MarkBody(masks, contact.encodedBodySimB, bit);
				anyRestitution = anyRestitution || contact.restitution > 0.0;
			}
		}
	}

	stats.packMasks += b3LaneNow() - ticks;
	let phaseTicks = b3LaneNow();

	// Free bodies go to the lane their index falls in; neighbors are lanes that share a body
	let neighbors = t.neighbors;
	for (let p = 0; p < B3_MAX_LANES; ++p) {
		neighbors[p] = 0;
	}
	let shared = 0;
	for (let index = 0; index < awakeCount; ++index) {
		let mask = masks[index];
		if (mask == 0) {
			masks[index] = 1 << <u32>((<i64>index * <i64>lanes) / <i64>awakeCount);
			continue;
		}
		if ((mask & (mask - 1)) != 0) {
			shared += 1;
			for (let p = 0; p < lanes; ++p) {
				if ((mask & (1 << <u32>p)) != 0) {
					neighbors[p] |= mask & ~(1 << <u32>p);
				}
			}
		}
	}
	stats.sharedBodies += <f64>shared;
	stats.bodies += <f64>awakeCount;

	// Who sends whom velocities in each color: the lane whose constraint writes a shared body, to the other lanes
	// that touch it. Each pair holds a step's worth of its sends.
	let needs = t.needs;
	needs.fill(0);
	let pairEntries = t.pairEntries;
	pairEntries.fill(0);
	if (shared > 0) {
		for (let i = 0; i < activeColorCount; ++i) {
			let color = colors[part.colorIndices[i]];
			let span = context.meshPrepareSpans[i];
			for (let q = 0; q < lanes; ++q) {
				let first = b3ColorLaneRange(color.wideConstraintCount, color.convexContacts.length, q, lanes);
				let last = b3ColorLaneRange(color.wideConstraintCount, color.convexContacts.length, q + 1, lanes);
				for (let k = first; k < last; ++k) {
					let contact = contacts[color.convexContacts[k]];
					b3NeedBody(needs, pairEntries, masks, contact.encodedBodySimA, q, i, lanes);
					b3NeedBody(needs, pairEntries, masks, contact.encodedBodySimB, q, i, lanes);
				}
				first = b3ColorLaneRange(color.wideMeshConstraintCount, color.contacts.length, q, lanes);
				last = b3ColorLaneRange(color.wideMeshConstraintCount, color.contacts.length, q + 1, lanes);
				for (let k = first; k < last; ++k) {
					let contact = contacts[color.contacts[span.order[span.orderBase + k]].contactId];
					b3NeedBody(needs, pairEntries, masks, contact.encodedBodySimA, q, i, lanes);
					b3NeedBody(needs, pairEntries, masks, contact.encodedBodySimB, q, i, lanes);
				}
			}
		}
	}
	stats.packNeeds += b3LaneNow() - phaseTicks;
	let passes = 3 * context.subStepCount + (anyRestitution ? world.restitutionIterations : 0);
	if (passes * activeColorCount >= X_TABLE_ENTRIES) {
		stats.fallbacks += 1;
		return false;
	}
	let regrown = false;
	for (let q = 0; q < lanes * lanes; ++q) {
		let bytes = <i32>X_DATA + passes * pairEntries[q] * E_WORDS * 4;
		let old = t.exchanges[q];
		if (bytes > <i32>old.size) {
			let fresh = SharedRegion.create(<u32>(bytes + (bytes >> 1)));
			fresh.atomicStore(X_EPOCH, old.atomicLoad(X_EPOCH));
			fresh.atomicStore(X_DONE, old.atomicLoad(X_DONE));
			t.exchanges[q] = fresh;
			old.release();
			regrown = true;
		}
	}
	if (regrown) {
		t.publishRegions();
	}

	t.serial += 1;
	let epochBase = t.serial * EPOCH_STRIDE;

	// The workers wake now and pick up each section as it is ready
	let replicaOnMain = lanes == 1 && t.replicaOnMain;
	let firstPacked = replicaOnMain ? 0 : 1;
	for (let p = firstPacked; p < lanes; ++p) {
		b3LaneTraceArm(t.traces[p]);
	}
	if (t.backend == b3_laneBackendWorkers && lanes > 1) {
		let control = t.control;
		control.atomicStore(C_COMMAND, CMD_SOLVE);
		control.atomicStore(C_SERIAL, t.serial);
		t.wake.notify();
	}
	t.jobs.length = 0;
	t.inputs.length = 0;

	let w = t.head;
	w.reset();
	w.i32(awakeCount);
	w.i32(activeColorCount);
	w.i32(context.subStepCount);
	w.f32(context.dt);
	w.f32(context.inv_dt);
	w.f32(context.h);
	w.f32(context.inv_h);
	w.f32(context.contactSoftness.biasRate);
	w.f32(context.contactSoftness.massScale);
	w.f32(context.contactSoftness.impulseScale);
	w.f32(context.staticSoftness.biasRate);
	w.f32(context.staticSoftness.massScale);
	w.f32(context.staticSoftness.impulseScale);
	w.f32(context.maxLinearVelocity);
	w.i32(context.enableWarmStarting ? 1 : 0);
	w.i32(world.restitutionIterations);
	w.f32(world.restitutionThreshold);
	w.i32(world.enableRestitutionPropagation ? 1 : 0);
	w.f32(world.contactSpeed);
	w.f32(world.hitEventThreshold);
	w.f32(world.gravity.x);
	w.f32(world.gravity.y);
	w.f32(world.gravity.z);
	w.i32(b3GetIdCapacity(world.contactIdPool));
	w.i32(anyRestitution ? 1 : 0);
	w.u32(epochBase);
	w.f32(b3GetLengthUnitsPerMeter());
	w.i32(lanes);
	for (let p = 0; p < B3_MAX_LANES; ++p) {
		w.u32(neighbors[p]);
	}
	for (let i = 0; i < B3_GRAPH_COLOR_COUNT; ++i) {
		w.i32(i < activeColorCount ? part.colorIndices[i] : -1);
	}
	for (let k = 0; k < B3_MAX_LANES * B3_GRAPH_COLOR_COUNT; ++k) {
		w.u32(needs[k]);
	}
	let head = w;
	w = t.writer;
	w.reset();

	t.packContacts = b3GrowI32(t.packContacts, contacts.length + 16);
	t.packOwned = b3GrowI32(t.packOwned, awakeCount);
	let packedContacts = 0;
	let packedOwned = 0;
	let simSize = <i32>sizeof<b3BodySim>();
	let stateSize = <i32>sizeof<b3BodyState>();
	let sims = b3ArrayData(context.sims);
	let states = b3ArrayData(context.states);
	for (let p = firstPacked; p < lanes; ++p) {
		let blobStart = w.length;
		w.bytes(head.buffer.dataStart, head.length);
		let sectionStart = w.length;
		w.u32(0);
		w.u32(0);
		for (let i = 0; i < activeColorCount; ++i) {
			let color = colors[part.colorIndices[i]];
			let wideCount = color.wideConstraintCount;
			let groupCount = color.wideMeshConstraintCount;
			w.i32(b3SplitStart(wideCount, p + 1, lanes) - b3SplitStart(wideCount, p, lanes));
			w.i32(
				b3ColorLaneRange(wideCount, color.convexContacts.length, p + 1, lanes) -
					b3ColorLaneRange(wideCount, color.convexContacts.length, p, lanes)
			);
			w.i32(b3SplitStart(groupCount, p + 1, lanes) - b3SplitStart(groupCount, p, lanes));
			w.i32(
				b3ColorLaneRange(groupCount, color.contacts.length, p + 1, lanes) -
					b3ColorLaneRange(groupCount, color.contacts.length, p, lanes)
			);
		}

		phaseTicks = b3LaneNow();
		t.contactStarts[p] = packedContacts;
		let outputBytes = 4 * O_WORDS;
		for (let i = 0; i < activeColorCount; ++i) {
			let color = colors[part.colorIndices[i]];
			let span = context.meshPrepareSpans[i];
			let first = b3ColorLaneRange(color.wideConstraintCount, color.convexContacts.length, p, lanes);
			let last = b3ColorLaneRange(color.wideConstraintCount, color.convexContacts.length, p + 1, lanes);
			for (let k = first; k < last; ++k) {
				let contact = contacts[color.convexContacts[k]];
				b3WriteContact(w, contact);
				t.packContacts[packedContacts++] = contact.contactId;
				outputBytes += b3ImpulseBytes(contact.manifolds, contact.manifoldCount);
			}
			first = b3ColorLaneRange(color.wideMeshConstraintCount, color.contacts.length, p, lanes);
			last = b3ColorLaneRange(color.wideMeshConstraintCount, color.contacts.length, p + 1, lanes);
			for (let k = first; k < last; ++k) {
				let contact = contacts[color.contacts[span.order[span.orderBase + k]].contactId];
				b3WriteContact(w, contact);
				t.packContacts[packedContacts++] = contact.contactId;
				outputBytes += b3ImpulseBytes(contact.manifolds, contact.manifoldCount);
			}
		}

		stats.packContacts += b3LaneNow() - phaseTicks;
		phaseTicks = b3LaneNow();
		t.ownedStarts[p] = packedOwned;
		let bit: u32 = 1 << <u32>p;
		let bodyCount = 0;
		let recordBytes = B_WORDS * 4 + simSize + stateSize;
		for (let index = 0; index < awakeCount; ++index) {
			let mask = masks[index];
			if ((mask & bit) == 0) {
				continue;
			}
			let at = w.reserve(recordBytes);
			store<i32>(at, index);
			store<u32>(at + 4, mask);
			memory.copy(at + <usize>(B_WORDS * 4), sims + <usize>index * <usize>simSize, <usize>simSize);
			memory.copy(at + <usize>(B_WORDS * 4 + simSize), states + <usize>index * <usize>stateSize, <usize>stateSize);
			bodyCount += 1;
			if (<i32>ctz(mask) == p) {
				t.packOwned[packedOwned++] = index;
				outputBytes += stateSize;
			}
		}

		stats.packBodies += b3LaneNow() - phaseTicks;
		w.patch(sectionStart, <u32>(packedContacts - t.contactStarts[p]));
		w.patch(sectionStart + 4, <u32>bodyCount);
		t.outputBytes[p] = outputBytes;
		if (outputBytes > <i32>t.outputs[p].size) {
			t.outputs[p].release();
			t.outputs[p] = SharedRegion.create(<u32>(outputBytes + (outputBytes >> 1)));
			t.publishRegions();
		}

		let blobTicks = b3LaneNow();
		let input = Blob.fromArray(w.buffer, blobStart, w.length - blobStart);
		stats.blob += b3LaneNow() - blobTicks;
		t.inputs.push(input);
		if (p > 0) {
			if (t.backend == b3_laneBackendWorkers) {
				t.control.atomicStore(C_INPUTS + 4 * <u32>p, input.id);
				t.control.atomicStore(C_READY + 4 * <u32>p, t.serial);
			} else {
				t.jobs.push(Jobs.run(b3LaneSolveJob, input, t.outputs[p], t.exchanges, p, t.traces[p]));
			}
		}
	}
	t.contactStarts[lanes] = packedContacts;
	t.ownedStarts[lanes] = packedOwned;
	b3SetMainPartition(t, context, activeColorCount);
	context.anyRestitution.value = anyRestitution ? 1 : 0;
	stats.pack += b3LaneNow() - ticks;
	stats.bytesIn += <f64>w.length;
	b3SpanEnd(trace, b3_traceCopy, traceBegin);

	ticks = b3LaneNow();
	if (replicaOnMain) {
		if (t.mainReplica == null) {
			t.mainReplica = new b3LaneSolveReplica();
		}
		let replica = t.mainReplica!;
		b3LaneSolvePartition(replica, t.inputs[0], t.outputs[0], t.exchanges, 0, t.traces[0]);
		part = replica.part;
	} else {
		part.trace = trace;
		b3LaneSolveStages(part, t.exchanges);
	}
	stats.partition0 += b3LaneNow() - ticks;

	ticks = b3LaneNow();
	if (t.backend == b3_laneBackendWorkers) {
		let done = epochBase | (EPOCH_STRIDE - 1);
		for (let p = 1; p < lanes; ++p) {
			stats.mainSpins += <f64>b3LaneAwait(t.exchanges[p * lanes + p], X_DONE, done);
		}
	} else {
		for (let k = 0; k < t.jobs.length; ++k) {
			let job = t.jobs[k];
			let result = job.join();
			if (result == null || !result.ok) {
				console.log("b3LaneSolve: job failed " + (job.failed != null ? job.failed! : "?"));
			}
			job.release();
		}
		t.jobs.length = 0;
	}
	stats.wait += b3LaneNow() - ticks;
	for (let p = firstPacked; p < lanes; ++p) {
		b3LaneTraceGather(t.traces[p], p);
	}

	ticks = b3LaneNow();
	traceBegin = b3SpanBegin(trace);
	b3LaneSolveApply(t, context, firstPacked);
	for (let k = 0; k < t.inputs.length; ++k) {
		t.inputs[k].release();
	}
	t.inputs.length = 0;
	stats.apply += b3LaneNow() - ticks;
	b3SpanEnd(trace, b3_traceCopy, traceBegin);
	stats.solves += 1;
	stats.mainSpins += <f64>part.spins;
	stats.exchanges += <f64>part.exchangesWaited;
	for (let i = 0; i < T_COUNT; ++i) {
		stats.times[i] += part.times[i];
	}

	// The main lane's partition stands for the stage times, as C's main thread does
	let times = part.times;
	world.profile.prepareConstraints += <f32>times[T_PREPARE];
	world.profile.integrateVelocities += <f32>times[T_INTEGRATE_VELOCITIES];
	world.profile.warmStart += <f32>times[T_WARM_START];
	world.profile.solveImpulses += <f32>times[T_SOLVE];
	world.profile.integratePositions += <f32>times[T_INTEGRATE_POSITIONS];
	world.profile.relaxImpulses += <f32>times[T_RELAX];
	world.profile.restitution += <f32>times[T_RESTITUTION];
	world.profile.storeImpulses += <f32>times[T_STORE];
	return true;
}

// The hit bits come from partition 0's stores straight into the world's task context, and from the others' outputs
unsafe function b3LaneSolveApply(t: b3LaneTasks, context: b3StepContext, firstPacked: i32): void {
	let world = context.world;
	let taskContext = world.taskContexts[0];
	let contacts = world.contacts;
	let stateSize = <usize>sizeof<b3BodyState>();
	let pointSize = sizeof<b3ManifoldPoint>();
	let states = b3ArrayData(context.states);
	let stats = t.stats;
	for (let p = firstPacked; p < t.lanes; ++p) {
		let bytes = t.outputBytes[p];
		t.staging = b3GrowU8(t.staging, bytes);
		let staging = t.staging;
		t.outputs[p].exclusive((view) => {
			view.readInto(0, staging, 0, bytes);
		});
		stats.bytesOut += <f64>bytes;
		let base = staging.dataStart;
		stats.laneSpins += <f64>load<u32>(base + 8) * 1000.0;
		let at = base + <usize>(4 * O_WORDS);
		for (let k = t.contactStarts[p]; k < t.contactStarts[p + 1]; ++k) {
			let contactId = t.packContacts[k];
			let contact = contacts[contactId];
			if (load<u32>(at) != 0) {
				b3SetBit(taskContext.hitEventBitSet, <u32>contactId);
				taskContext.hasHitEvents = true;
			}
			at += 4;
			let manifolds = changetype<usize>(contact.manifolds);
			for (let m = 0; m < contact.manifoldCount; ++m) {
				let manifold = manifolds + <usize>m * sizeof<b3Manifold>();
				memory.copy(manifold + offsetof<b3Manifold>("twistImpulse"), at, 28);
				at += 28;
				let pointCount = load<i32>(manifold + offsetof<b3Manifold>("pointCount"));
				for (let j = 0; j < pointCount; ++j) {
					let point = manifold + offsetof<b3Manifold>("points") + <usize>j * pointSize;
					store<u32>(point + offsetof<b3ManifoldPoint>("normalImpulse"), load<u32>(at));
					store<u32>(point + offsetof<b3ManifoldPoint>("totalNormalImpulse"), load<u32>(at + 4));
					store<u32>(point + offsetof<b3ManifoldPoint>("normalVelocity"), load<u32>(at + 8));
					at += 12;
				}
			}
		}
		for (let k = t.ownedStarts[p]; k < t.ownedStarts[p + 1]; ++k) {
			memory.copy(states + <usize>t.packOwned[k] * stateSize, at, stateSize);
			at += stateSize;
		}
	}
}

// Serializes the other lanes' shares of the step's awake contacts, starts them, runs b3CollideTask over the first
// share here and copies the others' contacts back. Answers false for what it does not cover.
unsafe function b3LaneCollide(t: b3LaneTasks, context: b3StepContext, contactCount: i32): bool {
	let m = t.collide;
	let world = context.world;
	let lanes = t.lanes;
	if (!b3LaneCollideSupported(world) || contactCount < 4 * lanes) {
		m.fallbacks += 1;
		return false;
	}
	t.serial += 1;
	let serial = t.serial;
	for (let p = 1; p < lanes; ++p) {
		b3LaneTraceArm(t.traces[p]);
	}
	let trace = b3TraceRowAt(0);
	let traceBegin = b3SpanBegin(trace);
	b3LaneCollideBegin(m, context, contactCount, lanes);
	let snapshot = m.snapshot.blob!;
	let control = t.control;
	if (t.backend == b3_laneBackendWorkers) {
		control.atomicStore(C_SNAPSHOT, snapshot.id);
		control.atomicStore(C_COMMAND, CMD_COLLIDE);
		control.atomicStore(C_SERIAL, serial);
		t.wake.notify();
	}
	t.collideJobs.length = 0;
	t.inputs.length = 0;
	for (let p = 1; p < lanes; ++p) {
		let input = b3LaneCollideSection(m, context, p);
		t.inputs.push(input);
		if (t.backend == b3_laneBackendWorkers) {
			control.atomicStore(C_INPUTS + 4 * <u32>p, input.id);
			control.atomicStore(C_READY + 4 * <u32>p, serial);
		} else {
			t.collideJobs.push(Jobs.run(b3LaneCollideJob, snapshot, input, p, t.traces[p]));
		}
	}
	b3SpanEnd(trace, b3_traceCopy, traceBegin);

	let ticks = b3LaneNow();
	traceBegin = b3SpanBegin(trace);
	b3CollideTask(m.ranges[0], m.ranges[1], 0, context);
	m.partition0 += b3LaneNow() - ticks;
	b3SpanEnd(trace, b3_traceCollide, traceBegin);

	for (let p = 1; p < lanes; ++p) {
		ticks = b3LaneNow();
		let answer: Blob? = null;
		if (t.backend == b3_laneBackendWorkers) {
			let mine = t.exchanges[p * lanes + p];
			b3LaneAwait(mine, X_DONE, serial * EPOCH_STRIDE + (EPOCH_STRIDE - 1));
			answer = new Blob(mine.atomicLoad(X_RESULT));
		} else {
			let job = t.collideJobs[p - 1];
			let result = job.join();
			if (result == null || !result.ok) {
				console.log("b3LaneCollide: job failed " + (job.failed != null ? job.failed! : "?"));
			} else {
				answer = result.value;
			}
		}
		m.wait += b3LaneNow() - ticks;
		b3LaneTraceGather(t.traces[p], p);
		if (answer != null) {
			traceBegin = b3SpanBegin(trace);
			b3LaneCollideApply(m, context, p, answer);
			b3SpanEnd(trace, b3_traceCopy, traceBegin);
			answer.release();
		}
		if (t.backend == b3_laneBackendJobs) {
			t.collideJobs[p - 1].release();
		}
	}
	for (let k = 0; k < t.inputs.length; ++k) {
		t.inputs[k].release();
	}
	t.inputs.length = 0;
	m.steps += 1;
	return true;
}

// Handle of the tree task in flight, the only task this layer's enqueue callback defers
const TREE_TASK: u64 = 1;

// The world's enqueue callback while the tree task is on lanes. Every other task carries its arguments inside its
// closure, which cannot cross to a lane, so it runs here at once as the default callback runs it.
unsafe function b3LaneEnqueue(t: b3LaneTasks, task: b3TaskCallback, taskContext: u64, name: string): u64 {
	if (name == "rebuild tree" && t.treeJob == null) {
		let trace = b3TraceRowAt(0);
		let traceBegin = b3SpanBegin(trace);
		let input = b3LaneTreePack(t.tree, t.world);
		t.treeInput = input;
		let treeTrace = t.traces[t.lanes];
		b3LaneTraceArm(treeTrace);
		t.treeJob = Jobs.run(b3LaneTreeJob, input, treeTrace);
		b3SpanEnd(trace, b3_traceCopy, traceBegin);
		t.tree.tasks += 1;
		return TREE_TASK;
	}
	task(taskContext);
	return 0;
}

unsafe function b3LaneFinish(t: b3LaneTasks, userTask: u64): void {
	let job = t.treeJob;
	if (userTask != TREE_TASK || job == null) {
		return;
	}
	let ticks = b3LaneNow();
	let result = job.join();
	t.tree.wait += b3LaneNow() - ticks;
	// The tree job's row follows the lanes'
	b3LaneTraceGather(t.traces[t.lanes], t.lanes);
	if (result == null || !result.ok) {
		console.log("b3LaneFinish: tree job failed " + (job.failed != null ? job.failed! : "?"));
	} else {
		let output = result.value;
		let trace = b3TraceRowAt(0);
		let traceBegin = b3SpanBegin(trace);
		b3LaneTreeApply(t.tree, t.world, output);
		b3SpanEnd(trace, b3_traceCopy, traceBegin);
		output.release();
	}
	job.release();
	t.treeJob = null;
	t.treeInput!.release();
	t.treeInput = null;
}

const b3_laneTasks = new Array<b3LaneTasks>();

// Runs worldId's narrow phase and solver stages, as tasks picks, on lanes partitions: partition 0 on the main lane,
// the others on jobs or workers. lanes 0 turns it off.
export function b3World_EnableLanes(worldId: b3WorldId, lanes: i32, backend: i32, tasks: i32): b3LaneTasks? {
	for (let i = 0; i < b3_laneTasks.length; ++i) {
		let old = b3_laneTasks[i];
		old.stop();
		old.release();
		old.world.enqueueTaskFcn = old.enqueueTask;
		old.world.finishTaskFcn = old.finishTask;
	}
	b3_laneTasks.length = 0;
	b3SetLaneSolve(null);
	b3SetLaneCollide(null);
	if (lanes <= 0) {
		return null;
	}
	if (backend == b3_laneBackendParallel) {
		b3World_EnableParallel(worldId, tasks & (b3_laneCollide | b3_laneSolve));
		return null;
	}
	b3SetLaneClock((): f64 => <f64>b3GetTicks() * 1e-6);
	let world = b3GetWorldFromId(worldId);
	lanes = min(lanes, B3_MAX_LANES);
	// The colored stages wait for every partition, so jobs need a pool thread for each lane past the main one
	if (backend == b3_laneBackendJobs && <u32>(lanes - 1) > Jobs.threads()) {
		console.log("b3World_EnableLanes: " + lanes.toString() + " lanes need " + (lanes - 1).toString() +
			" job threads, the pool has " + Jobs.threads().toString());
		lanes = <i32>Jobs.threads() + 1;
	}
	let laneTasks = new b3LaneTasks(world, lanes, backend);
	b3_laneTasks.push(laneTasks);
	if ((tasks & b3_laneSolve) != 0) {
		b3SetLaneSolve((context: b3StepContext): bool => b3LaneSolve(b3_laneTasks[0], context));
	}
	if ((tasks & b3_laneCollide) != 0) {
		b3SetLaneCollide((context: b3StepContext, contactCount: i32): bool => b3LaneCollide(b3_laneTasks[0], context, contactCount));
	}
	if ((tasks & b3_laneTree) != 0) {
		world.enqueueTaskFcn = (task: b3TaskCallback, taskContext: u64, userContext: u64, name: string): u64 =>
			b3LaneEnqueue(b3_laneTasks[0], task, taskContext, name);
		world.finishTaskFcn = (userTask: u64, userContext: u64): void => b3LaneFinish(b3_laneTasks[0], userTask);
	}
	return laneTasks;
}
