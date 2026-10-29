// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Box3D's threaded step on parallel calls: the solver stages and the narrow phase run as parts of one call over the
// world's own memory, as C's workers do. notes/threading.md has the design and the costs.

import { Parallel, Part } from "rive/parallel";
import { formatFixed } from "rive/format";

import { b3GetMillisecondsAndReset, b3GetTicks, b3Ticks } from "./core";
import { b3SetBitCountAndClear } from "./bitset";
import { b3GetIdCapacity } from "./id_pool";
import { B3_OVERFLOW_INDEX } from "./constraint_graph";
import {
	b3PrepareContacts_Overflow,
	b3WarmStartContacts_Overflow,
	b3PushContacts_Overflow,
	b3SolveContacts_Overflow,
	b3ApplyRestitution_Overflow,
	b3StoreImpulses_Overflow,
} from "./contact_solver";
import {
	b3PrepareJoints_Overflow,
	b3WarmStartJoints_Overflow,
	b3SolveJoints_Overflow,
} from "./joint";
import { b3PrepareCompoundContacts } from "./contact";
import { b3ReserveMeshScratchForWorkers } from "./mesh_contact";
import {
	b3ExecuteBlock,
	b3FinalizeBodiesTask,
	b3FinishParallelFinalize,
	b3PrepareParallelContinuous,
	b3SetLaneFinalize,
	b3SetLaneSolve,
	b3SolverStage,
	b3StepContext,
} from "./solver";
import {
	b3CollectParallelPairs,
	b3CrossContext,
	b3CrossPairsTask,
	b3FinishTreesTask,
	b3PrepareParallelPairs,
	b3ReserveTreesTask,
	b3SelfPairsTask,
	b3SetLanePairs,
	b3UpdateTreesTaskAt,
} from "./broad_phase";
import {
	b3CollideTask,
	b3DefaultFrictionCallback,
	b3DefaultRestitutionCallback,
	b3GetWorldFromId,
	b3SetLaneCollide,
	b3World,
	b3World_SetWorkerCount,
} from "./physics_world";
import { b3EnqueueTaskCallback, b3FinishTaskCallback, b3TaskCallback, b3WorldId } from "./types";
import { B3_MAX_WORKERS } from "./constants";
import {
	b3SpanBegin,
	b3SpanEnd,
	b3TraceRowAt,
	b3TraceStageKind,
	b3_traceCollide,
	b3_traceFinalize,
	b3_tracePairs,
	b3_traceTree,
} from "./trace";

export const b3_parallelCollide: i32 = 1;
export const b3_parallelSolve: i32 = 2;

// Per step totals of the work the parts leave to the main lane
export class b3ParallelStats {
	collides: i32 = 0;
	contacts: i64 = 0;
	deferredContacts: i64 = 0;
	finalizes: i32 = 0;
	deferredBodies: i64 = 0;
	solves: i32 = 0;
}

export const b3_parallelStats = new b3ParallelStats();

// The most parts a call runs on
let s_partLimit = B3_MAX_WORKERS;

// Runs one stage's blocks on whichever parts claim them, then waits for every part, as C's main thread waits for
// the stage's completion count.
function b3RunStage(part: Part, stage: b3SolverStage, context: b3StepContext): void {
	let blockCount = stage.blockCount;
	if (blockCount == 0) {
		return;
	}

	let blocks = stage.blocks;
	let workerIndex = part.index;
	let trace = b3TraceRowAt(workerIndex);
	let traceBegin = b3SpanBegin(trace);
	for (let i = part.claim(blockCount, 1); i < blockCount; i = part.claim(blockCount, 1)) {
		b3ExecuteBlock(stage, context, blocks[i].block, workerIndex);
	}
	b3SpanEnd(trace, b3TraceStageKind(stage.type), traceBegin);
	part.sync();
}

// C's single threaded overflow work runs on the main thread between stages while the workers spin
const OVERFLOW_PREPARE: i32 = 0;
const OVERFLOW_WARM_START: i32 = 1;
const OVERFLOW_SOLVE: i32 = 2;
const OVERFLOW_RELAX: i32 = 3;
const OVERFLOW_RESTITUTION: i32 = 4;
const OVERFLOW_STORE: i32 = 5;

function b3RunOverflow(part: Part, context: b3StepContext, kind: i32, hasOverflow: bool): void {
	if (hasOverflow == false) {
		return;
	}

	if (part.index == 0) {
		if (kind == OVERFLOW_PREPARE) {
			b3PrepareJoints_Overflow(context);
			b3PrepareContacts_Overflow(context);
		} else if (kind == OVERFLOW_WARM_START) {
			b3WarmStartJoints_Overflow(context);
			b3WarmStartContacts_Overflow(context);
		} else if (kind == OVERFLOW_SOLVE) {
			b3SolveJoints_Overflow(context, true);
			b3PushContacts_Overflow(context);
		} else if (kind == OVERFLOW_RELAX) {
			b3SolveJoints_Overflow(context, false);
			b3SolveContacts_Overflow(context);
		} else if (kind == OVERFLOW_RESTITUTION) {
			b3ApplyRestitution_Overflow(context);
		} else {
			b3StoreImpulses_Overflow(context);
		}
	}
	part.sync();
}

// b3SolverTask's orchestrator walk, run by every part: the stages in C's order with a sync after each
@parallel.inplace export function b3ParallelSolverTask(part: Part, context: b3StepContext): void {
	let activeColorCount = context.activeColorCount;
	let stages = context.stages;
	let overflow = context.graph.colors[B3_OVERFLOW_INDEX];
	let hasOverflow = overflow.contacts.length > 0 || overflow.jointSims.length > 0;
	let world = context.world;

	// C's main thread times the stages; here part 0 is the main lane
	let main = part.index == 0;
	let ticks = b3Ticks();
	ticks.value = main ? b3GetTicks() : 0;

	let stageIndex = 0;

	// Prepare joints, convex contacts and mesh contacts
	b3RunStage(part, stages[stageIndex], context);
	stageIndex += 1;
	b3RunStage(part, stages[stageIndex], context);
	stageIndex += 1;
	b3RunStage(part, stages[stageIndex], context);
	stageIndex += 1;

	b3RunOverflow(part, context, OVERFLOW_PREPARE, hasOverflow);
	if (main) {
		world.profile.prepareConstraints += b3GetMillisecondsAndReset(ticks.value);
	}

	let subStepCount = context.subStepCount;
	for (let subStepIndex = 0; subStepIndex < subStepCount; ++subStepIndex) {
		let iterationStageIndex = stageIndex;

		// Integrate velocities
		b3RunStage(part, stages[iterationStageIndex], context);
		iterationStageIndex += 1;
		if (main) {
			world.profile.integrateVelocities += b3GetMillisecondsAndReset(ticks.value);
		}

		// Warm start constraints
		b3RunOverflow(part, context, OVERFLOW_WARM_START, hasOverflow);
		for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
			b3RunStage(part, stages[iterationStageIndex], context);
			iterationStageIndex += 1;
		}
		if (main) {
			world.profile.warmStart += b3GetMillisecondsAndReset(ticks.value);
		}

		// Solve constraints, ITERATIONS is 1
		b3RunOverflow(part, context, OVERFLOW_SOLVE, hasOverflow);
		for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
			b3RunStage(part, stages[iterationStageIndex], context);
			iterationStageIndex += 1;
		}
		if (main) {
			world.profile.solveImpulses += b3GetMillisecondsAndReset(ticks.value);
		}

		// Integrate positions
		b3RunStage(part, stages[iterationStageIndex], context);
		iterationStageIndex += 1;
		if (main) {
			world.profile.integratePositions += b3GetMillisecondsAndReset(ticks.value);
		}

		// Relax constraints, RELAX_ITERATIONS is 1
		b3RunOverflow(part, context, OVERFLOW_RELAX, hasOverflow);
		for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
			b3RunStage(part, stages[iterationStageIndex], context);
			iterationStageIndex += 1;
		}
		if (main) {
			world.profile.relaxImpulses += b3GetMillisecondsAndReset(ticks.value);
		}
	}

	stageIndex += 1 + activeColorCount + activeColorCount + 1 + activeColorCount;

	let restitutionIterations = context.world.restitutionIterations;
	if (restitutionIterations > 0 && context.anyRestitution.value != 0) {
		for (let j = 0; j < restitutionIterations; ++j) {
			b3RunOverflow(part, context, OVERFLOW_RESTITUTION, hasOverflow);

			let iterationStageIndex = stageIndex;
			for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
				b3RunStage(part, stages[iterationStageIndex], context);
				iterationStageIndex += 1;
			}
		}
		if (main) {
			world.profile.restitution += b3GetMillisecondsAndReset(ticks.value);
		}
	}

	stageIndex += activeColorCount;

	// Store impulses
	b3RunOverflow(part, context, OVERFLOW_STORE, hasOverflow);
	b3RunStage(part, stages[stageIndex], context);
	stageIndex += 1;
	b3RunStage(part, stages[stageIndex], context);
	if (main) {
		world.profile.storeImpulses += b3GetMillisecondsAndReset(ticks.value);
	}
}

function b3ParallelSolve(context: b3StepContext): bool {
	let world = context.world;
	if (Parallel.size() > world.workerCount) {
		return false;
	}

	// What b3Solve's worker loop clears for each worker it starts
	let jointIdCapacity = b3GetIdCapacity(world.jointIdPool);
	let contactIdCapacity = b3GetIdCapacity(world.contactIdPool);
	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = world.taskContexts[i];
		b3SetBitCountAndClear(taskContext.jointStateBitSet, <u32>jointIdCapacity);
		b3SetBitCountAndClear(taskContext.hitEventBitSet, <u32>contactIdCapacity);
		taskContext.hasHitEvents = false;
	}

	Parallel.runUpTo(s_partLimit, b3ParallelSolverTask, context);
	b3_parallelStats.solves += 1;
	return true;
}

// b3ParallelFor's blocks over the awake contacts, claimed by whichever part asks next. The last part first rebuilds the
// trees when rebuildTrees is set, as C's tree task overlaps the narrow phase.
@parallel.inplace export function b3ParallelCollideTask(
	part: Part,
	context: b3StepContext,
	contactCount: i32,
	blockSize: i32,
	rebuildTrees: bool,
): void {
	let workerIndex = part.index;
	let trace = b3TraceRowAt(workerIndex);
	let traceBegin = b3SpanBegin(trace);
	if (rebuildTrees && part.index == part.count - 1) {
		b3UpdateTreesTaskAt(context.world);
		b3SpanEnd(trace, b3_traceTree, traceBegin);
		traceBegin = b3SpanBegin(trace);
	}

	for (let start = part.claim(contactCount, blockSize); start < contactCount; start = part.claim(contactCount, blockSize)) {
		b3CollideTask(start, min(start + blockSize, contactCount), workerIndex, context);
	}
	b3SpanEnd(trace, b3_traceCollide, traceBegin);
}

// b3ParallelFor's block size for itemCount items of at least minRange each over workerCount workers
function b3ParallelForBlockSize(itemCount: i32, minRange: i32, workerCount: i32): i32 {
	let maxBlockCount = 32 * workerCount;
	if (itemCount <= minRange * maxBlockCount) {
		return minRange;
	}
	return (itemCount + maxBlockCount - 1) / maxBlockCount;
}

unsafe function b3ParallelCollide(context: b3StepContext, contactCount: i32): bool {
	let world = context.world;
	let custom = world.preSolveFcn != null || world.frictionCallback != b3DefaultFrictionCallback ||
		world.restitutionCallback != b3DefaultRestitutionCallback;
	if (custom || Parallel.size() > world.workerCount) {
		return false;
	}

	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = world.taskContexts[i];
		if (taskContext.deferredContacts.length < contactCount) {
			taskContext.deferredContacts = new StaticArray<i32>(contactCount + (contactCount >> 1));
		}
		taskContext.deferredCount = 0;
	}
	b3ReserveMeshScratchForWorkers(world.workerCount);
	b3PrepareCompoundContacts(world);

	let rebuildTrees = s_treesPending;
	if (rebuildTrees) {
		b3ReserveTreesTask(world);
	}

	Parallel.runUpTo(
		s_partLimit,
		b3ParallelCollideTask,
		context,
		contactCount,
		b3ParallelForBlockSize(contactCount, 20, world.workerCount),
		rebuildTrees,
	);

	if (rebuildTrees) {
		s_treesPending = false;
		b3FinishTreesTask(world);
	}

	let stats = b3_parallelStats;
	stats.collides += 1;
	stats.contacts += contactCount;
	let trace = b3TraceRowAt(0);
	let traceBegin = b3SpanBegin(trace);
	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = world.taskContexts[i];
		let deferred = taskContext.deferredContacts;
		stats.deferredContacts += taskContext.deferredCount;
		for (let k = 0; k < taskContext.deferredCount; ++k) {
			let index = deferred[k];
			b3CollideTask(index, index + 1, i, context);
		}
	}
	b3SpanEnd(trace, b3_traceCollide, traceBegin);
	return true;
}

@parallel.inplace export unsafe function b3ParallelFinalizeTask(part: Part, context: b3StepContext, bodyCount: i32, blockSize: i32): void {
	let workerIndex = part.index;
	let trace = b3TraceRowAt(workerIndex);
	let traceBegin = b3SpanBegin(trace);
	for (let start = part.claim(bodyCount, blockSize); start < bodyCount; start = part.claim(bodyCount, blockSize)) {
		b3FinalizeBodiesTask(start, min(start + blockSize, bodyCount), workerIndex, context);
	}
	b3SpanEnd(trace, b3_traceFinalize, traceBegin);
}

unsafe function b3ParallelFinalize(context: b3StepContext, bodyCount: i32): bool {
	let world = context.world;
	if (Parallel.size() > world.workerCount) {
		return false;
	}

	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = world.taskContexts[i];
		if (taskContext.deferredBodies.length < bodyCount) {
			taskContext.deferredBodies = new StaticArray<i32>(bodyCount + (bodyCount >> 1));
		}
		taskContext.deferredBodyCount = 0;
		taskContext.unstableCount = 0;
	}
	context.parallelContinuous = b3PrepareParallelContinuous(world);

	Parallel.runUpTo(s_partLimit, b3ParallelFinalizeTask, context, bodyCount, b3ParallelForBlockSize(bodyCount, 8, world.workerCount));
	b3FinishParallelFinalize(world, context);

	let stats = b3_parallelStats;
	stats.finalizes += 1;
	for (let i = 0; i < world.workerCount; ++i) {
		stats.deferredBodies += world.taskContexts[i].deferredBodyCount;
	}
	return true;
}

// b3UpdateBroadPhasePairs' two parallel fors, the cross tree seeds and then the moved siblings of the dynamic tree
@parallel.inplace export function b3ParallelPairsTask(
	part: Part,
	world: b3World,
	crossContext: b3CrossContext,
	crossCount: i32,
	selfCount: i32,
	selfBlockSize: i32,
): void {
	let workerIndex = part.index;
	let trace = b3TraceRowAt(workerIndex);
	let traceBegin = b3SpanBegin(trace);
	for (let i = part.claim(crossCount, 1); i < crossCount; i = part.claim(crossCount, 1)) {
		b3CrossPairsTask(i, i + 1, workerIndex, crossContext);
	}
	b3SpanEnd(trace, b3_tracePairs, traceBegin);
	part.sync();

	traceBegin = b3SpanBegin(trace);
	for (let start = part.claim(selfCount, selfBlockSize); start < selfCount; start = part.claim(selfCount, selfBlockSize)) {
		b3SelfPairsTask(start, min(start + selfBlockSize, selfCount), workerIndex, world);
	}
	b3SpanEnd(trace, b3_tracePairs, traceBegin);
}

// Room for this many new pairs per part, grown after a step that ran out
let s_pairKeyCapacity = 1024;

function b3ParallelPairs(world: b3World, crossContext: b3CrossContext, crossCount: i32, selfCount: i32): bool {
	// Parts cannot call a custom filter
	if (world.customFilterFcn != null || Parallel.size() > world.workerCount) {
		return false;
	}

	b3PrepareParallelPairs(world, s_pairKeyCapacity);
	Parallel.runUpTo(
		s_partLimit,
		b3ParallelPairsTask,
		world,
		crossContext,
		crossCount,
		selfCount,
		b3ParallelForBlockSize(selfCount, 64, world.workerCount),
	);

	// Finding pairs changes nothing else, so a part that ran out of room means finding them again here
	if (b3CollectParallelPairs(world) < 0) {
		s_pairKeyCapacity *= 4;
		for (let i = 0; i < world.workerCount; ++i) {
			world.taskContexts[i].pairKeys.length = 0;
		}
		return false;
	}
	return true;
}

// The tree rebuild task b3UpdateBroadPhasePairs enqueues, run by the narrow phase's parallel call or else when finished
let s_treesPending = false;
const B3_TREES_TASK: u64 = 1;

function b3ParallelEnqueue(task: b3TaskCallback, taskContext: u64, name: string): u64 {
	if (name == "rebuild tree") {
		s_treesPending = true;
		return B3_TREES_TASK;
	}

	task(taskContext);
	return 0;
}

function b3ParallelFinish(world: b3World, userTask: u64): void {
	if (userTask == B3_TREES_TASK && s_treesPending) {
		s_treesPending = false;
		let trace = b3TraceRowAt(0);
		let traceBegin = b3SpanBegin(trace);
		b3UpdateTreesTaskAt(world);
		b3SpanEnd(trace, b3_traceTree, traceBegin);
	}
}

// The task callbacks the world had before b3World_EnableParallel
const s_savedWorlds = new Array<b3World>();
const s_savedEnqueue = new Array<b3EnqueueTaskCallback>();
const s_savedFinish = new Array<b3FinishTaskCallback>();

// Runs worldId's narrow phase, and its solver stages and body finalization, as tasks picks, on parallel calls. The world gets a task context
// per part, which leaves its results unchanged as C's do at any worker count. tasks 0 turns it off.
export function b3World_EnableParallel(worldId: b3WorldId, tasks: i32): void {
	b3SetLaneSolve(null);
	b3SetLaneCollide(null);
	b3SetLaneFinalize(null);
	b3SetLanePairs(null);
	for (let i = 0; i < s_savedWorlds.length; ++i) {
		s_savedWorlds[i].enqueueTaskFcn = s_savedEnqueue[i];
		s_savedWorlds[i].finishTaskFcn = s_savedFinish[i];
	}
	s_savedWorlds.length = 0;
	s_savedEnqueue.length = 0;
	s_savedFinish.length = 0;
	s_treesPending = false;
	if (tasks == 0) {
		return;
	}

	b3World_SetWorkerCount(worldId, Parallel.size());
	let world = b3GetWorldFromId(worldId);
	if ((tasks & b3_parallelCollide) != 0) {
		b3SetLaneCollide((context: b3StepContext, contactCount: i32): bool => b3ParallelCollide(context, contactCount));
		b3SetLanePairs((world: b3World, crossContext: b3CrossContext, crossCount: i32, selfCount: i32): bool =>
			b3ParallelPairs(world, crossContext, crossCount, selfCount),
		);
		s_savedWorlds.push(world);
		s_savedEnqueue.push(world.enqueueTaskFcn);
		s_savedFinish.push(world.finishTaskFcn);
		world.enqueueTaskFcn = (task: b3TaskCallback, taskContext: u64, userContext: u64, name: string): u64 =>
			b3ParallelEnqueue(task, taskContext, name);
		world.finishTaskFcn = (userTask: u64, userContext: u64): void => b3ParallelFinish(world, userTask);
	}
	if ((tasks & b3_parallelSolve) != 0) {
		b3SetLaneSolve((context: b3StepContext): bool => b3ParallelSolve(context));
		b3SetLaneFinalize((context: b3StepContext, bodyCount: i32): bool => b3ParallelFinalize(context, bodyCount));
	}
}

// Runs later calls on at most parts parts, 0 on as many as Parallel.size()
export function b3SetParallelLimit(parts: i32): void {
	s_partLimit = parts > 0 ? parts : B3_MAX_WORKERS;
}

// Fields for a bench's result line
export function b3ParallelReport(): string {
	let ps = b3_parallelStats;
	return " parts=" + Parallel.size().toString() + " parSolves=" + ps.solves.toString() + " parCollides=" +
		ps.collides.toString() + " parFinalizes=" + ps.finalizes.toString() + " parContacts=" +
		formatFixed(<f64>ps.contacts / <f64>max(1, ps.collides), 1) + " parDeferredContacts=" +
		formatFixed(<f64>ps.deferredContacts / <f64>max(1, ps.collides), 1) + " parDeferredBodies=" +
		formatFixed(<f64>ps.deferredBodies / <f64>max(1, ps.finalizes), 1);
}
