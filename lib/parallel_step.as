// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Box3D's threaded step on parallel calls: the solver stages and the narrow phase run as parts of one call over the
// world's own memory, as C's workers do. notes/threading.md has the design and the costs.

import { Parallel, Part } from "rive/parallel";

import { b3InParallel, b3OnMain } from "./core";
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
import { b3ReserveMeshScratchForWorkers } from "./mesh_contact";
import { b3ExecuteBlock, b3SetLaneSolve, b3SolverStage, b3StepContext } from "./solver";
import {
	b3CollideTask,
	b3DefaultFrictionCallback,
	b3DefaultRestitutionCallback,
	b3SetLaneCollide,
	b3World_SetWorkerCount,
} from "./physics_world";
import { b3WorldId } from "./types";

export const b3_parallelCollide: i32 = 1;
export const b3_parallelSolve: i32 = 2;

// Runs one stage's blocks on whichever parts claim them, then waits for every part, as C's main thread waits for
// the stage's completion count.
function b3RunStage(part: Part, stage: b3SolverStage, context: b3StepContext): void {
	let blockCount = stage.blockCount;
	if (blockCount == 0) {
		return;
	}

	let blocks = stage.blocks;
	let workerIndex = part.index;
	for (let i = part.claim(blockCount, 1); i < blockCount; i = part.claim(blockCount, 1)) {
		b3ExecuteBlock(stage, context, blocks[i].block, workerIndex);
	}
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

	let stageIndex = 0;

	// Prepare joints, convex contacts and mesh contacts
	b3RunStage(part, stages[stageIndex], context);
	stageIndex += 1;
	b3RunStage(part, stages[stageIndex], context);
	stageIndex += 1;
	b3RunStage(part, stages[stageIndex], context);
	stageIndex += 1;

	b3RunOverflow(part, context, OVERFLOW_PREPARE, hasOverflow);

	let subStepCount = context.subStepCount;
	for (let subStepIndex = 0; subStepIndex < subStepCount; ++subStepIndex) {
		let iterationStageIndex = stageIndex;

		// Integrate velocities
		b3RunStage(part, stages[iterationStageIndex], context);
		iterationStageIndex += 1;

		// Warm start constraints
		b3RunOverflow(part, context, OVERFLOW_WARM_START, hasOverflow);
		for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
			b3RunStage(part, stages[iterationStageIndex], context);
			iterationStageIndex += 1;
		}

		// Solve constraints, ITERATIONS is 1
		b3RunOverflow(part, context, OVERFLOW_SOLVE, hasOverflow);
		for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
			b3RunStage(part, stages[iterationStageIndex], context);
			iterationStageIndex += 1;
		}

		// Integrate positions
		b3RunStage(part, stages[iterationStageIndex], context);
		iterationStageIndex += 1;

		// Relax constraints, RELAX_ITERATIONS is 1
		b3RunOverflow(part, context, OVERFLOW_RELAX, hasOverflow);
		for (let colorIndex = 0; colorIndex < activeColorCount; ++colorIndex) {
			b3RunStage(part, stages[iterationStageIndex], context);
			iterationStageIndex += 1;
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
	}

	stageIndex += activeColorCount;

	// Store impulses
	b3RunOverflow(part, context, OVERFLOW_STORE, hasOverflow);
	b3RunStage(part, stages[stageIndex], context);
	stageIndex += 1;
	b3RunStage(part, stages[stageIndex], context);
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

	Parallel.run(b3ParallelSolverTask, context);
	return true;
}

// b3ParallelFor's blocks over the awake contacts, claimed by whichever part asks next
@parallel.inplace export function b3ParallelCollideTask(part: Part, context: b3StepContext, contactCount: i32, blockSize: i32): void {
	let workerIndex = part.index;
	for (let start = part.claim(contactCount, blockSize); start < contactCount; start = part.claim(contactCount, blockSize)) {
		b3CollideTask<b3InParallel>(start, min(start + blockSize, contactCount), workerIndex, context);
	}
}

// b3ParallelFor's block size for the narrow phase's minRange of 20 and world.workerCount workers
function b3CollideBlockSize(contactCount: i32, workerCount: i32): i32 {
	let minRange = 20;
	let maxBlockCount = 32 * workerCount;
	if (contactCount <= minRange * maxBlockCount) {
		return minRange;
	}
	return (contactCount + maxBlockCount - 1) / maxBlockCount;
}

function b3ParallelCollide(context: b3StepContext, contactCount: i32): bool {
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

	Parallel.run(b3ParallelCollideTask, context, contactCount, b3CollideBlockSize(contactCount, world.workerCount));

	for (let i = 0; i < world.workerCount; ++i) {
		let taskContext = world.taskContexts[i];
		let deferred = taskContext.deferredContacts;
		for (let k = 0; k < taskContext.deferredCount; ++k) {
			let index = deferred[k];
			b3CollideTask<b3OnMain>(index, index + 1, i, context);
		}
	}
	return true;
}

// Runs worldId's solver stages and narrow phase, as tasks picks, on parallel calls. The world gets a task context
// per part, which leaves its results unchanged as C's do at any worker count. tasks 0 turns it off.
export function b3World_EnableParallel(worldId: b3WorldId, tasks: i32): void {
	b3SetLaneSolve(null);
	b3SetLaneCollide(null);
	if (tasks == 0) {
		return;
	}

	b3World_SetWorkerCount(worldId, Parallel.size());
	if ((tasks & b3_parallelCollide) != 0) {
		b3SetLaneCollide((context: b3StepContext, contactCount: i32): bool => b3ParallelCollide(context, contactCount));
	}
	if ((tasks & b3_parallelSolve) != 0) {
		b3SetLaneSolve((context: b3StepContext): bool => b3ParallelSolve(context));
	}
}
