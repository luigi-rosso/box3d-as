// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/parallel_for.c

import { B3_ASSERT, b3AtomicInt } from "./core";
import { B3_MAX_TASKS, B3_MAX_WORKERS } from "./constants";
import { b3World } from "./physics_world";
import { b3IsTracing, b3SpanBegin, b3SpanEnd, b3TraceRowAt, b3TraceTaskKind } from "./trace";

// Callback invoked by b3ParallelFor to process a range of items. May be called
// multiple times per worker: work is divided into blocks that workers claim
// atomically, so a worker that finishes early picks up the next unclaimed
// block instead of sitting idle. workerIndex is the worker identity and is
// stable across all invocations from the same worker, so it is safe to use as
// an index into per-worker state (e.g. world.taskContexts[workerIndex]).
// The closure carries C's context.
export type b3ParallelForCallback = (startIndex: i32, endIndex: i32, workerIndex: i32) => void;

// functions from parallel_for.c follow

// Shared state for one b3ParallelFor invocation. Workers race on nextBlock to
// claim work, so a slow chunk can't strand the other threads.
class b3ParallelForShared {
	nextBlock: b3AtomicInt = b3AtomicInt();
	blockCount: i32 = 0;
	blockSize: i32 = 0;
	itemCount: i32 = 0;
	traceKind: i32 = 0;
	callback: b3ParallelForCallback;

	constructor(callback: b3ParallelForCallback) {
		this.callback = callback;
	}
}

class b3ParallelForTask {
	shared: b3ParallelForShared;
	workerIndex: i32;

	constructor(shared: b3ParallelForShared, workerIndex: i32) {
		this.shared = shared;
		this.workerIndex = workerIndex;
	}
}

function b3ParallelForTrampoline(task: b3ParallelForTask): void {
	let shared = task.shared;
	let workerIndex = task.workerIndex;
	let callback = shared.callback;

	let blockCount = shared.blockCount;
	let blockSize = shared.blockSize;
	let itemCount = shared.itemCount;
	let trace = b3TraceRowAt(workerIndex);
	let traceBegin = b3SpanBegin(trace);

	for (;;) {
		// b3AtomicFetchAddInt
		let blockIndex = shared.nextBlock.value;
		shared.nextBlock.value += 1;
		if (blockIndex >= blockCount) {
			break;
		}

		let start = blockIndex * blockSize;
		let end = start + blockSize;
		if (end > itemCount) {
			end = itemCount;
		}

		callback(start, end, workerIndex);
	}
	b3SpanEnd(trace, shared.traceKind, traceBegin);
}

// Divide [0, itemCount) into blocks and process them with cooperative claiming:
// up to world.workerCount tasks are enqueued, and each task loops, atomically
// claiming the next unclaimed block until the range is drained. Blocks the
// caller until all work is complete. minRange is the minimum block size; block
// size grows once itemCount exceeds 4 * workerCount * minRange so block count
// stays bounded.
export function b3ParallelFor(
	world: b3World,
	callback: b3ParallelForCallback,
	itemCount: i32,
	minRange: i32,
	name: string,
): void {
	if (itemCount <= 0) {
		return;
	}

	B3_ASSERT(minRange > 0);

	let workerCount = world.workerCount;
	B3_ASSERT(0 < workerCount && workerCount <= B3_MAX_WORKERS);

	// Target multiple blocks per worker to reduce thread stalls.
	// block size grows once items exceed maxBlockCount * minRange
	// so the block count stays bounded and per-block sync overhead stays low.
	// Benchmarking shows 32 is optimal for the convex pile benchmark and others.
	// This is a lot different than Box2D because the 3D narrow phase cost
	// varies a lot.
	let blocksPerWorker = 32;
	let maxBlockCount = blocksPerWorker * workerCount;

	let blockSize: i32;
	let blockCount: i32;
	if (itemCount <= minRange * maxBlockCount) {
		blockSize = minRange;
		blockCount = (itemCount + blockSize - 1) / blockSize;
	} else {
		blockSize = (itemCount + maxBlockCount - 1) / maxBlockCount;
		blockCount = (itemCount + blockSize - 1) / blockSize;
	}
	B3_ASSERT(blockCount >= 1);
	B3_ASSERT(blockSize * blockCount >= itemCount);

	// No point enqueueing more tasks than blocks.
	let taskCount = workerCount < blockCount ? workerCount : blockCount;

	let shared = new b3ParallelForShared(callback);
	shared.blockCount = blockCount;
	shared.blockSize = blockSize;
	shared.itemCount = itemCount;
	shared.traceKind = b3IsTracing() ? b3TraceTaskKind(name) : 0;
	shared.nextBlock.value = 0;

	let handles = new StaticArray<u64>(taskCount);
	for (let i = 0; i < taskCount; ++i) {
		const task = new b3ParallelForTask(shared, i);

		if (world.taskCount < B3_MAX_TASKS) {
			handles[i] = world.enqueueTaskFcn(
				(taskContext: u64): void => b3ParallelForTrampoline(task),
				0,
				world.userTaskContext,
				name,
			);
			world.taskCount += 1;
		} else {
			handles[i] = 0;
			b3ParallelForTrampoline(task);
		}
	}

	for (let i = 0; i < taskCount; ++i) {
		if (handles[i] != 0) {
			world.finishTaskFcn(handles[i], world.userTaskContext);
		}
	}
}
