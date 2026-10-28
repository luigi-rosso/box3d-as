// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// What the lane tasks of lanes.as and lane_collide.as share: the byte writer they serialize with, the spin wait and
// the main lane's clock.

import { SharedRegion } from "rive/shared";

import { b3CreateBitSet } from "./bitset";
import { b3TaskContext } from "./physics_world";
import { b3TraceRow, b3TraceRowAt } from "./trace";
import { b3WorldId } from "./types";

export const b3_laneBackendJobs: i32 = 1;
export const b3_laneBackendWorkers: i32 = 2;
// Parallel calls over the world's own memory, from lib/parallel/ when a project includes it
export const b3_laneBackendParallel: i32 = 3;

// What lib/parallel/parallel_step.as registers when a project includes it. Parallel calls need the parallel-calls
// toolchain, so the core lib reaches them only through this.
export class b3ParallelBackend {
	enable(worldId: b3WorldId, tasks: i32): void {}

	// Parts a run gets
	size(): i32 {
		return 1;
	}

	// Runs later calls on at most parts parts, 0 on as many as size()
	limit(parts: i32): void {}

	// Fields for a bench's result line
	report(): string {
		return "";
	}
}

let b3_parallelBackend: b3ParallelBackend? = null;

export function b3SetParallelBackend(backend: b3ParallelBackend?): void {
	b3_parallelBackend = backend;
}

export function b3GetParallelBackend(): b3ParallelBackend? {
	return b3_parallelBackend;
}

export const B3_MAX_LANES: i32 = 8;

// Only the main lane times its partition for the profile: the clock is set there, and a lane sees the initial null.
export type b3LaneClock = () => f64;

let b3_laneClock: b3LaneClock? = null;

export function b3SetLaneClock(clock: b3LaneClock?): void {
	b3_laneClock = clock;
}

// Milliseconds on the main lane, 0 on the others
@inline export function b3LaneNow(): f64 {
	let clock = b3_laneClock;
	return clock != null ? clock() : 0.0;
}

// A growable byte buffer written with raw stores. Offsets stay valid across growth; addresses do not.
export class b3LaneWriter {
	buffer: Uint8Array = new Uint8Array(1 << 16);
	length: i32 = 0;

	reset(): void {
		this.length = 0;
	}

	// The address of byteCount fresh bytes, good until the next reserve
	unsafe reserve(byteCount: i32): usize {
		let offset = this.length;
		let need = offset + byteCount;
		if (need > this.buffer.length) {
			let capacity = this.buffer.length;
			while (capacity < need) {
				capacity *= 2;
			}
			let grown = new Uint8Array(capacity);
			grown.set(this.buffer.subarray(0, offset));
			this.buffer = grown;
		}
		this.length = need;
		return this.buffer.dataStart + <usize>offset;
	}

	@inline unsafe u32(value: u32): void {
		store<u32>(this.reserve(4), value);
	}

	@inline unsafe i32(value: i32): void {
		store<i32>(this.reserve(4), value);
	}

	@inline unsafe f32(value: f32): void {
		store<f32>(this.reserve(4), value);
	}

	@inline unsafe bytes(source: usize, byteCount: i32): void {
		memory.copy(this.reserve(byteCount), source, <usize>byteCount);
	}

	@inline unsafe patch(offset: i32, value: u32): void {
		store<u32>(this.buffer.dataStart + <usize>offset, value);
	}
}

@inline export function b3GrowU8(a: Uint8Array, byteCount: i32): Uint8Array {
	return a.length >= byteCount ? a : new Uint8Array(byteCount + (byteCount >> 2) + 64);
}

@inline export function b3GrowI32(a: Int32Array, count: i32): Int32Array {
	return a.length >= count ? a : new Int32Array(count + (count >> 2) + 16);
}

@inline export function b3GrowU32(a: Uint32Array, count: i32): Uint32Array {
	return a.length >= count ? a : new Uint32Array(count + (count >> 2) + 16);
}

@inline export unsafe function b3ReadU32(base: usize, offset: i32): u32 {
	return load<u32>(base + <usize>offset);
}

@inline export unsafe function b3ReadI32(base: usize, offset: i32): i32 {
	return load<i32>(base + <usize>offset);
}

@inline export unsafe function b3ReadF32(base: usize, offset: i32): f32 {
	return load<f32>(base + <usize>offset);
}

@inline export unsafe function b3HeaderI32(h: Uint32Array, index: i32): i32 {
	return load<i32>(h.dataStart + <usize>index * 4);
}

@inline export unsafe function b3HeaderF32(h: Uint32Array, index: i32): f32 {
	return load<f32>(h.dataStart + <usize>index * 4);
}

// The first of count items lane p of lanes takes: an even split in item order
@inline export function b3SplitStart(count: i32, p: i32, lanes: i32): i32 {
	return <i32>((<i64>count * <i64>p) / <i64>lanes);
}

// Spins until a lane's word reaches value, counting the spins. A lane that never gets there is a bug, which this
// reports and gives up on after about a minute rather than hang.
export function b3LaneAwait(region: SharedRegion, offset: u32, value: u32): u64 {
	let spins: u64 = 0;
	while (region.atomicLoad(offset) < value) {
		spins += 1;
		if (spins == 4000000000) {
			console.log("b3LaneAwait: gave up on region " + region.id.toString() + " word " + offset.toString() + " at " +
				region.atomicLoad(offset).toString() + " for " + value.toString());
			break;
		}
	}
	return spins;
}

export function b3MakeLaneTaskContext(): b3TaskContext {
	let taskContext = new b3TaskContext();
	taskContext.hitEventBitSet = b3CreateBitSet(1024);
	taskContext.jointStateBitSet = b3CreateBitSet(64);
	taskContext.contactStateBitSet = b3CreateBitSet(1024);
	taskContext.awakeIslandBitSet = b3CreateBitSet(64);
	return taskContext;
}

// A lane's trace spans travel back in a region: whether the main lane traces, the span count, then kind, begin and end
// per span as f64
const LANE_TRACE_SPANS: i32 = 128;
export const B3_LANE_TRACE_BYTES: u32 = 8 + 24 * <u32>LANE_TRACE_SPANS;

const b3_laneTraceRow = new b3TraceRow();
const b3_laneTracePacked = new Float64Array(3 * LANE_TRACE_SPANS);

// Before a command: whether the lane records into region
export function b3LaneTraceArm(region: SharedRegion): void {
	region.atomicStore(4, 0);
	region.atomicStore(0, b3TraceRowAt(0) != null ? 1 : 0);
}

// On the lane: its row for this command, or null when the main lane is not tracing
export function b3LaneTraceOpen(region: SharedRegion): b3TraceRow? {
	if (region.atomicLoad(0) == 0) {
		return null;
	}
	b3_laneTraceRow.count = 0;
	return b3_laneTraceRow;
}

// On the lane, before it signals it is done
export function b3LaneTraceSend(region: SharedRegion, row: b3TraceRow?): void {
	if (row == null) {
		return;
	}
	let count = min(row.count, LANE_TRACE_SPANS);
	let packed = b3_laneTracePacked;
	for (let i = 0; i < count; ++i) {
		packed[3 * i] = <f64>row.kinds[i];
		packed[3 * i + 1] = row.begins[i];
		packed[3 * i + 2] = row.ends[i];
	}
	region.exclusive((view) => {
		view.writeFrom(8, packed, 0, 3 * count);
	});
	region.atomicStore(4, <u32>count);
}

// After a command: what the lane sent, into the main lane's row rowIndex
export function b3LaneTraceGather(region: SharedRegion, rowIndex: i32): void {
	let row = b3TraceRowAt(rowIndex);
	if (row == null) {
		return;
	}
	let count = <i32>region.atomicLoad(4);
	let packed = b3_laneTracePacked;
	region.exclusive((view) => {
		view.readInto(8, packed, 0, 3 * count);
	});
	for (let i = 0; i < count; ++i) {
		row.add(<i32>packed[3 * i], packed[3 * i + 1], packed[3 * i + 2]);
	}
	region.atomicStore(4, 0);
}
