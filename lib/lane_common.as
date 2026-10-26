// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// What the lane tasks of lanes.as and lane_collide.as share: the byte writer they serialize with, the spin wait and
// the main lane's clock.

import { SharedRegion } from "rive/shared";

import { b3CreateBitSet } from "./bitset";
import { b3TaskContext } from "./physics_world";

export const b3_laneBackendJobs: i32 = 1;
export const b3_laneBackendWorkers: i32 = 2;
// Parallel calls over the world's own memory, lib/parallel_step.as
export const b3_laneBackendParallel: i32 = 3;

export const B3_MAX_LANES: i32 = 8;

// Jobs and workers have no clock, so only the main lane times its partition: the clock is set there, and a lane
// sees the initial null.
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
