// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box3d/base.h, src/core.h and src/core.c

import { b3IsValidFloat } from "./math_functions";

// <float.h> and <stdint.h> limits used across the port
export const FLT_MAX: f32 = reinterpret<f32>(0x7F7FFFFF);
export const FLT_MIN: f32 = reinterpret<f32>(0x00800000);
export const FLT_EPSILON: f32 = reinterpret<f32>(0x34000000);
export const INT32_MAX: i32 = 2147483647;
export const INT32_MIN: i32 = -2147483648;
export const UINT8_MAX: i32 = 255;
export const UINT16_MAX: i32 = 65535;
export const UINT32_MAX: u32 = 0xFFFFFFFF;
export const UINT64_MAX: u64 = 0xFFFFFFFFFFFFFFFF;

// base.h

export const B3_ENABLE_VALIDATION: bool = false;

/// This is used to indicate null for interfaces that work with indices instead of pointers
export const B3_NULL_INDEX: i32 = -1;

/// Prototype for user allocation function. Allocation is owned by the runtime in the port.
export type b3AllocFcn = (size: i32, alignment: i32) => u64;

/// Prototype for user free function.
export type b3FreeFcn = (mem: u64, size: i32) => void;

/// Prototype for the user assert callback. Return 0 to skip the debugger break.
export type b3AssertFcn = (condition: string, fileName: string, lineNumber: i32) => i32;

/// Prototype for user log callback. Used to log warnings.
export type b3LogFcn = (message: string) => void;

/// The runtime owns memory, so the hooks are accepted and ignored.
export function b3SetAllocator(allocFcn: b3AllocFcn?, freeFcn: b3FreeFcn?): void {}

/// Total bytes allocated by Box3D. The runtime owns memory, so this reads 0.
export function b3GetByteCount(): i64 {
	return 0;
}

function b3DefaultAssertFcn(_condition: string, _fileName: string, _lineNumber: i32): i32 {
	// return non-zero to break to debugger
	return 1;
}

let b3AssertHandler: b3AssertFcn = b3DefaultAssertFcn;

/// Override the default assert callback.
///	@param assertFcn a non-null assert callback
export function b3SetAssertFcn(assertFcn: b3AssertFcn): void {
	b3AssertHandler = assertFcn;
}

/// Internal assertion handler. Allows for host intervention.
export function b3InternalAssert(condition: string, fileName: string, lineNumber: i32): i32 {
	let result = b3AssertHandler(condition, fileName, lineNumber);
	if (result != 0) {
		unreachable();
	}
	return result;
}

/// True when built with -D B3_ENABLE_ASSERT. Folds at compile time, so
/// if (B3_ASSERT_ENABLED) B3_ASSERT(x) costs nothing in a release build.
export const B3_ASSERT_ENABLED: bool = isDefined(B3_ENABLE_ASSERT);

/// Assert that a condition is true. Compiled out unless built with -D B3_ENABLE_ASSERT.
/// The argument is still evaluated under -O, so guard calls in hot paths with B3_ASSERT_ENABLED.
@inline
export function B3_ASSERT(condition: bool): void {
	if (B3_ASSERT_ENABLED) {
		if (!condition) {
			b3InternalAssert("", "", 0);
		}
	}
}

function b3DefaultLogFcn(message: string): void {}

let b3LogHandler: b3LogFcn = b3DefaultLogFcn;

/// Override the default logging callback.
export function b3SetLogFcn(logFcn: b3LogFcn): void {
	b3LogHandler = logFcn;
}

/// Version numbering scheme.
/// See https://semver.org/
export struct b3Version {
	/// Significant changes
	major: i32 = 0;

	/// Incremental changes
	minor: i32 = 0;

	/// Bug fixes
	revision: i32 = 0;

	constructor(major: i32, minor: i32, revision: i32) {
		this.major = major;
		this.minor = minor;
		this.revision = revision;
	}
}

/// @return true if the current CPU has the AVX2 instruction set.
export function b3IsAVX2Available(): bool {
	return false;
}

/// Get the absolute number of system ticks. Profiling timers are skipped, so this reads 0.
export function b3GetTicks(): u64 {
	return 0;
}

/// Get the milliseconds passed from an initial tick value.
export function b3GetMilliseconds(ticks: u64): f32 {
	return 0.0;
}

/// Get the milliseconds passed from an initial tick value.
export function b3GetMillisecondsAndReset(ticks: ref u64): f32 {
	ticks = 0;
	return 0.0;
}

/// Yield to be used in a busy loop.
export function b3Yield(): void {}

/// Sleep the current thread for a number of milliseconds.
export function b3Sleep(milliseconds: i32): void {}

// Simple djb2 hash function for determinism testing
export const B3_HASH_INIT: u32 = 5381;

// Ported from src/timer.c, the only part of that file the port keeps.
export function b3Hash(hash: u32, data: Uint8Array, count: i32): u32 {
	let result = hash;
	let i = 0;

	while (i + 8 <= count) {
		let lo = b3ReadU32(data, i);
		let hi = b3ReadU32(data, i + 4);
		result = (result << 5) + result + lo;
		result = (result << 5) + result + hi;
		i += 8;
	}

	while (i < count) {
		result = (result << 5) + result + <u32>data[i];
		i++;
	}

	return result;
}

// core.h

export const B3_DEBUG: i32 = 0;

export struct b3AtomicInt {
	value: i32 = 0;
}

export struct b3AtomicU32 {
	value: u32 = 0;
}

export struct b3AtomicI64 {
	value: i64 = 0;
}

// Minimum memory alignment used for all allocations
export const B3_ALIGNMENT: i32 = 32;

// Use to validate definitions. Do not take my cookie.
export const B3_SECRET_COOKIE: i32 = 1152023;

export function b3Log(message: string): void {
	b3LogHandler(message);
}

// core.c

// This allows the user to change the length units at runtime
let b3_lengthUnitsPerMeter: f32 = 1.0;

export function b3SetLengthUnitsPerMeter(lengthUnits: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(lengthUnits) && lengthUnits > 0.0);
	b3_lengthUnitsPerMeter = lengthUnits;
}

export function b3GetLengthUnitsPerMeter(): f32 {
	return b3_lengthUnitsPerMeter;
}

export function b3GetVersion(): b3Version {
	return b3Version(0, 2, 0);
}

export function b3IsDoublePrecision(): bool {
	return false;
}

export function b3GetMaxManifoldPoints(): i32 {
	return 4;
}

// rapidhash, compact and fast variant, from src/rapidhash.h

const rapid_secret0: u64 = 0x2d358dccaa6c78a5;
const rapid_secret1: u64 = 0x8bb84b93962eacc9;
const rapid_secret2: u64 = 0x4b33a62ed433d4a3;
const rapid_secret3: u64 = 0x4d5a2da51de1aa47;
const rapid_secret4: u64 = 0xa0761d6478bd642f;
const rapid_secret5: u64 = 0xe7037ed1a0b428db;
const rapid_secret6: u64 = 0x90ed1765281c388c;
const rapid_secret7: u64 = 0xaaaaaaaaaaaaaaaa;

@inline
function b3ReadU32(p: Uint8Array, i: i32): u32 {
	return <u32>p[i] | (<u32>p[i + 1] << 8) | (<u32>p[i + 2] << 16) | (<u32>p[i + 3] << 24);
}

@inline
function rapid_read64(p: Uint8Array, i: i32): u64 {
	return <u64>b3ReadU32(p, i) | (<u64>b3ReadU32(p, i + 4) << 32);
}

@inline
function rapid_read32(p: Uint8Array, i: i32): u64 {
	return <u64>b3ReadU32(p, i);
}

// High 64 bits of the 128 bit product, the portable branch of rapid_mum.
function rapid_mulhi(A: u64, B: u64): u64 {
	let ha = A >> 32;
	let hb = B >> 32;
	let la = A & 0xFFFFFFFF;
	let lb = B & 0xFFFFFFFF;
	let rh = ha * hb;
	let rm0 = ha * lb;
	let rm1 = hb * la;
	let rl = la * lb;
	let t = rl + (rm0 << 32);
	let c: u64 = t < rl ? 1 : 0;
	let lo = t + (rm1 << 32);
	c += lo < t ? 1 : 0;
	return rh + (rm0 >> 32) + (rm1 >> 32) + c;
}

@inline
function rapid_mix(A: u64, B: u64): u64 {
	return (A * B) ^ rapid_mulhi(A, B);
}

function rapidhash(p: Uint8Array, len: i32): u64 {
	let seed: u64 = 0;
	seed ^= rapid_mix(seed ^ rapid_secret2, rapid_secret1);
	let a: u64 = 0;
	let b: u64 = 0;
	let i = len;
	let o = 0;
	if (len <= 16) {
		if (len >= 4) {
			seed ^= <u64>len;
			if (len >= 8) {
				a = rapid_read64(p, 0);
				b = rapid_read64(p, len - 8);
			} else {
				a = rapid_read32(p, 0);
				b = rapid_read32(p, len - 4);
			}
		} else if (len > 0) {
			a = (<u64>p[0] << 45) | <u64>p[len - 1];
			b = <u64>p[len >> 1];
		} else {
			a = 0;
			b = 0;
		}
	} else {
		if (len > 112) {
			let see1 = seed;
			let see2 = seed;
			let see3 = seed;
			let see4 = seed;
			let see5 = seed;
			let see6 = seed;
			do {
				seed = rapid_mix(rapid_read64(p, o) ^ rapid_secret0, rapid_read64(p, o + 8) ^ seed);
				see1 = rapid_mix(rapid_read64(p, o + 16) ^ rapid_secret1, rapid_read64(p, o + 24) ^ see1);
				see2 = rapid_mix(rapid_read64(p, o + 32) ^ rapid_secret2, rapid_read64(p, o + 40) ^ see2);
				see3 = rapid_mix(rapid_read64(p, o + 48) ^ rapid_secret3, rapid_read64(p, o + 56) ^ see3);
				see4 = rapid_mix(rapid_read64(p, o + 64) ^ rapid_secret4, rapid_read64(p, o + 72) ^ see4);
				see5 = rapid_mix(rapid_read64(p, o + 80) ^ rapid_secret5, rapid_read64(p, o + 88) ^ see5);
				see6 = rapid_mix(rapid_read64(p, o + 96) ^ rapid_secret6, rapid_read64(p, o + 104) ^ see6);
				o += 112;
				i -= 112;
			} while (i > 112);
			seed ^= see1;
			see2 ^= see3;
			see4 ^= see5;
			seed ^= see6;
			see2 ^= see4;
			seed ^= see2;
		}
		if (i > 16) {
			seed = rapid_mix(rapid_read64(p, o) ^ rapid_secret2, rapid_read64(p, o + 8) ^ seed);
			if (i > 32) {
				seed = rapid_mix(rapid_read64(p, o + 16) ^ rapid_secret2, rapid_read64(p, o + 24) ^ seed);
				if (i > 48) {
					seed = rapid_mix(rapid_read64(p, o + 32) ^ rapid_secret1, rapid_read64(p, o + 40) ^ seed);
					if (i > 64) {
						seed = rapid_mix(rapid_read64(p, o + 48) ^ rapid_secret1, rapid_read64(p, o + 56) ^ seed);
						if (i > 80) {
							seed = rapid_mix(rapid_read64(p, o + 64) ^ rapid_secret2, rapid_read64(p, o + 72) ^ seed);
							if (i > 96) {
								seed = rapid_mix(rapid_read64(p, o + 80) ^ rapid_secret1, rapid_read64(p, o + 88) ^ seed);
							}
						}
					}
				}
			}
		}
		a = rapid_read64(p, o + i - 16) ^ <u64>i;
		b = rapid_read64(p, o + i - 8);
	}
	a ^= rapid_secret1;
	b ^= seed;
	let lo = a * b;
	let hi = rapid_mulhi(a, b);
	return rapid_mix(lo ^ rapid_secret7, hi ^ rapid_secret1 ^ <u64>i);
}

export function b3Hash64NonZero(bytes: Uint8Array, n: i32): u64 {
	if (n <= 0) {
		return 1;
	}

	let h = rapidhash(bytes, n);
	return h == 0 ? 1 : h;
}

/// C float to int casts on the reference target saturate and send NaN to zero,
/// where a wasm cast traps. Use this wherever C could see such a value.
@inline
export function b3FloatToIntSat(a: f32): i32 {
	if (isNaN(a)) {
		return 0;
	}
	if (a >= 2147483648.0) {
		return INT32_MAX;
	}
	if (a <= -2147483648.0) {
		return INT32_MIN;
	}
	return <i32>a;
}

/// Unsigned variant of b3FloatToIntSat.
@inline
export function b3FloatToUIntSat(a: f32): u32 {
	if (isNaN(a) || a <= 0.0) {
		return 0;
	}
	if (a >= 4294967296.0) {
		return UINT32_MAX;
	}
	return <u32>a;
}
