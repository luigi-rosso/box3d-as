// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Not in C: when each task ran on each worker over a step, for a timeline. Off unless b3EnableTrace turns it on, which
// leaves each traced task one check.

import { clock } from "rive/os";

export const B3_TRACE_ROWS: i32 = 16;
export const B3_TRACE_SPANS: i32 = 512;

export const b3_tracePairs: i32 = 0;
export const b3_traceTree: i32 = 1;
export const b3_traceCollide: i32 = 2;
export const b3_tracePrepare: i32 = 3;
export const b3_traceIntegrateVelocities: i32 = 4;
export const b3_traceWarmStart: i32 = 5;
export const b3_traceSolve: i32 = 6;
export const b3_traceIntegratePositions: i32 = 7;
export const b3_traceRelax: i32 = 8;
export const b3_traceRestitution: i32 = 9;
export const b3_traceStore: i32 = 10;
export const b3_traceFinalize: i32 = 11;
export const b3_traceBullets: i32 = 12;
export const b3_traceSensors: i32 = 13;
// Copying step state to or from a lane
export const b3_traceCopy: i32 = 14;
export const b3_traceKindCount: i32 = 15;

// One worker's spans, in seconds on clock()
export class b3TraceRow {
	count: i32 = 0;
	kinds: Uint8Array = new Uint8Array(B3_TRACE_SPANS);
	begins: Float64Array = new Float64Array(B3_TRACE_SPANS);
	ends: Float64Array = new Float64Array(B3_TRACE_SPANS);

	add(kind: i32, begin: f64, end: f64): void {
		let n = this.count;
		if (n < B3_TRACE_SPANS) {
			this.kinds[n] = <u8>kind;
			this.begins[n] = begin;
			this.ends[n] = end;
			this.count = n + 1;
		}
	}
}

function b3MakeTraceRows(): StaticArray<b3TraceRow> {
	let rows = new StaticArray<b3TraceRow>(B3_TRACE_ROWS);
	for (let i = 0; i < B3_TRACE_ROWS; ++i) {
		rows[i] = new b3TraceRow();
	}
	return rows;
}

// A row per worker, and each part writes its own
@parallel.shared const b3_traceRows = b3MakeTraceRows();
let b3_traceOn = false;
let b3_traceBegin: f64 = 0;
let b3_traceEnd: f64 = 0;

export function b3EnableTrace(on: bool): void {
	b3_traceOn = on;
}

export function b3IsTracing(): bool {
	return b3_traceOn;
}

// Clears every row for the step about to run
export function b3TraceBegin(): void {
	if (!b3_traceOn) {
		return;
	}
	for (let i = 0; i < B3_TRACE_ROWS; ++i) {
		b3_traceRows[i].count = 0;
	}
	b3_traceBegin = clock();
	b3_traceEnd = b3_traceBegin;
}

export function b3TraceEnd(): void {
	if (b3_traceOn) {
		b3_traceEnd = clock();
	}
}

export function b3GetTraceBegin(): f64 {
	return b3_traceBegin;
}

export function b3GetTraceEnd(): f64 {
	return b3_traceEnd;
}

export function b3GetTraceRow(index: i32): b3TraceRow {
	return b3_traceRows[index];
}

// Where worker index records, or null while tracing is off
@inline export function b3TraceRowAt(index: i32): b3TraceRow? {
	return b3_traceOn && index < B3_TRACE_ROWS ? b3_traceRows[index] : null;
}

@inline export function b3SpanBegin(row: b3TraceRow?): f64 {
	return row != null ? clock() : 0.0;
}

@inline export function b3SpanEnd(row: b3TraceRow?, kind: i32, begin: f64): void {
	if (row != null) {
		row.add(kind, begin, clock());
	}
}

// b3SolverStageType to span kind: the three prepare stages and the two store stages share one
export function b3TraceStageKind(type: i32): i32 {
	if (type <= 2) {
		return b3_tracePrepare;
	}
	if (type >= 9) {
		return b3_traceStore;
	}
	return type + 1;
}

// A b3ParallelFor's span kind, by the name it gives its tasks
export function b3TraceTaskKind(name: string): i32 {
	if (name == "collide") {
		return b3_traceCollide;
	}
	if (name == "ccd") {
		return b3_traceFinalize;
	}
	if (name == "bullets") {
		return b3_traceBullets;
	}
	if (name == "sensors") {
		return b3_traceSensors;
	}
	return b3_tracePairs;
}
