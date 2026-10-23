// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/parallel_for.c

// Callback invoked by b3ParallelFor to process a range of items. May be called
// multiple times per worker: work is divided into blocks that workers claim
// atomically, so a worker that finishes early picks up the next unclaimed
// block instead of sitting idle. workerIndex is the worker identity and is
// stable across all invocations from the same worker, so it is safe to use as
// an index into per-worker state (e.g. world.taskContexts[workerIndex]).
// The closure carries C's context.
export type b3ParallelForCallback = (startIndex: i32, endIndex: i32, workerIndex: i32) => void;

// functions from parallel_for.c follow
