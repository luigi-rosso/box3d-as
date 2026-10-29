// What the Threads scene steps on and what its panel shows, shared by the scene, the shell and the graph script.

import { Job, Jobs } from "rive/jobs";
import { clock } from "rive/os";
import { Parallel } from "rive/parallel";
import { b3World_EnableLanes, b3_laneBackendJobs, b3_laneCollide, b3_laneSolve, b3_laneTree } from "../lib/lanes";
import { b3SetParallelLimit, b3World_EnableParallel } from "../lib/parallel_step";
import { b3WorldId, b3World_IsValid, b3World_SetWorkerCount } from "../lib/box3d";

export const BACKEND_SERIAL: i32 = 0;
export const BACKEND_LANES: i32 = 1;
export const BACKEND_PARALLEL: i32 = 2;
export const BACKEND_NAMES: StaticArray<string> = ["Serial", "Lanes", "Parallel"];

export const THREAD_CHOICES: StaticArray<i32> = [1, 2, 4, 8];

// Five seconds of steps
export const HISTORY: i32 = 300;

const NO_THREADS = "This runtime has no threads.";
const CHECKING = "Checking for parallel calls.";

@job export function threadsClock(): f64 {
    return clock();
}

export class ThreadsState {
    // The Threads scene is open
    open: bool = false;
    // What the panel asked for; -1 picks the best backend there is
    wantBackend: i32 = -1;
    wantThreads: i32 = 8;
    // What the world steps on now
    backend: i32 = BACKEND_SERIAL;
    threads: i32 = 1;
    applied: bool = false;
    // Threads each backend can have, and why not when it has one
    lanesMax: i32 = 1;
    parallelMax: i32 = 1;
    lanesReason: string = "";
    parallelReason: string = "";

    // Parallel.size() traps on runtimes from before parallel calls, which give a script no way to ask. The threaded web
    // runtimes released before them (2.47.0 to 2.48.0) also ran a job's clock from its pool thread's start, hundreds of
    // ms behind the page's, so a job that reads behind the main lane marks one. Read with peek, since a join could run
    // the job here.
    clockJob: Job<f64>? = null;
    clockAsked: f64 = 0;

    // Step milliseconds, oldest first from head once count reaches HISTORY
    history: Float64Array = new Float64Array(HISTORY);
    historyBackend: Uint8Array = new Uint8Array(HISTORY);
    historyThreads: Uint8Array = new Uint8Array(HISTORY);
    historyHead: i32 = 0;
    historyCount: i32 = 0;
    stepMs: f64 = 0;
    // The serial step, smoothed, as last measured this session
    serialMs: f64 = 0;

    constructor() {
        // For captures, which cannot press the panel's buttons before the frames they skip
        if (isDefined(THREADS_BACKEND)) {
            this.wantBackend = THREADS_BACKEND;
        }
        if (isDefined(THREADS_COUNT)) {
            this.wantThreads = THREADS_COUNT;
        }
    }

    // Starts the job detect reads, early so it is done by the time the scene opens
    probe(): void {
        if (this.clockJob == null && Jobs.threads() > 0) {
            this.clockAsked = clock();
            this.clockJob = Jobs.run(threadsClock);
        }
    }

    // Parallel.size() answers 1 in top level code, so this runs when the scene opens
    detect(): void {
        this.probe();
        let jobThreads = <i32>Jobs.threads();
        // A lane per job thread plus the main one
        this.lanesMax = min(8, jobThreads + 1);
        this.lanesReason = jobThreads == 0 ? NO_THREADS : "";
        this.parallelMax = 1;
        let clocked = this.clockJob != null ? this.clockJob!.peek() : null;
        if (jobThreads == 0) {
            this.parallelReason = NO_THREADS;
        } else if (clocked == null) {
            this.parallelReason = CHECKING;
        } else if (!clocked.ok || clocked.value < this.clockAsked - 0.005) {
            this.parallelReason = "This runtime has no parallel calls.";
        } else {
            this.parallelMax = min(8, Parallel.size());
            this.parallelReason = this.parallelMax == 1 ? "Parallel calls get one part here." : "";
        }
    }

    maxThreads(backend: i32): i32 {
        if (backend == BACKEND_LANES) {
            return this.lanesMax;
        }
        if (backend == BACKEND_PARALLEL) {
            return this.parallelMax;
        }
        return 1;
    }

    available(backend: i32): bool {
        return backend == BACKEND_SERIAL || this.maxThreads(backend) > 1;
    }

    // The largest choice up to what was asked for that the backend has
    threadsFor(backend: i32, want: i32): i32 {
        let most = min(want, this.maxThreads(backend));
        let pick = 1;
        for (let i = 0; i < THREAD_CHOICES.length; i++) {
            if (THREAD_CHOICES[i] <= most) {
                pick = THREAD_CHOICES[i];
            }
        }
        return pick;
    }

    targetBackend(): i32 {
        let want = this.wantBackend;
        if (want >= 0 && this.available(want)) {
            return want;
        }
        if (this.available(BACKEND_PARALLEL)) {
            return BACKEND_PARALLEL;
        }
        return this.available(BACKEND_LANES) ? BACKEND_LANES : BACKEND_SERIAL;
    }

    // Puts the world on the backend asked for, between steps
    apply(worldId: b3WorldId): void {
        if (this.parallelReason == CHECKING) {
            this.detect();
        }
        let backend = this.targetBackend();
        let threads = this.threadsFor(backend, this.wantThreads);
        if (this.applied && backend == this.backend && threads == this.threads) {
            return;
        }
        this.stop(worldId);
        if (backend == BACKEND_LANES) {
            b3World_EnableLanes(worldId, threads, b3_laneBackendJobs, b3_laneCollide | b3_laneSolve | b3_laneTree);
        } else if (backend == BACKEND_PARALLEL) {
            b3SetParallelLimit(threads);
            b3World_EnableParallel(worldId, b3_laneCollide | b3_laneSolve);
        }
        this.backend = backend;
        this.threads = threads;
        this.applied = true;
        this.stepMs = 0;
    }

    // Back to the serial path, which the lane hooks being global leaves to whoever enabled them
    stop(worldId: b3WorldId): void {
        this.applied = false;
        if (!b3World_IsValid(worldId)) {
            return;
        }
        b3World_EnableParallel(worldId, 0);
        b3SetParallelLimit(0);
        b3World_EnableLanes(worldId, 0, b3_laneBackendJobs, 0);
        b3World_SetWorkerCount(worldId, 1);
    }

    // What the backend does, then why the others or more threads are off
    note(backend: i32): string {
        let text = backend == BACKEND_SERIAL ? "All on the main thread."
            : backend == BACKEND_LANES ? "Jobs step copies of the world; accent spans are the copying."
            : "Parts share the world's memory; nothing is copied.";
        if (this.lanesReason.length > 0 && this.lanesReason == this.parallelReason) {
            return text + " " + this.lanesReason;
        }
        if (this.lanesReason.length > 0) {
            text += " " + this.lanesReason;
        }
        if (this.parallelReason.length > 0) {
            text += " " + this.parallelReason;
        }
        let most = this.maxThreads(backend);
        if (backend != BACKEND_SERIAL && most < 8) {
            text += " Up to " + most.toString() + " here.";
        }
        return text;
    }

    record(ms: f64): void {
        let at = this.historyHead;
        this.history[at] = ms;
        this.historyBackend[at] = <u8>this.backend;
        this.historyThreads[at] = <u8>this.threads;
        this.historyHead = (at + 1) % HISTORY;
        this.historyCount = min(this.historyCount + 1, HISTORY);
        this.stepMs = this.stepMs == 0 ? ms : this.stepMs * 0.95 + ms * 0.05;
        if (this.backend == BACKEND_SERIAL) {
            this.serialMs = this.serialMs == 0 ? ms : this.serialMs * 0.95 + ms * 0.05;
        }
    }

    clearHistory(): void {
        this.historyHead = 0;
        this.historyCount = 0;
        this.stepMs = 0;
    }
}

export const g_threads = new ThreadsState();
