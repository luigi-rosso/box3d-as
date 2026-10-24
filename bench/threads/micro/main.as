// Lane primitive costs on this machine: atomics, spin barriers, region and blob copies, job and port round trips.
// --define=WORKERS=n (default 3) spinning workers join the barrier tests besides the main lane.

import { Layout, Context } from "rive/host";
import { clock } from "rive/os";
import { formatFixed } from "rive/format";
import { Jobs, Job } from "rive/jobs";
import { Worker } from "rive/workers";
import { Port } from "rive/ports";
import { Blob, SharedRegion, Signal } from "rive/shared";

const MB: i32 = 1 << 20;

@job export function emptyJob(x: i32): i32 {
	return x + 1;
}

@job export function readBlobJob(blob: Blob, words: i32): f32 {
	let out = new Float32Array(words);
	blob.readInto(0, out);
	return out[words - 1];
}

@job export function readRegionJob(region: SharedRegion, words: i32): f32 {
	let out = new Float32Array(words);
	region.exclusive((view) => {
		view.readInto(0, out);
	});
	return out[words - 1];
}

// Sense counting barrier on one region: word 0 counts arrivals, word 4 is the generation
function barrierArrive(region: SharedRegion, participants: u32): void {
	let g = region.atomicLoad(4);
	if (region.atomicAdd(0, 1) == participants - 1) {
		region.atomicStore(0, 0);
		region.atomicStore(4, g + 1);
	} else {
		while (region.atomicLoad(4) == g) {}
	}
}

@worker export async function barrierWorker(region: SharedRegion, rounds: i32, participants: i32): Promise<void> {
	for (let r = 0; r < rounds; ++r) {
		barrierArrive(region, <u32>participants);
	}
}

// Each worker spins on the go region and reports on its own flag region
@worker export async function flagWorker(go: SharedRegion, flag: SharedRegion, rounds: i32): Promise<void> {
	for (let r = 1; r <= rounds; ++r) {
		flag.atomicStore(0, <u32>r);
		while (go.atomicLoad(0) < <u32>r) {}
	}
}

// Awaits the signal, then acks through the region, as a lane woken per step would
@worker export async function signalWorker(go: Signal, ack: SharedRegion, rounds: i32): Promise<void> {
	let seen = 0.0;
	for (let r = 1; r <= rounds; ++r) {
		seen = await go.next(seen);
		ack.atomicStore(0, <u32>r);
	}
}

// Sleeps in short naps between polls of the region, as an idle lane without a signal would
@worker export async function napWorker(go: SharedRegion, ack: SharedRegion, rounds: i32): Promise<void> {
	for (let r = 1; r <= rounds; ++r) {
		while (go.atomicLoad(0) < <u32>r) {
			await Worker.sleep(0.0002);
		}
		ack.atomicStore(0, <u32>r);
	}
}

// Locks the region and copies 64 bytes out, rounds times, as an exchange read does
@worker export async function lockWorker(region: SharedRegion, start: SharedRegion, rounds: i32): Promise<void> {
	let out = new Float32Array(16);
	while (start.atomicLoad(0) == 0) {}
	for (let r = 0; r < rounds; ++r) {
		region.exclusive((view) => {
			view.readInto(0, out);
		});
	}
	start.atomicAdd(4, 1);
}

@worker export async function echoWorker(requests: Port<i32>, replies: Port<i32>): Promise<void> {
	while (true) {
		let request = await requests.receive();
		if (request == null) {
			return;
		}
		replies.post(request.value + 1);
	}
}

function report(name: string, seconds: f64, count: i32): void {
	console.log("MICRO " + name + " us=" + formatFixed(seconds * 1e6 / <f64>count, 3) + " count=" + count.toString());
}

function workerCount(): i32 {
	if (isDefined(WORKERS)) {
		return WORKERS;
	}
	return 3;
}

export class Main extends Layout {
	frame: i32 = 0;
	workers: Array<Worker> = new Array<Worker>();
	requests: Port<i32> = Port.create<i32>();
	replies: Port<i32> = Port.create<i32>();
	echo: Worker? = null;

	override init(context: Context): bool {
		console.log("MICRO jobThreads=" + Jobs.threads().toString());
		this.echo = Worker.spawn(echoWorker, this.requests, this.replies);
		return true;
	}

	override advance(seconds: f64): bool {
		this.frame += 1;
		if (this.frame == 2) {
			this.copies();
		} else if (this.frame == 3) {
			this.jobs();
		} else if (this.frame == 4) {
			this.ports();
		} else if (this.frame == 5) {
			this.barrier();
		} else if (this.frame == 6) {
			this.flags();
		} else if (this.frame == 7) {
			this.signals();
		} else if (this.frame == 8) {
			this.naps();
		} else if (this.frame == 9) {
			this.locks(true);
		} else if (this.frame == 10) {
			this.locks(false);
		} else if (this.frame == 11) {
			console.log("MICRO done");
		}
		return true;
	}

	unsafe copies(): void {
		this.copiesOf(1);
		this.copiesOf(8);
	}

	unsafe copiesOf(megabytes: i32): void {
		let tag = "_" + megabytes.toString() + "MB";
		let bytes = megabytes * MB;
		let words = bytes / 4;
		let data = new Float32Array(words);
		for (let i = 0; i < words; ++i) {
			data[i] = <f32>i;
		}
		let other = new Float32Array(words);
		let reps = megabytes == 1 ? 50 : 10;

		let t0 = clock();
		for (let r = 0; r < reps; ++r) {
			memory.copy(other.dataStart, data.dataStart, <usize>bytes);
		}
		report("memory.copy" + tag, (clock() - t0) / <f64>megabytes, reps);

		t0 = clock();
		for (let r = 0; r < reps; ++r) {
			for (let i = 0; i < words; ++i) {
				other[i] = data[i];
			}
		}
		report("elementCopy" + tag, (clock() - t0) / <f64>megabytes, reps);

		t0 = clock();
		let blob = Blob.fromArray(data);
		for (let r = 1; r < reps; ++r) {
			blob.release();
			blob = Blob.fromArray(data);
		}
		report("Blob.fromArray" + tag, (clock() - t0) / <f64>megabytes, reps);

		t0 = clock();
		for (let r = 0; r < reps; ++r) {
			blob.readInto(0, other);
		}
		report("blob.readInto" + tag, (clock() - t0) / <f64>megabytes, reps);

		let region = SharedRegion.create(<u32>bytes);
		t0 = clock();
		for (let r = 0; r < reps; ++r) {
			region.exclusive((view) => {
				view.writeFrom(0, data);
			});
		}
		report("region.writeFrom" + tag, (clock() - t0) / <f64>megabytes, reps);

		t0 = clock();
		for (let r = 0; r < reps; ++r) {
			region.exclusive((view) => {
				view.readInto(0, other);
			});
		}
		report("region.readInto" + tag, (clock() - t0) / <f64>megabytes, reps);

		let small = new Float32Array(16);
		let smallReps = 20000;
		t0 = clock();
		for (let r = 0; r < smallReps; ++r) {
			region.exclusive((view) => {
				view.writeFrom(<u32>((r & 255) * 64), small);
			});
		}
		report("region.writeFrom_64B", clock() - t0, smallReps);

		t0 = clock();
		for (let r = 0; r < smallReps; ++r) {
			blob.readInto(<u32>((r & 255) * 64), small);
		}
		report("blob.readInto_64B", clock() - t0, smallReps);

		let atomicReps = 200000;
		let sum: u32 = 0;
		t0 = clock();
		for (let r = 0; r < atomicReps; ++r) {
			sum += region.atomicLoad(0);
		}
		report("atomicLoad", clock() - t0, atomicReps);

		t0 = clock();
		for (let r = 0; r < atomicReps; ++r) {
			region.atomicAdd(8, 1);
		}
		report("atomicAdd", clock() - t0, atomicReps);
		console.log("MICRO sum=" + sum.toString());
		blob.release();
		region.release();
	}

	jobs(): void {
		let reps = 2000;
		let t0 = clock();
		let acc = 0;
		for (let r = 0; r < reps; ++r) {
			let job = Jobs.run(emptyJob, r);
			let result = job.join();
			if (result != null && result.ok) {
				acc += result.value;
			}
			job.release();
		}
		report("job_run_join_empty", clock() - t0, reps);

		// Fan out then join, as a parallel for would
		let batch = new Array<Job<i32>>();
		let fanReps = 500;
		t0 = clock();
		for (let r = 0; r < fanReps; ++r) {
			for (let k = 0; k < 4; ++k) {
				batch.push(Jobs.run(emptyJob, k));
			}
			for (let k = 0; k < 4; ++k) {
				let result = batch[k].join();
				if (result != null && result.ok) {
					acc += result.value;
				}
				batch[k].release();
			}
			batch.length = 0;
		}
		report("job_fan4_join", clock() - t0, fanReps);

		let words = MB / 4;
		let data = new Float32Array(words);
		data[words - 1] = 7;
		let blob = Blob.fromArray(data);
		let blobReps = 100;
		t0 = clock();
		for (let r = 0; r < blobReps; ++r) {
			let job = Jobs.run(readBlobJob, blob, words);
			job.join();
			job.release();
		}
		report("job_read_blob_1MB", clock() - t0, blobReps);

		let region = SharedRegion.create(<u32>MB);
		region.exclusive((view) => {
			view.writeFrom(0, data);
		});
		t0 = clock();
		for (let r = 0; r < blobReps; ++r) {
			let job = Jobs.run(readRegionJob, region, words);
			job.join();
			job.release();
		}
		report("job_read_region_1MB", clock() - t0, blobReps);

		// The deep copied argument path
		t0 = clock();
		for (let r = 0; r < blobReps; ++r) {
			let job = Jobs.run(sumJob, data);
			job.join();
			job.release();
		}
		report("job_arg_copy_1MB", clock() - t0, blobReps);
		console.log("MICRO acc=" + acc.toString());
		blob.release();
		region.release();
	}

	ports(): void {
		let reps = 2000;
		let t0 = clock();
		for (let r = 0; r < reps; ++r) {
			this.requests.post(r);
			this.replies.receiveWithin(0.5);
		}
		report("port_round_trip", clock() - t0, reps);
	}

	barrier(): void {
		let n = workerCount();
		let participants = n + 1;
		let rounds = 20000;
		let region = SharedRegion.create(64);
		for (let k = 0; k < n; ++k) {
			this.workers.push(Worker.spawn(barrierWorker, region, rounds + 1, participants));
		}
		// The first round waits for every worker to start
		let t0 = clock();
		barrierArrive(region, <u32>participants);
		let t1 = clock();
		for (let r = 0; r < rounds; ++r) {
			barrierArrive(region, <u32>participants);
		}
		report("barrier_" + participants.toString() + "_first", t1 - t0, 1);
		report("barrier_" + participants.toString(), clock() - t1, rounds);
	}

	// Every worker locks one shared region, or each its own
	locks(sharedOne: bool): void {
		let n = workerCount();
		let rounds = 20000;
		let start = SharedRegion.create(64);
		let one = SharedRegion.create(64);
		for (let k = 0; k < n; ++k) {
			let region = sharedOne ? one : SharedRegion.create(64);
			this.workers.push(Worker.spawn(lockWorker, region, start, rounds));
		}
		// Let the workers reach the start line
		let idle = clock();
		while (clock() - idle < 0.05) {}
		let t0 = clock();
		start.atomicStore(0, 1);
		while (start.atomicLoad(4) < <u32>n) {}
		let seconds = clock() - t0;
		report((sharedOne ? "lock_one_region_" : "lock_own_region_") + n.toString(), seconds, rounds);
	}

	signals(): void {
		let rounds = 200;
		let go = Signal.create();
		let ack = SharedRegion.create(64);
		this.workers.push(Worker.spawn(signalWorker, go, ack, rounds));
		let total = 0.0;
		let worst = 0.0;
		for (let r = 1; r <= rounds; ++r) {
			// Let the worker park first, as it would between steps
			let idle = clock();
			while (clock() - idle < 0.002) {}
			let t0 = clock();
			go.notify();
			while (ack.atomicLoad(0) < <u32>r) {}
			let us = (clock() - t0) * 1e6;
			total += us;
			worst = Math.max(worst, us);
		}
		console.log("MICRO signal_wake us=" + formatFixed(total / <f64>rounds, 2) + " max=" + formatFixed(worst, 1));
	}

	naps(): void {
		let rounds = 200;
		let go = SharedRegion.create(64);
		let ack = SharedRegion.create(64);
		this.workers.push(Worker.spawn(napWorker, go, ack, rounds));
		let total = 0.0;
		let worst = 0.0;
		for (let r = 1; r <= rounds; ++r) {
			let idle = clock();
			while (clock() - idle < 0.002) {}
			let t0 = clock();
			go.atomicStore(0, <u32>r);
			while (ack.atomicLoad(0) < <u32>r) {}
			let us = (clock() - t0) * 1e6;
			total += us;
			worst = Math.max(worst, us);
		}
		console.log("MICRO nap_wake us=" + formatFixed(total / <f64>rounds, 2) + " max=" + formatFixed(worst, 1));
	}

	flags(): void {
		let n = workerCount();
		let rounds = 20000;
		let go = SharedRegion.create(64);
		let flags = new Array<SharedRegion>();
		for (let k = 0; k < n; ++k) {
			let flag = SharedRegion.create(64);
			flags.push(flag);
			this.workers.push(Worker.spawn(flagWorker, go, flag, rounds));
		}
		let t0 = clock();
		for (let r = 1; r <= rounds; ++r) {
			for (let k = 0; k < n; ++k) {
				while (flags[k].atomicLoad(0) < <u32>r) {}
			}
			go.atomicStore(0, <u32>r);
		}
		report("flags_" + (n + 1).toString(), clock() - t0, rounds);
	}
}

@job export function sumJob(values: Float32Array): f32 {
	return values[values.length - 1];
}
