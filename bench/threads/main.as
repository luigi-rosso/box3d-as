// Step timing with Box3D's tasks on lanes, the threaded side of bench/native; bench/as/main.as with lanes.
// LANES=n runs the tasks on n partitions, partition 0 on the main lane (0, the default, leaves the serial path);
// TASKS is a mask of 1 collide, 2 solve and 4 the tree rebuild, 3 (the default) collide and solve; BACKEND=1 jobs, 2 (the default) workers, 3 parallel calls; HASHLOG=1 logs the
// state hash after every step, outside the timing, for verify/threads.sh; REPLICA=1 with LANES=1 sends the main
// lane's solver partition through the serialized path. Each option comes from the options blob run1.sh writes,
// so every run shares one bake and its compiled code, or else from a --define of the same name.
// --define=SCENE=0 pyramid (base 10), 1 large_pyramid (base 100), 2 many_bodies, built here; 3 and up are
// built by the verify/scenes mirror (SCENE_NAMES below, copied in as ./scenes/scenes.as by run.sh):
// 3 chains, 4 plank_pile, 5 mesh_drop, 6 village, 7 queries, 8 capsule_mesh, 9 heightfield, 10 mover,
// 11 card_house, 12 bullets. --define=STEPS=n; --define=STEPLOG logs every step; --define=DELAY=s spins
// s seconds before the first step; IDLEFRAMES=n idles n frames before it. One world step per frame; the scene is built in init and the result
// logged after the last step.

import { Layout, Context } from "rive/host";
import { clock } from "rive/os";
import { formatFixed } from "rive/format";
import { Parallel } from "rive/parallel";
import { b3_parallelStats } from "./lib/parallel_step";
import {
	b3BodyId,
	b3CreateBody,
	b3CreateHullShape,
	b3CreateWorld,
	b3DefaultBodyDef,
	b3DefaultShapeDef,
	b3DefaultWorldDef,
	b3MakeBoxHull,
	b3Profile,
	b3Vec3,
	b3WorldId,
	b3World_EnableSleeping,
	b3World_GetCounters,
	b3World_GetProfile,
	b3World_Step,
	b3_dynamicBody,
	b3Body_GetPosition,
	b3Body_GetRotation,
	b3Body_GetLinearVelocity,
	b3Body_GetAngularVelocity,
	b3Body_IsValid,
} from "./lib/box3d";
// run.sh copies verify/scenes/scenes.as here, the mirror of the native bench's scenes.c
import { Scene, Scene_Create, Scene_Step, SCENE_QUERY_KIND_COUNT } from "./scenes/scenes";
import { b3LaneTasks, b3World_EnableLanes, b3_laneBackendParallel, b3_laneBackendWorkers, b3_laneCollide, b3_laneSolve, T_COUNT } from "./lib/lanes";

const LANE_TIME_NAMES: StaticArray<string> = [
	"setup", "prepare", "integrateVelocities", "warmStart", "solve", "integratePositions", "relax", "restitution",
	"store", "exchange", "firstWait", "output",
];

const g_options = new Map<string, i32>();

function loadOptions(context: Context): void {
	let bytes = context.blob("options");
	if (bytes == null) {
		return;
	}
	let words = String.UTF8.decode(bytes).replaceAll("\n", " ").split(" ");
	for (let i = 0; i < words.length; ++i) {
		let word = words[i];
		let equals = word.indexOf("=");
		if (equals > 0) {
			g_options.set(word.substring(0, equals), I32.parseInt(word.substring(equals + 1)));
		}
	}
}

function option(name: string, fallback: i32): i32 {
	return g_options.has(name) ? g_options.get(name) : fallback;
}

function laneCount(): i32 {
	if (isDefined(LANES)) {
		return option("LANES", LANES);
	}
	return option("LANES", 0);
}

function laneTasks(): i32 {
	if (isDefined(TASKS)) {
		return option("TASKS", TASKS);
	}
	return option("TASKS", b3_laneCollide | b3_laneSolve);
}

function laneBackend(): i32 {
	if (isDefined(BACKEND)) {
		return option("BACKEND", BACKEND);
	}
	return option("BACKEND", b3_laneBackendWorkers);
}

function hashLog(): bool {
	if (isDefined(HASHLOG)) {
		return true;
	}
	return option("HASHLOG", 0) != 0;
}

function replicaOnMain(): bool {
	if (isDefined(REPLICA)) {
		return true;
	}
	return option("REPLICA", 0) != 0;
}

// Every body in creation order, for the state hash
const g_bodies = new Array<b3BodyId>();

// FNV-1a over the float bits of each body's pose and velocities, as verify/scenes/hash.mjs reads them from a
// native recording, so a run on any tier or artifact can be checked bit exact. A destroyed body counts as zeros,
// as the recording writes it.
function stateHash(bodies: Array<b3BodyId>): u32 {
	let h: u32 = 2166136261;
	for (let i = 0; i < bodies.length; ++i) {
		let id = bodies[i];
		if (!b3Body_IsValid(id)) {
			for (let k = 0; k < 13 * 4; ++k) {
				h = h * 16777619;
			}
			continue;
		}
		let p = b3Body_GetPosition(id);
		let q = b3Body_GetRotation(id);
		let v = b3Body_GetLinearVelocity(id);
		let w = b3Body_GetAngularVelocity(id);
		let words: StaticArray<f32> = [p.x, p.y, p.z, q.v.x, q.v.y, q.v.z, q.s, v.x, v.y, v.z, w.x, w.y, w.z];
		for (let k = 0; k < words.length; ++k) {
			let bits = reinterpret<u32>(words[k]);
			for (let b = 0; b < 4; ++b) {
				h = (h ^ ((bits >> (8 * <u32>b)) & 0xff)) * 16777619;
			}
		}
	}
	return h;
}

const SCENE_PYRAMID: i32 = 0;
const SCENE_LARGE_PYRAMID: i32 = 1;
const SCENE_MANY_BODIES: i32 = 2;
// From here on built by verify/scenes, as the native bench builds them
const SCENE_CHAINS: i32 = 3;
const SCENE_NAMES: StaticArray<string> = [
	"chains", "plank_pile", "mesh_drop", "village", "queries", "capsule_mesh", "heightfield", "mover", "card_house",
	"bullets",
];
const QUERY_NAMES: StaticArray<string> = ["qRay", "qRayAll", "qShapeCast", "qOverlap", "qMover"];
const STAGE_COUNT: i32 = 23;
const WARMUP_STEPS: i32 = 1;
const SUBSTEP_COUNT: i32 = 4;

const STAGE_NAMES: StaticArray<string> = [
	"step", "pairs", "collide", "solve", "solverSetup", "constraints", "prepareConstraints",
	"integrateVelocities", "warmStart", "solveImpulses", "integratePositions", "relaxImpulses",
	"restitution", "storeImpulses", "splitIslands", "transforms", "sensorHits", "jointEvents",
	"hitEvents", "refit", "bullets", "sleepIslands", "sensors",
];

function sceneIndex(): i32 {
	if (isDefined(SCENE)) {
		return option("SCENE", SCENE);
	}
	return option("SCENE", SCENE_LARGE_PYRAMID);
}

function sceneName(scene: i32): string {
	if (scene == SCENE_PYRAMID) {
		return "pyramid";
	}
	if (scene == SCENE_LARGE_PYRAMID) {
		return "large_pyramid";
	}
	if (scene >= SCENE_CHAINS) {
		return SCENE_NAMES[scene - SCENE_CHAINS];
	}
	return "many_bodies";
}

function stepCount(scene: i32): i32 {
	if (isDefined(STEPS)) {
		return option("STEPS", STEPS);
	}
	return option("STEPS", scene == SCENE_MANY_BODIES ? 100 : scene >= SCENE_CHAINS ? 300 : 200);
}

function profileToArray(p: b3Profile, out: Float64Array): void {
	out[0] = <f64>p.step;
	out[1] = <f64>p.pairs;
	out[2] = <f64>p.collide;
	out[3] = <f64>p.solve;
	out[4] = <f64>p.solverSetup;
	out[5] = <f64>p.constraints;
	out[6] = <f64>p.prepareConstraints;
	out[7] = <f64>p.integrateVelocities;
	out[8] = <f64>p.warmStart;
	out[9] = <f64>p.solveImpulses;
	out[10] = <f64>p.integratePositions;
	out[11] = <f64>p.relaxImpulses;
	out[12] = <f64>p.restitution;
	out[13] = <f64>p.storeImpulses;
	out[14] = <f64>p.splitIslands;
	out[15] = <f64>p.transforms;
	out[16] = <f64>p.sensorHits;
	out[17] = <f64>p.jointEvents;
	out[18] = <f64>p.hitEvents;
	out[19] = <f64>p.refit;
	out[20] = <f64>p.bullets;
	out[21] = <f64>p.sleepIslands;
	out[22] = <f64>p.sensors;
}

// shared/benchmarks.c CreateLargePyramid, as verify/scenes/scenes.c CreatePyramid
function createPyramid(worldId: b3WorldId, baseCount: i32): void {
	b3World_EnableSleeping(worldId, false);

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, -1.0, 0.0);
		let groundId = b3CreateBody(worldId, bodyDef);
		g_bodies.push(groundId);

		let box = b3MakeBoxHull(400.0, 1.0, 400.0);
		let shapeDef = b3DefaultShapeDef();
		b3CreateHullShape(groundId, shapeDef, box.base);
	}

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	let shapeDef = b3DefaultShapeDef();
	shapeDef.density = 100.0;

	let h: f32 = 0.5;
	let box = b3MakeBoxHull(h, h, h);

	let shift: f32 = 1.0 * h;

	for (let i = 0; i < baseCount; ++i) {
		let y: f32 = (2.0 * <f32>i + 1.0) * shift;

		for (let j = i; j < baseCount; ++j) {
			let x: f32 = (<f32>i + 1.0) * shift + 2.0 * <f32>(j - i) * shift - h * <f32>baseCount;

			bodyDef.position = b3Vec3(x, y, 0.0);

			let bodyId: b3BodyId = b3CreateBody(worldId, bodyDef);
			g_bodies.push(bodyId);
			b3CreateHullShape(bodyId, shapeDef, box.base);
		}
	}
}

// shared/benchmarks.c CreateSmallPyramid
function createSmallPyramid(worldId: b3WorldId, baseCount: i32, extent: f32, centerX: f32, baseZ: f32): void {
	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;

	let shapeDef = b3DefaultShapeDef();
	shapeDef.density = 100.0;

	let box = b3MakeBoxHull(extent, extent, extent);

	for (let i = 0; i < baseCount; ++i) {
		let y: f32 = (2.0 * <f32>i + 1.0) * extent;

		for (let j = i; j < baseCount; ++j) {
			let x: f32 = (<f32>i + 1.0) * extent + 2.0 * <f32>(j - i) * extent + centerX - 0.5;
			bodyDef.position = b3Vec3(x, y, baseZ);

			let bodyId: b3BodyId = b3CreateBody(worldId, bodyDef);
			g_bodies.push(bodyId);
			b3CreateHullShape(bodyId, shapeDef, box.base);
		}
	}
}

// shared/benchmarks.c CreateManyPyramids, release sizes
function createManyPyramids(worldId: b3WorldId): void {
	let baseCount = 10;
	let extent: f32 = 0.5;
	let rowCount = 14;
	let columnCount = 14;
	let groundExtent: f32 = extent * <f32>columnCount * (<f32>baseCount + 1.0);

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, -1.0, 0.0);
		let groundId = b3CreateBody(worldId, bodyDef);
		g_bodies.push(groundId);

		let shapeDef = b3DefaultShapeDef();
		let box = b3MakeBoxHull(groundExtent, 1.0, groundExtent);
		b3CreateHullShape(groundId, shapeDef, box.base);
	}

	let baseWidth: f32 = 2.0 * extent * <f32>baseCount;
	let baseZ: f32 = -groundExtent + 2.0 * extent;
	let deltaZ: f32 = 2.0 * (groundExtent - 2.0 * extent) / (<f32>rowCount - 1.0);

	for (let i = 0; i < rowCount; ++i) {
		for (let j = 0; j < columnCount; ++j) {
			let centerX: f32 = -groundExtent + <f32>j * (baseWidth + 2.0 * extent) + 2.0 * extent;
			createSmallPyramid(worldId, baseCount, extent, centerX, baseZ);
		}

		baseZ += deltaZ;
	}
}

function median(sorted: Float64Array): f64 {
	let n = sorted.length;
	return (n & 1) != 0 ? sorted[n >> 1] : 0.5 * (sorted[(n >> 1) - 1] + sorted[n >> 1]);
}

export class Main extends Layout {
	scene: i32 = 0;
	steps: i32 = 0;
	stepIndex: i32 = 0;
	idleFrames: i32 = 0;
	worldId: b3WorldId = b3WorldId();
	stepMs: Float64Array = new Float64Array(0);
	stageSum: Float64Array = new Float64Array(STAGE_COUNT);
	stageMin: Float64Array = new Float64Array(STAGE_COUNT);
	stages: Float64Array = new Float64Array(STAGE_COUNT);
	createMs: f64 = 0;
	sceneData: Scene = new Scene();
	lanes: b3LaneTasks? = null;
	hashLog: bool = false;

	// The scene's body list follows bodies it creates and destroys as it steps
	hashBodies(): Array<b3BodyId> {
		return this.scene >= SCENE_CHAINS ? this.sceneData.bodies : g_bodies;
	}

	override init(context: Context): bool {
		loadOptions(context);
		this.hashLog = hashLog();
		this.scene = sceneIndex();
		this.steps = stepCount(this.scene);
		this.stepMs = new Float64Array(this.steps - WARMUP_STEPS);

		let t0 = clock();
		if (this.scene >= SCENE_CHAINS) {
			Scene_Create(this.sceneData, sceneName(this.scene), 1);
			this.worldId = this.sceneData.worldId;
			for (let i = 0; i < this.sceneData.bodies.length; ++i) {
				g_bodies.push(this.sceneData.bodies[i]);
			}
		} else {
			let worldDef = b3DefaultWorldDef();
			worldDef.workerCount = 1;
			this.worldId = b3CreateWorld(worldDef);
			if (this.scene == SCENE_PYRAMID) {
				createPyramid(this.worldId, 10);
			} else if (this.scene == SCENE_LARGE_PYRAMID) {
				createPyramid(this.worldId, 100);
			} else {
				createManyPyramids(this.worldId);
			}
		}
		this.createMs = (clock() - t0) * 1000.0;
		if (laneCount() > 0) {
			this.lanes = b3World_EnableLanes(this.worldId, laneCount(), laneBackend(), laneTasks());
			let lanes = this.lanes;
			if (lanes != null) {
				lanes.replicaOnMain = replicaOnMain();
			}
		}
		if (laneCount() > 0 && laneBackend() == b3_laneBackendParallel) {
			console.log("BENCH parallel size=" + Parallel.size().toString());
		}
		console.log("BENCH start scene=" + sceneName(this.scene) + " steps=" + this.steps.toString() +
			" create=" + formatFixed(this.createMs, 2) + "ms");
		return true;
	}

	override advance(seconds: f64): bool {
		if (this.stepIndex >= this.steps) {
			return true;
		}

		// On the web a worker's thread starts only once the page yields, so give it frames before the first step
		if (option("IDLEFRAMES", 0) > this.idleFrames) {
			this.idleFrames += 1;
			return true;
		}

		if (isDefined(DELAY)) {
			// Lets a profiler attach before the first step; profile.sh passes it
			if (this.stepIndex == 0) {
				let start = clock();
				while (clock() - start < <f64>DELAY) {}
			}
		}

		let t0 = clock();
		if (this.scene >= SCENE_CHAINS) {
			Scene_Step(this.sceneData, this.stepIndex);
		} else {
			b3World_Step(this.worldId, 1.0 / 60.0, SUBSTEP_COUNT);
		}
		let ms = (clock() - t0) * 1000.0;
		if (this.hashLog) {
			console.log("HASH step=" + this.stepIndex.toString() + " hash=" + stateHash(this.hashBodies()).toString());
		}

		if (this.stepIndex >= WARMUP_STEPS) {
			this.stepMs[this.stepIndex - WARMUP_STEPS] = ms;
			profileToArray(b3World_GetProfile(this.worldId), this.stages);
			if (isDefined(STEPLOG)) {
				let line = "BENCH STEP " + this.stepIndex.toString() + " ms=" + formatFixed(ms, 3);
				for (let k = 1; k < STAGE_COUNT; ++k) {
					if (this.stages[k] >= 0.5) {
						line += " " + STAGE_NAMES[k] + "=" + formatFixed(this.stages[k], 2);
					}
				}
				console.log(line);
			}
			for (let k = 0; k < STAGE_COUNT; ++k) {
				this.stageSum[k] += this.stages[k];
				this.stageMin[k] = this.stepIndex == WARMUP_STEPS ? this.stages[k] : Math.min(this.stageMin[k], this.stages[k]);
			}
		}

		this.stepIndex += 1;
		if (this.stepIndex == this.steps) {
			this.report();
		}
		return true;
	}

	report(): void {
		let timed = this.stepMs.length;
		let total: f64 = 0;
		for (let i = 0; i < timed; ++i) {
			total += this.stepMs[i];
		}
		let sorted = this.stepMs.slice(0);
		sorted.sort();
		let counters = b3World_GetCounters(this.worldId);

		let line = "BENCH RESULT scene=" + sceneName(this.scene) + " steps=" + this.steps.toString() +
			" timed=" + timed.toString() + " bodies=" + counters.bodyCount.toString() +
			" contacts=" + counters.contactCount.toString() + " create=" + formatFixed(this.createMs, 2) +
			" mean=" + formatFixed(total / <f64>timed, 4) + " min=" + formatFixed(sorted[0], 4) +
			" median=" + formatFixed(median(sorted), 4) + " max=" + formatFixed(sorted[timed - 1], 4);
		for (let k = 0; k < STAGE_COUNT; ++k) {
			line += " " + STAGE_NAMES[k] + "=" + formatFixed(this.stageSum[k] / <f64>timed, 4);
		}
		// Per stage minimum over the timed steps, which a loaded machine disturbs least
		for (let k = 0; k < STAGE_COUNT; ++k) {
			line += " min_" + STAGE_NAMES[k] + "=" + formatFixed(this.stageMin[k], 4);
		}
		if (this.scene >= SCENE_CHAINS) {
			for (let k = 0; k < SCENE_QUERY_KIND_COUNT; ++k) {
				line += " " + QUERY_NAMES[k] + "=" + formatFixed(this.sceneData.queryMs[k] / <f64>this.steps, 4);
			}
			for (let k = 0; k < SCENE_QUERY_KIND_COUNT; ++k) {
				let hash = this.sceneData.queryHash[k];
				line += " " + QUERY_NAMES[k] + "Hash=" + hash.toString(16).padStart(8, "0");
			}
		}
		line += " hash=" + stateHash(this.hashBodies()).toString();
		line += " parts=" + Parallel.size().toString();
		let ps = b3_parallelStats;
		line += " parSolves=" + ps.solves.toString() + " parCollides=" + ps.collides.toString() + " parFinalizes=" + ps.finalizes.toString();
		line += " parContacts=" + formatFixed(<f64>ps.contacts / <f64>max(1, ps.collides), 1);
		line += " parDeferredContacts=" + formatFixed(<f64>ps.deferredContacts / <f64>max(1, ps.collides), 1);
		line += " parDeferredBodies=" + formatFixed(<f64>ps.deferredBodies / <f64>max(1, ps.finalizes), 1);
		line += " lanes=" + laneCount().toString() + " backend=" + laneBackend().toString() + " tasks=" + laneTasks().toString();
		let lanes = this.lanes;
		if (lanes != null) {
			let s = lanes.stats;
			let n = <f64>max(1, s.solves);
			line += " solves=" + s.solves.toString() + " fallbacks=" + s.fallbacks.toString();
			line += " lanePack=" + formatFixed(s.pack / n, 4) + " laneBlob=" + formatFixed(s.blob / n, 4);
			line += " packMasks=" + formatFixed(s.packMasks / n, 4) + " packNeeds=" + formatFixed(s.packNeeds / n, 4);
			line += " packContacts=" + formatFixed(s.packContacts / n, 4) + " packBodies=" + formatFixed(s.packBodies / n, 4);
			line += " lanePartition0=" + formatFixed(s.partition0 / n, 4) + " laneWait=" + formatFixed(s.wait / n, 4);
			line += " laneApply=" + formatFixed(s.apply / n, 4);
			line += " laneMBin=" + formatFixed(s.bytesIn / n / 1048576.0, 3) + " laneMBout=" + formatFixed(s.bytesOut / n / 1048576.0, 3);
			line += " laneShared=" + formatFixed(s.sharedBodies / max(1.0, s.bodies), 4);
			line += " laneExchanges=" + formatFixed(s.exchanges / n, 1);
			line += " mainSpins=" + formatFixed(s.mainSpins / n, 0) + " laneSpins=" + formatFixed(s.laneSpins / n, 0);
			for (let k = 0; k < T_COUNT; ++k) {
				line += " part_" + LANE_TIME_NAMES[k] + "=" + formatFixed(s.times[k] / n, 4);
			}
			let c = lanes.collide;
			let cn = <f64>max(1, c.steps);
			line += " collides=" + c.steps.toString() + " collideFallbacks=" + c.fallbacks.toString();
			line += " snapshots=" + c.snapshot.builds.toString();
			line += " collidePack=" + formatFixed(c.pack / cn, 4) + " collidePartition0=" + formatFixed(c.partition0 / cn, 4);
			line += " collideWait=" + formatFixed(c.wait / cn, 4) + " collideApply=" + formatFixed(c.apply / cn, 4);
			line += " collideMBin=" + formatFixed(c.bytesIn / cn / 1048576.0, 3) + " collideMBout=" + formatFixed(c.bytesOut / cn / 1048576.0, 3);
			let tree = lanes.tree;
			let tn = <f64>max(1, tree.tasks);
			line += " treeTasks=" + tree.tasks.toString() + " treePack=" + formatFixed(tree.pack / tn, 4) + " treeWait=" + formatFixed(tree.wait / tn, 4);
			line += " treeApply=" + formatFixed(tree.apply / tn, 4) + " treeMB=" + formatFixed(tree.bytes / tn / 1048576.0, 3);
		}
		console.log(line);
	}
}
