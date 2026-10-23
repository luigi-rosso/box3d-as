// Step timing for the port, the AnimaScript side of bench/native.
// --define=SCENE=0 pyramid (base 10), 1 large_pyramid (base 100), 2 many_bodies; --define=STEPS=n.
// One world step per frame; the scene is built in init and the result logged after the last step.

import { Layout, Context } from "rive/host";
import { clock } from "rive/os";
import { formatFixed } from "rive/format";
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
} from "./lib/box3d";

const SCENE_PYRAMID: i32 = 0;
const SCENE_LARGE_PYRAMID: i32 = 1;
const SCENE_MANY_BODIES: i32 = 2;
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
		return SCENE;
	}
	return SCENE_LARGE_PYRAMID;
}

function sceneName(scene: i32): string {
	return scene == SCENE_PYRAMID ? "pyramid" : scene == SCENE_LARGE_PYRAMID ? "large_pyramid" : "many_bodies";
}

function stepCount(scene: i32): i32 {
	if (isDefined(STEPS)) {
		return STEPS;
	}
	return scene == SCENE_MANY_BODIES ? 100 : 200;
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
	worldId: b3WorldId = b3WorldId();
	stepMs: Float64Array = new Float64Array(0);
	stageSum: Float64Array = new Float64Array(STAGE_COUNT);
	stages: Float64Array = new Float64Array(STAGE_COUNT);
	createMs: f64 = 0;

	override init(context: Context): bool {
		this.scene = sceneIndex();
		this.steps = stepCount(this.scene);
		this.stepMs = new Float64Array(this.steps - WARMUP_STEPS);

		let t0 = clock();
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
		this.createMs = (clock() - t0) * 1000.0;
		console.log("BENCH start scene=" + sceneName(this.scene) + " steps=" + this.steps.toString() +
			" create=" + formatFixed(this.createMs, 2) + "ms");
		return true;
	}

	override advance(seconds: f64): bool {
		if (this.stepIndex >= this.steps) {
			return true;
		}

		let t0 = clock();
		b3World_Step(this.worldId, 1.0 / 60.0, SUBSTEP_COUNT);
		let ms = (clock() - t0) * 1000.0;

		if (this.stepIndex >= WARMUP_STEPS) {
			this.stepMs[this.stepIndex - WARMUP_STEPS] = ms;
			profileToArray(b3World_GetProfile(this.worldId), this.stages);
			for (let k = 0; k < STAGE_COUNT; ++k) {
				this.stageSum[k] += this.stages[k];
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
		console.log(line);
	}
}
