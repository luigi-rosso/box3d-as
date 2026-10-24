// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript mirror of verify/scenes/scenes.c, statement for statement over lib/box3d.as. Every
// int to float conversion is an explicit <f32>, every float local is annotated.

import { FLT_MAX, b3GetMilliseconds, b3GetTicks } from "../../lib/core";
import {
	B3_DEG_TO_RAD,
	B3_PI,
	b3Add,
	b3BodyDef,
	b3BodyId,
	b3BodyType,
	b3Body_ApplyLinearImpulse,
	b3Body_GetAngularVelocity,
	b3Body_GetInverseMass,
	b3Body_GetLinearVelocity,
	b3Body_GetType,
	b3Body_GetWorldCenter,
	b3Body_GetWorldInverseRotationalInertia,
	b3Capsule,
	b3ClipVector,
	b3CollisionPlane,
	b3CreateBody,
	b3CreateCapsuleShape,
	b3CreateDistanceJoint,
	b3ComputeCosSin,
	b3DefaultDistanceJointDef,
	b3CreateFilterJoint,
	b3CreateHeightFieldShape,
	b3CreateHullShape,
	b3CreateMeshShape,
	b3CreateRevoluteJoint,
	b3CreateRock,
	b3CreateSphereShape,
	b3CreateSphericalJoint,
	b3CreateWave,
	b3CreateWaveMesh,
	b3CreateWorld,
	b3Cross,
	b3DefaultBodyDef,
	b3DefaultFilterJointDef,
	b3DefaultQueryFilter,
	b3DefaultRevoluteJointDef,
	b3DefaultShapeDef,
	b3DefaultSphericalJointDef,
	b3DefaultWorldDef,
	b3DestroyBody,
	b3DestroyHeightField,
	b3DestroyHull,
	b3DestroyMesh,
	b3DestroyWorld,
	b3Dot,
	b3GetLengthAndNormalize,
	b3HeightFieldData,
	b3JointId,
	b3JointType,
	b3Length,
	b3LengthSquared,
	b3MakeBoxHull,
	b3MakeQuatFromAxisAngle,
	b3MakeTransformedBoxHull,
	b3MaxFloat,
	b3MeshData,
	b3MulMV,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3Normalize,
	b3NormalizeQuat,
	b3OffsetPos,
	b3PlaneResult,
	b3Pos,
	b3Quat,
	b3Quat_identity,
	b3ShapeId,
	b3Shape_GetAABB,
	b3Shape_GetBody,
	b3SolvePlanes,
	b3Sphere,
	b3Sub,
	b3SubPos,
	b3Transform,
	b3TransformWorldPoint,
	b3Transform_identity,
	b3Vec2,
	b3Vec3,
	b3Vec3_axisX,
	b3Vec3_axisY,
	b3Vec3_axisZ,
	b3Vec3_one,
	b3WorldId,
	b3WorldTransform,
	b3World_CastMover,
	b3World_CastRayClosest,
	b3World_CollideMover,
	b3World_EnableSleeping,
	b3World_Step,
	b3_dynamicBody,
	b3_nullBodyId,
	b3_nullJointId,
	b3_revoluteJoint,
	b3_sphericalJoint,
	b3CompoundCapsuleDef,
	b3CompoundData,
	b3CompoundDef,
	b3CompoundHullDef,
	b3CompoundMeshDef,
	b3CompoundSphereDef,
	b3CreateBakedCompoundShape,
	b3CreateCompound,
	b3DefaultSurfaceMaterial,
	b3DestroyCompound,
	b3MakeCubeHull,
	b3RotateVector,
	b3ShapeProxy,
	b3SurfaceMaterial,
	b3ToPos,
	b3Vec3_zero,
	b3World_CastRay,
	b3World_CastShape,
	b3World_OverlapShape,
} from "../../lib/box3d";

export const SCENE_MOVER_PLANE_CAPACITY: i32 = 8;
export const SCENE_SUBSTEP_COUNT: i32 = 4;
export const SCENE_QUERY_MOVER_COUNT: i32 = 8;

// Query kinds timed and hashed by the village and queries scenes
export const SCENE_QUERY_RAY: i32 = 0;
export const SCENE_QUERY_RAY_ALL: i32 = 1;
export const SCENE_QUERY_SHAPE_CAST: i32 = 2;
export const SCENE_QUERY_OVERLAP: i32 = 3;
export const SCENE_QUERY_MOVER: i32 = 4;
export const SCENE_QUERY_KIND_COUNT: i32 = 5;

// Port of CharacterMover in samples/mover.h, minus the camera and drawing
export class SceneMover {
	transform: b3WorldTransform = b3Transform();
	velocity: b3Vec3 = b3Vec3();
	capsule: b3Capsule = b3Capsule();
	planes: StaticArray<b3CollisionPlane> = new StaticArray<b3CollisionPlane>(SCENE_MOVER_PLANE_CAPACITY);
	planePoints: StaticArray<b3Pos> = new StaticArray<b3Pos>(SCENE_MOVER_PLANE_CAPACITY);
	planeShapes: StaticArray<b3ShapeId> = new StaticArray<b3ShapeId>(SCENE_MOVER_PLANE_CAPACITY);
	planeCount: i32 = 0;
	totalIterations: i32 = 0;
	pogoVelocity: f32 = 0;
	onGround: bool = false;
	sprint: bool = false;

	// Recorded per step, not used by the mover itself
	rayHit: bool = false;
	rayFraction: f32 = 0;
	castFraction: f32 = 0;
	moveIterations: i32 = 0;
}

export class Scene {
	name: string = "";
	worldId: b3WorldId = b3WorldId();

	// Every body the scene created, in creation order. Destroyed bodies stay in the list.
	bodies: Array<b3BodyId> = new Array<b3BodyId>(0);
	bodyCount: i32 = 0;

	seed: u32 = 0;

	mesh: b3MeshData? = null;
	heightField: b3HeightFieldData? = null;
	bulletId: b3BodyId = b3BodyId();

	hasMover: bool = false;
	mover: SceneMover = new SceneMover();

	// Village and queries scenes
	compound: b3CompoundData? = null;
	compoundRotation: b3Quat = b3Quat();
	halfSpan: f32 = 0;
	moverCount: i32 = 0;
	movers: Array<SceneMover> = new Array<SceneMover>(0);

	// Milliseconds spent in each query kind, summed over steps, and a hash and hit count of the
	// results
	queryMs: Float64Array = new Float64Array(SCENE_QUERY_KIND_COUNT);
	queryHash: Uint32Array = new Uint32Array(SCENE_QUERY_KIND_COUNT);
	queryHits: Int32Array = new Int32Array(SCENE_QUERY_KIND_COUNT);
	queryNested: bool = false;
}

const s_sceneNames: StaticArray<string> = [
	"pyramid",
	"large_pyramid",
	"card_house",
	"chains",
	"capsule_mesh",
	"heightfield",
	"bullets",
	"mover",
	"many_bodies",
	"plank_pile",
	"mesh_drop",
	"village",
	"queries",
];

export function Scene_GetCount(): i32 {
	return s_sceneNames.length;
}

export function Scene_GetName(index: i32): string {
	return s_sceneNames[index];
}

// Numerical Recipes LCG
function Scene_Random(scene: Scene): u32 {
	scene.seed = scene.seed * 1664525 + 1013904223;
	return scene.seed;
}

// Uniform in [lo, hi] from the top 24 bits, exact up to the final multiply add
function Scene_RandomFloat(scene: Scene, lo: f32, hi: f32): f32 {
	let bits: u32 = Scene_Random(scene) >> 8;
	let r: f32 = <f32>bits * (<f32>1.0 / <f32>16777216.0);
	let range: f32 = hi - lo;
	let scaled: f32 = range * r;
	return lo + scaled;
}

function Scene_AddBody(scene: Scene, bodyDef: b3BodyDef): b3BodyId {
	let bodyId = b3CreateBody(scene.worldId, bodyDef);
	scene.bodies.push(bodyId);
	scene.bodyCount += 1;
	return bodyId;
}

// samples/sample.cpp Sample::AddGroundBox
function Scene_AddGroundBox(scene: Scene, extent: f32): b3BodyId {
	let bodyDef = b3DefaultBodyDef();
	bodyDef.position = b3Vec3(0.0, -1.0, 0.0);
	let groundId = Scene_AddBody(scene, bodyDef);

	let shapeDef = b3DefaultShapeDef();
	let hull = b3MakeBoxHull(extent, 1.0, extent);
	b3CreateHullShape(groundId, shapeDef, hull.base);

	return groundId;
}

// shared/benchmarks.c CreateLargePyramid with the base count as a parameter
function CreatePyramid(scene: Scene, baseCount: i32): void {
	b3World_EnableSleeping(scene.worldId, false);

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, -1.0, 0.0);
		let groundId = Scene_AddBody(scene, bodyDef);

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

			let bodyId = Scene_AddBody(scene, bodyDef);
			b3CreateHullShape(bodyId, shapeDef, box.base);
		}
	}
}

// samples/sample_stacking.cpp CardHouse
function CreateCardHouse(scene: Scene): void {
	Scene_AddGroundBox(scene, 10.0);

	let shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = 0.05;
	shapeDef.baseMaterial.friction = 0.7;

	let cardHeight: f32 = 0.2;
	let cardThickness: f32 = 0.001;
	let cardDepth: f32 = 0.1;

	let angle0: f32 = 25.0 * B3_PI / 180.0;
	let angle1: f32 = -25.0 * B3_PI / 180.0;
	let angle2: f32 = 0.5 * B3_PI;

	let cardBox = b3MakeBoxHull(cardThickness, cardHeight, cardDepth);
	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	let Nb = 5;
	let z0: f32 = 0.0;
	let y: f32 = cardHeight - 0.02;
	while (Nb > 0) {
		let z: f32 = z0;
		for (let i = 0; i < Nb; i++) {
			if (i != Nb - 1) {
				bodyDef.position = b3Vec3(z + 0.25, y + cardHeight - 0.015, 0.0);
				bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle2);
				let bodyId = Scene_AddBody(scene, bodyDef);
				b3CreateHullShape(bodyId, shapeDef, cardBox.base);
			}

			bodyDef.position = b3Vec3(z, y, 0.0);
			bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle1);
			let bodyId = Scene_AddBody(scene, bodyDef);
			b3CreateHullShape(bodyId, shapeDef, cardBox.base);

			z += 0.175;

			bodyDef.position = b3Vec3(z, y, 0.0);
			bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle0);
			bodyId = Scene_AddBody(scene, bodyDef);
			b3CreateHullShape(bodyId, shapeDef, cardBox.base);

			z += 0.175;
		}
		y += cardHeight * 2.0 - 0.03;
		z0 += 0.175;
		Nb -= 1;
	}
}

// shared/human.h
const bone_pelvis = 0;
const bone_spine_01 = 1;
const bone_spine_03 = 2;
const bone_neck = 3;
const bone_thigh_l = 4;
const bone_calf_l = 5;
const bone_thigh_r = 6;
const bone_calf_r = 7;
const bone_upper_arm_l = 8;
const bone_lower_arm_l = 9;
const bone_upper_arm_r = 10;
const bone_lower_arm_r = 11;
const bone_count = 12;

class SceneBone {
	bodyId: b3BodyId = b3BodyId();
	jointId: b3JointId = b3JointId();
	localFrameA: b3Transform = b3Transform();
	localFrameB: b3Transform = b3Transform();
	referenceFrame: b3Transform = b3Transform();
	jointType: b3JointType = 0;
	swingLimit: f32 = 0;
	twistLimit: b3Vec2 = b3Vec2();
	jointFriction: f32 = 0;
	parentIndex: i32 = 0;
}

// shared/human.c CreateHuman without colors, names and user data
function Scene_CreateHuman(
	scene: Scene,
	position: b3Pos,
	frictionTorque: f32,
	hertz: f32,
	dampingRatio: f32,
	groupIndex: i32,
): void {
	let bones = new Array<SceneBone>(0);
	for (let i = 0; i < bone_count; ++i) {
		bones.push(new SceneBone());
	}

	for (let i = 0; i < bone_count; ++i) {
		bones[i].bodyId = b3_nullBodyId;
		bones[i].jointId = b3_nullJointId;
		bones[i].jointFriction = 1.0;
		bones[i].parentIndex = -1;
	}

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	let shapeDef = b3DefaultShapeDef();
	let defaultDensity: f32 = shapeDef.density;

	{
		let bone = bones[bone_pelvis];
		bone.parentIndex = -1;

		bone.referenceFrame = b3Transform(b3Vec3(0.000000, 0.996219, -0.023868), b3Quat(b3Vec3(1.000000, 0.000000, 0.000000), 0.000000));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.6;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(0.040000, 0.000001, 0.000000), b3Vec3(-0.040000, -0.000001, 0.000000), 0.150000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);
	}

	{
		let bone = bones[bone_spine_01];
		bone.parentIndex = bone_pelvis;

		bone.referenceFrame = b3Transform(b3Vec3(0.000000, 1.017288, -0.024882), b3Quat(b3Vec3(1.000000, 0.000000, 0.000000), 0.000000));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = -groupIndex;

		shapeDef.baseMaterial.friction = 0.5;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(0.029876, -0.146581, 0.006260), b3Vec3(-0.029876, -0.146574, 0.006260), 0.145663);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

		bone.jointType = b3_sphericalJoint;
		bone.localFrameA = b3Transform(b3Vec3(0.000000, -0.021069, 0.001014), b3Quat(b3Vec3(-0.642737, 0.000000, 0.000000), -0.766087));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.707107, 0.000000, 0.000000), -0.707107));
		bone.swingLimit = 35.0 * B3_DEG_TO_RAD;
		bone.twistLimit = b3Vec2(-17.5 * B3_DEG_TO_RAD, 17.5 * B3_DEG_TO_RAD);
	}

	{
		let bone = bones[bone_spine_03];
		bone.parentIndex = bone_spine_01;

		bone.referenceFrame = b3Transform(b3Vec3(0.000000, 1.267766, -0.022320), b3Quat(b3Vec3(1.000000, 0.000000, 0.000000), 0.000000));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.5;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(0.063996, -0.117434, -0.040199), b3Vec3(-0.063996, -0.117432, -0.040199), 0.165004);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

		bone.jointType = b3_sphericalJoint;
		bone.localFrameA = b3Transform(b3Vec3(0.000000, -0.250478, -0.002562), b3Quat(b3Vec3(-0.642766, 0.000000, 0.000000), -0.766063));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.707107, 0.000000, 0.000000), -0.707107));
		bone.swingLimit = 35.0 * B3_DEG_TO_RAD;
		bone.twistLimit = b3Vec2(-17.5 * B3_DEG_TO_RAD, 17.5 * B3_DEG_TO_RAD);
	}

	{
		let bone = bones[bone_neck];
		bone.parentIndex = bone_spine_03;

		bone.referenceFrame = b3Transform(b3Vec3(0.000000, 1.498783, 0.024462), b3Quat(b3Vec3(0.987879, 0.000000, 0.000000), 0.155228));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.2;
		shapeDef.baseMaterial.rollingResistance = 0.05;
		let head = b3Capsule(b3Vec3(0.000000, -0.212340, 0.000000), b3Vec3(0.000000, -0.087340, 0.000000), 0.110000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, head);

		bone.jointType = b3_sphericalJoint;
		bone.localFrameA = b3Transform(b3Vec3(0.000000, -0.231017, -0.046781), b3Quat(b3Vec3(-0.516157, -0.000002, 0.000003), -0.856494));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.707108, -0.000002, 0.000002), -0.707106));
		bone.swingLimit = 30.0 * B3_DEG_TO_RAD;
		bone.twistLimit = b3Vec2(-17.5 * B3_DEG_TO_RAD, 17.5 * B3_DEG_TO_RAD);
	}

	{
		let bone = bones[bone_thigh_l];
		bone.parentIndex = bone_pelvis;

		bone.referenceFrame = b3Transform(b3Vec3(0.092175, 0.971562, -0.011177), b3Quat(b3Vec3(0.471412, 0.529510, -0.499888), -0.497495));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = -groupIndex;

		shapeDef.baseMaterial.friction = 0.6;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(-0.047269, 0.000001, 0.000000), b3Vec3(-0.379769, 0.000005, 0.000000), 0.091539);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

		bone.jointType = b3_sphericalJoint;
		bone.localFrameA = b3Transform(b3Vec3(0.092175, 0.024656, -0.012691), b3Quat(b3Vec3(-0.460186, -0.365368, -0.637662), 0.498118));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.499997, -0.500003, 0.500002), 0.499998));
		bone.swingLimit = 30.0 * B3_DEG_TO_RAD;
		bone.twistLimit = b3Vec2(-10.0 * B3_DEG_TO_RAD, 10.0 * B3_DEG_TO_RAD);
	}

	{
		let bone = bones[bone_calf_l];
		bone.parentIndex = bone_thigh_l;

		bone.referenceFrame = b3Transform(b3Vec3(0.118720, 0.534541, -0.035362), b3Quat(b3Vec3(0.521188, 0.480597, -0.546380), -0.445935));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.1;
		shapeDef.baseMaterial.rollingResistance = 0.0;
		let shin = b3Capsule(b3Vec3(-0.445000, 0.000001, 0.000000), b3Vec3(0.005000, -0.000001, 0.000000), 0.080000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, shin);

		shapeDef.density = 0.5 * defaultDensity;
		let foot = b3Capsule(b3Vec3(-0.456371, 0.129321, 0.000013), b3Vec3(-0.359977, 0.014394, 0.000013), 0.070000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, foot);
		shapeDef.density = defaultDensity;

		bone.jointType = b3_revoluteJoint;
		bone.localFrameA = b3Transform(b3Vec3(-0.438494, -0.000175, 0.000000), b3Quat(b3Vec3(0.344866, -0.938652, 0.000002), -0.000008));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.008182, -0.999967, 0.000001), -0.000007));
		bone.twistLimit = b3Vec2(-30.0 * B3_DEG_TO_RAD, 30.0 * B3_DEG_TO_RAD);
	}

	{
		let bone = bones[bone_thigh_r];
		bone.parentIndex = bone_pelvis;

		bone.referenceFrame = b3Transform(b3Vec3(-0.092175, 0.971562, -0.011177), b3Quat(b3Vec3(-0.497495, 0.499888, 0.529510), -0.471412));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = -groupIndex;

		shapeDef.baseMaterial.friction = 0.6;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(0.047269, -0.000001, 0.000000), b3Vec3(0.379769, -0.000005, 0.000000), 0.092587);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

		bone.jointType = b3_sphericalJoint;
		bone.localFrameA = b3Transform(b3Vec3(-0.092175, 0.024656, -0.012691), b3Quat(b3Vec3(0.380666, 0.491846, 0.488644), -0.611889));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.500002, 0.499998, -0.499998), 0.500002));
		bone.swingLimit = 30.0 * B3_DEG_TO_RAD;
		bone.twistLimit = b3Vec2(-10.0 * B3_DEG_TO_RAD, 10.0 * B3_DEG_TO_RAD);
	}

	{
		let bone = bones[bone_calf_r];
		bone.parentIndex = bone_thigh_r;

		bone.referenceFrame = b3Transform(b3Vec3(-0.118720, 0.534541, -0.035362), b3Quat(b3Vec3(-0.445935, 0.546380, 0.480597), -0.521188));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.1;
		shapeDef.baseMaterial.rollingResistance = 0.0;
		let shin = b3Capsule(b3Vec3(0.445000, -0.000002, 0.000003), b3Vec3(-0.005000, 0.000000, 0.000003), 0.080000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, shin);

		shapeDef.density = 0.5 * defaultDensity;
		let foot = b3Capsule(b3Vec3(0.369113, -0.006830, 0.000017), b3Vec3(0.465507, -0.121757, 0.000017), 0.070000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, foot);
		shapeDef.density = defaultDensity;

		bone.jointType = b3_revoluteJoint;
		bone.localFrameA = b3Transform(b3Vec3(0.438494, 0.000175, 0.000000), b3Quat(b3Vec3(0.000007, 0.000005, -0.344860), -0.938654));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(0.000006, 0.000005, 0.008188), -0.999966));
		bone.twistLimit = b3Vec2(-30.0 * B3_DEG_TO_RAD, 30.0 * B3_DEG_TO_RAD);
	}

	{
		let bone = bones[bone_upper_arm_l];
		bone.parentIndex = bone_spine_03;

		bone.referenceFrame = b3Transform(b3Vec3(0.185817, 1.443630, 0.031306), b3Quat(b3Vec3(0.560220, -0.307179, 0.366233), -0.676512));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.6;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(0.296038, 0.004386, -0.000794), b3Vec3(-0.016086, 0.004384, -0.016127), 0.065000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

		bone.jointType = b3_sphericalJoint;
		bone.localFrameA = b3Transform(b3Vec3(0.185817, -0.175865, -0.053625), b3Quat(b3Vec3(-0.559466, -0.213784, -0.670199), 0.438323));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.500953, -0.502617, -0.509318), -0.486844));
		bone.swingLimit = 45.0 * B3_DEG_TO_RAD;
		bone.twistLimit = b3Vec2(-20.0 * B3_DEG_TO_RAD, 20.0 * B3_DEG_TO_RAD);
		bone.jointFriction = 0.8;
	}

	{
		let bone = bones[bone_lower_arm_l];
		bone.parentIndex = bone_upper_arm_l;

		bone.referenceFrame = b3Transform(b3Vec3(0.347683, 1.193455, 0.029378), b3Quat(b3Vec3(0.406736, -0.493012, 0.089135), -0.763911));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.3;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(0.312329, -0.008256, -0.016454), b3Vec3(0.033153, 0.008259, -0.002741), 0.060000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

		bone.jointType = b3_revoluteJoint;
		bone.localFrameA = b3Transform(b3Vec3(0.297979, 0.000361, 0.000000), b3Quat(b3Vec3(0.916726, 0.399352, 0.000839), -0.011456));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(0.999922, -0.003857, 0.004659), -0.010908));
		bone.twistLimit = b3Vec2(-35.0 * B3_DEG_TO_RAD, 35.0 * B3_DEG_TO_RAD);
		bone.jointFriction = 0.06;
	}

	{
		let bone = bones[bone_upper_arm_r];
		bone.parentIndex = bone_spine_03;

		bone.referenceFrame = b3Transform(b3Vec3(-0.185817, 1.443630, 0.031306), b3Quat(b3Vec3(0.676512, 0.366233, 0.307179), 0.560220));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.6;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(-0.300237, -0.000001, -0.000319), b3Vec3(0.011887, 0.000001, 0.015013), 0.065000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

		bone.jointType = b3_sphericalJoint;
		bone.localFrameA = b3Transform(b3Vec3(-0.185817, -0.175865, -0.053625), b3Quat(b3Vec3(0.213793, 0.559449, 0.438334), -0.670204));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.509319, 0.486843, 0.500954), -0.502617));
		bone.swingLimit = 45.0 * B3_DEG_TO_RAD;
		bone.twistLimit = b3Vec2(-20.0 * B3_DEG_TO_RAD, 20.0 * B3_DEG_TO_RAD);
		bone.jointFriction = 0.8;
	}

	{
		let bone = bones[bone_lower_arm_r];
		bone.parentIndex = bone_upper_arm_r;

		bone.referenceFrame = b3Transform(b3Vec3(-0.347683, 1.193455, 0.029378), b3Quat(b3Vec3(0.763911, 0.089135, 0.493012), 0.406736));
		bodyDef.rotation = bone.referenceFrame.q;
		bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
		bone.bodyId = Scene_AddBody(scene, bodyDef);

		shapeDef.filter.groupIndex = 0;

		shapeDef.baseMaterial.friction = 0.3;
		shapeDef.baseMaterial.rollingResistance = 0.1;
		let capsule = b3Capsule(b3Vec3(-0.312649, -0.000003, 0.016471), b3Vec3(-0.032986, 0.000000, 0.002732), 0.060000);
		b3CreateCapsuleShape(bone.bodyId, shapeDef, capsule);

		bone.jointType = b3_revoluteJoint;
		bone.localFrameA = b3Transform(b3Vec3(-0.297979, -0.000361, 0.000000), b3Quat(b3Vec3(-0.405444, 0.914059, 0.010541), 0.000852));
		bone.localFrameB = b3Transform(b3Vec3(0.0, 0.0, 0.0), b3Quat(b3Vec3(-0.002794, 0.999936, 0.010042), 0.004364));
		bone.twistLimit = b3Vec2(-35.0 * B3_DEG_TO_RAD, 35.0 * B3_DEG_TO_RAD);
		bone.jointFriction = 0.06;
	}

	for (let i = 1; i < bone_count; ++i) {
		let bone = bones[i];
		let parent = bones[bone.parentIndex];

		let bodyIdA = parent.bodyId;
		let bodyIdB = bone.bodyId;

		bone.localFrameA.q = b3NormalizeQuat(bone.localFrameA.q);
		bone.localFrameB.q = b3NormalizeQuat(bone.localFrameB.q);

		if (bone.jointType == b3_revoluteJoint) {
			let jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = bodyIdA;
			jointDef.base.bodyIdB = bodyIdB;
			jointDef.base.localFrameA = bone.localFrameA;
			jointDef.base.localFrameB = bone.localFrameB;
			jointDef.enableLimit = true;
			jointDef.lowerAngle = bone.twistLimit.x;
			jointDef.upperAngle = bone.twistLimit.y;
			jointDef.enableSpring = hertz > 0.0;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = bone.jointFriction * frictionTorque;
			bone.jointId = b3CreateRevoluteJoint(scene.worldId, jointDef);
		} else if (bone.jointType == b3_sphericalJoint) {
			let jointDef = b3DefaultSphericalJointDef();
			jointDef.base.bodyIdA = bodyIdA;
			jointDef.base.bodyIdB = bodyIdB;
			jointDef.base.localFrameA = bone.localFrameA;
			jointDef.base.localFrameB = bone.localFrameB;
			jointDef.enableConeLimit = true;
			jointDef.coneAngle = bone.swingLimit;
			jointDef.enableTwistLimit = true;
			jointDef.lowerTwistAngle = bone.twistLimit.x;
			jointDef.upperTwistAngle = bone.twistLimit.y;
			jointDef.enableSpring = hertz > 0.0;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = bone.jointFriction * frictionTorque;
			bone.jointId = b3CreateSphericalJoint(scene.worldId, jointDef);
		}
	}

	// Disable some collisions
	let filterDef = b3DefaultFilterJointDef();
	filterDef.base.bodyIdA = bones[bone_thigh_l].bodyId;
	filterDef.base.bodyIdB = bones[bone_thigh_r].bodyId;
	b3CreateFilterJoint(scene.worldId, filterDef);
	filterDef.base.bodyIdA = bones[bone_neck].bodyId;
	filterDef.base.bodyIdB = bones[bone_upper_arm_l].bodyId;
	b3CreateFilterJoint(scene.worldId, filterDef);
	filterDef.base.bodyIdA = bones[bone_neck].bodyId;
	filterDef.base.bodyIdB = bones[bone_upper_arm_r].bodyId;
	b3CreateFilterJoint(scene.worldId, filterDef);
}

// samples/sample_joint.cpp BallAndChain, plus four ragdolls from shared/human.c on a box at z = -30
function CreateChains(scene: Scene): void {
	{
		let bodyDef = b3DefaultBodyDef();
		let groundBody = Scene_AddBody(scene, bodyDef);

		let linkRadius: f32 = 0.125;
		let linkExtent: f32 = 0.5;
		let capsule = b3Capsule(b3Vec3(-linkExtent, 0.0, 0.0), b3Vec3(linkExtent, 0.0, 0.0), linkRadius);
		let shapeDef = b3DefaultShapeDef();

		let linkCount = 32;

		bodyDef.type = b3_dynamicBody;
		let parent = groundBody;
		let jointDef = b3DefaultSphericalJointDef();
		jointDef.base.localFrameA = b3Transform_identity;
		jointDef.base.localFrameB = b3Transform(b3Vec3(-linkExtent, 0.0, 0.0), b3Quat_identity);
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 10.0;

		for (let i = 0; i < linkCount; ++i) {
			bodyDef.position = b3Vec3((1.0 + 2.0 * <f32>i) * linkExtent, 0.0, 0.0);
			let childId = Scene_AddBody(scene, bodyDef);

			b3CreateCapsuleShape(childId, shapeDef, capsule);

			jointDef.base.bodyIdA = parent;
			jointDef.base.bodyIdB = childId;
			b3CreateSphericalJoint(scene.worldId, jointDef);

			jointDef.base.localFrameA = b3Transform(b3Vec3(linkExtent, 0.0, 0.0), b3Quat_identity);
			parent = childId;
		}

		let sphereRadius: f32 = 2.0;
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), sphereRadius);

		bodyDef.position = b3Vec3((1.0 + 2.0 * <f32>linkCount) * linkExtent + sphereRadius - linkExtent, 0.0, 0.0);

		let childId = Scene_AddBody(scene, bodyDef);

		b3CreateSphereShape(childId, shapeDef, sphere);
		jointDef.base.bodyIdA = parent;
		jointDef.base.bodyIdB = childId;
		jointDef.base.localFrameB = b3Transform(b3Vec3(-sphereRadius, 0.0, 0.0), b3Quat_identity);
		b3CreateSphericalJoint(scene.worldId, jointDef);
	}

	{
		let boxZ: f32 = -30.0;

		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, -1.0, boxZ);
		let groundId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b3DefaultShapeDef();
		let hull = b3MakeBoxHull(10.0, 1.0, 10.0);
		b3CreateHullShape(groundId, shapeDef, hull.base);

		let humanCount = 4;
		for (let i = 0; i < humanCount; ++i) {
			let position = b3Vec3(0.2 * <f32>i, 2.0 * <f32>i, boxZ + 0.1 * <f32>i);
			let torque: f32 = 10.0;
			let hertz: f32 = 0.5;
			let damping: f32 = 0.7;
			let groupIndex = i + 1;
			Scene_CreateHuman(scene, position, torque, hertz, damping, groupIndex);
		}
	}
}

// samples/sample_continuous.cpp MeshDrop on a 16 x 16 grid with random velocities from the scene LCG
function CreateCapsuleMesh(scene: Scene): void {
	{
		let bodyDef = b3DefaultBodyDef();
		let groundId = Scene_AddBody(scene, bodyDef);

		let gridCount = 40;
		let cellWidth: f32 = 1.0;
		let amplitude: f32 = 0.5;
		let rowHz: f32 = 0.1;
		let columnHz: f32 = 0.2;

		scene.mesh = b3CreateWaveMesh(gridCount, gridCount, cellWidth, amplitude, rowHz, columnHz);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		b3CreateMeshShape(groundId, shapeDef, scene.mesh!, b3Vec3_one);

		let extent: f32 = 0.5 * <f32>gridCount * cellWidth;
		let halfHeight: f32 = 1.0;

		{
			let transform = b3Transform();
			transform.p = b3Vec3(0.0, halfHeight, -extent);
			transform.q = b3Quat_identity;
			let wallBox = b3MakeTransformedBoxHull(extent, halfHeight, 0.1, transform);
			b3CreateHullShape(groundId, shapeDef, wallBox.base);
		}

		{
			let transform = b3Transform();
			transform.p = b3Vec3(0.0, halfHeight, extent);
			transform.q = b3Quat_identity;
			let wallBox = b3MakeTransformedBoxHull(extent, halfHeight, 0.1, transform);
			b3CreateHullShape(groundId, shapeDef, wallBox.base);
		}

		{
			let transform = b3Transform();
			transform.p = b3Vec3(-extent, halfHeight, 0.0);
			transform.q = b3Quat_identity;
			let wallBox = b3MakeTransformedBoxHull(0.1, halfHeight, extent, transform);
			b3CreateHullShape(groundId, shapeDef, wallBox.base);
		}

		{
			let transform = b3Transform();
			transform.p = b3Vec3(extent, halfHeight, 0.0);
			transform.q = b3Quat_identity;
			let wallBox = b3MakeTransformedBoxHull(0.1, halfHeight, extent, transform);
			b3CreateHullShape(groundId, shapeDef, wallBox.base);
		}
	}

	let box = b3MakeBoxHull(0.02, 0.2, 0.04);
	let capsule = b3Capsule(b3Vec3(0.0, -0.2, 0.0), b3Vec3(0.0, 0.2, 0.0), 0.05);
	let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.05);

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	let capsuleShapeDef = b3DefaultShapeDef();
	capsuleShapeDef.baseMaterial.rollingResistance = 0.4;

	let shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = 0.1;

	scene.seed = 12345;

	let gridCount = 16;
	let halfGrid: f32 = 0.5 * <f32>gridCount;
	let bodyIndex = 0;

	for (let i = 0; i < gridCount; ++i) {
		for (let j = 0; j < gridCount; ++j) {
			let linearVelocity = b3Vec3();
			linearVelocity.x = Scene_RandomFloat(scene, -1.0, 1.0);
			linearVelocity.y = Scene_RandomFloat(scene, -1.0, 1.0);
			linearVelocity.z = Scene_RandomFloat(scene, -1.0, 1.0);

			let angularVelocity = b3Vec3();
			angularVelocity.x = Scene_RandomFloat(scene, -5.0, 5.0);
			angularVelocity.y = Scene_RandomFloat(scene, -5.0, 5.0);
			angularVelocity.z = Scene_RandomFloat(scene, -5.0, 5.0);

			bodyDef.position = b3Vec3(0.5 * (<f32>i - halfGrid), 5.0, 0.5 * (<f32>j - halfGrid));
			bodyDef.linearVelocity = linearVelocity;
			bodyDef.angularVelocity = angularVelocity;
			let bodyId = Scene_AddBody(scene, bodyDef);

			let kind = bodyIndex % 8;
			if (kind == 6) {
				b3CreateSphereShape(bodyId, shapeDef, sphere);
			} else if (kind == 7) {
				b3CreateHullShape(bodyId, shapeDef, box.base);
			} else {
				b3CreateCapsuleShape(bodyId, capsuleShapeDef, capsule);
			}

			bodyIndex += 1;
		}
	}
}

// shared/determinism.c CreateWavePile on a wave height field, jitter and rotations from the scene LCG
function CreateHeightField(scene: Scene): void {
	scene.seed = 52977;

	let fieldCount = 21;
	let fieldScale = b3Vec3(1.0, 0.6, 1.0);
	scene.heightField = b3CreateWave(fieldCount, fieldCount, fieldScale, 0.08, 0.06, false);

	{
		let extent: f32 = fieldScale.x * <f32>(fieldCount - 1);
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position.x = -0.5 * extent;
		bodyDef.position.z = -0.5 * extent;
		let groundId = Scene_AddBody(scene, bodyDef);

		let shapeDef = b3DefaultShapeDef();
		b3CreateHeightFieldShape(groundId, shapeDef, scene.heightField!);
	}

	let rock = b3CreateRock(0.55)!;
	let box = b3MakeBoxHull(0.45, 0.3, 0.55);
	let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.5);
	let capsule = b3Capsule(b3Vec3(0.0, -0.3, 0.0), b3Vec3(0.0, 0.3, 0.0), 0.35);

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	let shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = 0.3;

	let gridCount = 5;
	let layerCount = 4;
	let spacing: f32 = 1.7;
	let halfSpan: f32 = 0.5 * <f32>(gridCount - 1);
	let index = 0;
	for (let layer = 0; layer < layerCount; ++layer) {
		for (let i = 0; i < gridCount; ++i) {
			for (let j = 0; j < gridCount; ++j) {
				let jitter = b3Vec3();
				jitter.x = Scene_RandomFloat(scene, -0.3, 0.3);
				jitter.y = Scene_RandomFloat(scene, -0.3, 0.3);
				jitter.z = Scene_RandomFloat(scene, -0.3, 0.3);

				let axis = b3Vec3();
				axis.x = Scene_RandomFloat(scene, -1.0, 1.0);
				axis.y = Scene_RandomFloat(scene, -1.0, 1.0);
				axis.z = Scene_RandomFloat(scene, -1.0, 1.0);
				let angle: f32 = Scene_RandomFloat(scene, -B3_PI, B3_PI);

				bodyDef.position.x = spacing * (<f32>i - halfSpan) + jitter.x;
				bodyDef.position.y = 2.5 + 1.6 * <f32>layer + 0.3 * jitter.y;
				bodyDef.position.z = spacing * (<f32>j - halfSpan) + jitter.z;
				bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Normalize(axis), angle);

				let bodyId = Scene_AddBody(scene, bodyDef);

				let kind = index % 4;
				if (kind == 0) {
					b3CreateSphereShape(bodyId, shapeDef, sphere);
				} else if (kind == 1) {
					b3CreateCapsuleShape(bodyId, shapeDef, capsule);
				} else if (kind == 2) {
					b3CreateHullShape(bodyId, shapeDef, box.base);
				} else {
					b3CreateHullShape(bodyId, shapeDef, rock);
				}

				index += 1;
			}
		}
	}

	// The world keeps its own copy
	b3DestroyHull(rock);
}

// samples/sample_continuous.cpp BulletVersusStack, plus ThinWall moved to z = -40 on the same ground
function CreateBullets(scene: Scene): void {
	Scene_AddGroundBox(scene, 50.0);

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, -1.0, 0.0);
		let groundBodyId = Scene_AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		let transform = b3Transform(b3Vec3(-1.0, 5.0, 0.0), b3Quat_identity);
		let wallBox = b3MakeTransformedBoxHull(0.1, 5.0, 10.0, transform);
		b3CreateHullShape(groundBodyId, shapeDef, wallBox.base);
	}

	{
		let box = b3MakeBoxHull(0.5, 0.5, 0.5);
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		let shapeDef = b3DefaultShapeDef();
		for (let row = 0; row < 10; ++row) {
			bodyDef.position = b3Vec3(0.0, 0.5 + 1.1 * <f32>row, 0.0);
			let bodyId = Scene_AddBody(scene, bodyDef);
			b3CreateHullShape(bodyId, shapeDef, box.base);
		}
	}

	scene.bulletId = b3_nullBodyId;

	{
		let wallZ: f32 = -40.0;

		let bodyDef = b3DefaultBodyDef();
		let shapeDef = b3DefaultShapeDef();

		bodyDef.position = b3Vec3(0.0, 10.0, wallZ);
		bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 90.0 * B3_DEG_TO_RAD);
		let wallId = Scene_AddBody(scene, bodyDef);
		let wallBox = b3MakeBoxHull(10.0, 0.1, 10.0);
		b3CreateHullShape(wallId, shapeDef, wallBox.base);

		bodyDef.type = b3_dynamicBody;
		bodyDef.rotation = b3Quat_identity;
		shapeDef.baseMaterial.rollingResistance = 0.1;

		bodyDef.position = b3Vec3(-5.0, 10.0, wallZ + 20.0);
		bodyDef.linearVelocity = b3Vec3(0.0, 0.0, -180.0);
		bodyDef.angularVelocity = b3Vec3(20.0, 0.0, 0.0);
		let sphereBodyId = Scene_AddBody(scene, bodyDef);

		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.1);
		b3CreateSphereShape(sphereBodyId, shapeDef, sphere);

		bodyDef.position = b3Vec3(0.0, 10.0, wallZ + 20.0);
		bodyDef.linearVelocity = b3Vec3(0.0, 0.0, -180.0);
		bodyDef.angularVelocity = b3Vec3(20.0, -5.0, 0.0);
		let capsuleBodyId = Scene_AddBody(scene, bodyDef);

		let capsule = b3Capsule(b3Vec3(-0.3, 0.0, 0.0), b3Vec3(0.3, 0.0, 0.0), 0.1);
		b3CreateCapsuleShape(capsuleBodyId, shapeDef, capsule);

		bodyDef.position = b3Vec3(5.0, 10.0, wallZ + 20.0);
		bodyDef.linearVelocity = b3Vec3(0.0, 0.0, -180.0);
		bodyDef.angularVelocity = b3Vec3(20.0, 5.0, 0.0);
		let boxBodyId = Scene_AddBody(scene, bodyDef);
		let box = b3MakeBoxHull(0.4, 0.1, 0.1);
		b3CreateHullShape(boxBodyId, shapeDef, box.base);
	}
}

// samples/sample_continuous.cpp BulletVersusStack::Launch
function LaunchBullet(scene: Scene): void {
	if (scene.bulletId.index1 != 0) {
		b3DestroyBody(scene.bulletId);
	}

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.isBullet = true;
	bodyDef.position = b3Vec3(20.5, 5.5, 0.0);
	bodyDef.linearVelocity = b3Vec3(-500.0, 0.0, 0.0);
	scene.bulletId = Scene_AddBody(scene, bodyDef);

	let shapeDef = b3DefaultShapeDef();
	shapeDef.density *= 10.0;
	let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.25);
	b3CreateSphereShape(scene.bulletId, shapeDef, sphere);
}

// samples/mover.cpp CharacterMover constants
const s_moverJumpSpeed: f32 = 5.0;
const s_moverMaxSpeed: f32 = 6.0;
const s_moverMinSpeed: f32 = 0.01;
const s_moverStopSpeed: f32 = 1.0;
const s_moverAccelerate: f32 = 30.0;
const s_moverFriction: f32 = 4.0;
const s_moverGravity: f32 = 15.0;

// samples/mover.cpp CharacterMover::Initialize
function Scene_InitializeMover(mover: SceneMover, position: b3Pos): void {
	mover.transform.p = position;
	mover.transform.q = b3Quat_identity;
	mover.velocity = b3Vec3(0.0, 0.0, 0.0);
	mover.capsule = b3Capsule(b3Vec3(0.0, -0.5, 0.0), b3Vec3(0.0, 0.5, 0.0), 0.3);
	mover.planeCount = 0;
	mover.totalIterations = 0;
	mover.pogoVelocity = 0.0;
	mover.onGround = false;
	mover.sprint = false;
}

// A small level from boxes in place of GeometricMover's obj mesh: floor, step, ramp, wall, and one
// dynamic box in the path for the push code
function CreateMover(scene: Scene): void {
	Scene_AddGroundBox(scene, 20.0);

	let bodyDef = b3DefaultBodyDef();
	let levelId = Scene_AddBody(scene, bodyDef);
	let shapeDef = b3DefaultShapeDef();

	{
		let transform = b3Transform(b3Vec3(4.0, 0.15, 0.0), b3Quat_identity);
		let box = b3MakeTransformedBoxHull(1.0, 0.15, 3.0, transform);
		b3CreateHullShape(levelId, shapeDef, box.base);
	}

	{
		let transform = b3Transform();
		transform.p = b3Vec3(9.0, 0.5, 0.0);
		transform.q = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 20.0 * B3_DEG_TO_RAD);
		let box = b3MakeTransformedBoxHull(3.0, 0.1, 3.0, transform);
		b3CreateHullShape(levelId, shapeDef, box.base);
	}

	{
		let transform = b3Transform(b3Vec3(14.0, 1.5, 0.0), b3Quat_identity);
		let box = b3MakeTransformedBoxHull(0.2, 1.5, 6.0, transform);
		b3CreateHullShape(levelId, shapeDef, box.base);
	}

	{
		let boxDef = b3DefaultBodyDef();
		boxDef.type = b3_dynamicBody;
		boxDef.position = b3Vec3(1.5, 0.4, 0.5);
		let boxId = Scene_AddBody(scene, boxDef);
		let box = b3MakeBoxHull(0.4, 0.4, 0.4);
		b3CreateHullShape(boxId, shapeDef, box.base);
	}

	scene.hasMover = true;
	scene.mover = new SceneMover();
	Scene_InitializeMover(scene.mover, b3Vec3(0.0, 1.4, 0.0));
}

function Scene_MoverFilter(shapeId: b3ShapeId): bool {
	return true;
}

// samples/mover.cpp PlaneResultFcn; no shape carries MoverShapeUserData so every plane is rigid and
// clips. The query callbacks carry no context, so the mover being solved sits in a module global.
let s_planeMover: SceneMover = new SceneMover();

function Scene_MoverPlaneResult(shapeId: b3ShapeId, planeResults: StaticArray<b3PlaneResult>, planeCount: i32): bool {
	let mover = s_planeMover;
	let maxPush: f32 = FLT_MAX;
	let clipVelocity = true;

	for (let i = 0; i < planeCount && mover.planeCount < SCENE_MOVER_PLANE_CAPACITY; ++i) {
		let index = mover.planeCount;
		mover.planes[index].plane = planeResults[i].plane;
		mover.planes[index].pushLimit = maxPush;
		mover.planes[index].push = 0.0;
		mover.planes[index].clipVelocity = clipVelocity;
		mover.planePoints[index] = b3OffsetPos(mover.transform.p, planeResults[i].point);
		mover.planeShapes[index] = shapeId;
		mover.planeCount += 1;
	}

	return true;
}

// samples/mover.cpp CharacterMover::SolveMove
function Scene_SolveMove(
	scene: Scene,
	mover: SceneMover,
	timeStep: f32,
	forward: b3Vec3,
	right: b3Vec3,
	throttle: b3Vec2,
	clipVelocity: bool,
): void {
	// Friction
	let speed: f32 = b3Length(mover.velocity);
	if (speed < s_moverMinSpeed) {
		mover.velocity.x = 0.0;
		mover.velocity.z = 0.0;
	} else {
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		let control: f32 = speed < s_moverStopSpeed ? s_moverStopSpeed : speed;

		// friction has units of 1/time
		let drop: f32 = control * s_moverFriction * timeStep;
		let newSpeed: f32 = b3MaxFloat(0.0, speed - drop);
		let ratio: f32 = newSpeed / speed;
		mover.velocity.x *= ratio;
		mover.velocity.z *= ratio;
	}

	let maxSpeed: f32 = mover.sprint ? 1.5 * s_moverMaxSpeed : s_moverMaxSpeed;

	let desiredForward = b3MulSV(maxSpeed * throttle.x, forward);
	let desiredRight = b3MulSV(maxSpeed * throttle.y, right);
	let desiredVelocity = b3Add(desiredForward, desiredRight);
	let desiredSpeedOut: FixedArray<f32, 1>;
	desiredSpeedOut[0] = 0.0;
	let desiredDirection = b3GetLengthAndNormalize(desiredSpeedOut[0], desiredVelocity);
	let desiredSpeed: f32 = desiredSpeedOut[0];

	if (desiredSpeed > maxSpeed) {
		desiredVelocity = b3MulSV(maxSpeed / desiredSpeed, desiredVelocity);
		desiredSpeed = maxSpeed;
	}

	if (mover.onGround) {
		mover.velocity.y = 0.0;
	}

	// Accelerate
	let currentSpeed: f32 = b3Dot(mover.velocity, desiredDirection);
	let addSpeed: f32 = desiredSpeed - currentSpeed;
	if (addSpeed > 0.0) {
		let accelSpeed: f32 = s_moverAccelerate * maxSpeed * timeStep;
		if (accelSpeed > addSpeed) {
			accelSpeed = addSpeed;
		}

		mover.velocity = b3Add(mover.velocity, b3MulSV(accelSpeed, desiredDirection));
	}

	mover.velocity.y -= s_moverGravity * timeStep;

	let worldId = scene.worldId;

	let pogoRestLength: f32 = 3.0 * mover.capsule.radius;
	let rayLength: f32 = pogoRestLength + mover.capsule.radius;
	let rayOrigin = b3TransformWorldPoint(mover.transform, mover.capsule.center1);
	let rayTranslation = b3MulSV(-rayLength, b3Vec3_axisY);

	// The sample writes { 1, ~2u }, which zero extends the mask
	let skipTeamFilter = b3DefaultQueryFilter();
	skipTeamFilter.categoryBits = 1;
	skipTeamFilter.maskBits = 0xfffffffd;
	let rayResult = b3World_CastRayClosest(worldId, rayOrigin, rayTranslation, skipTeamFilter);
	mover.rayHit = rayResult.hit;
	mover.rayFraction = rayResult.fraction;

	// After gravity was applied, disable pogo when still moving up.
	// Avoids getting pulled back to the ground when jumping.
	let suppressPogo = mover.velocity.y > 0.0;

	if (rayResult.hit == false || suppressPogo) {
		mover.onGround = false;
		mover.pogoVelocity = 0.0;
	} else {
		mover.onGround = true;
		let pogoCurrentLength: f32 = rayResult.fraction * rayLength;

		let zeta: f32 = 0.7;
		let hertz: f32 = 4.0;
		let omega: f32 = 2.0 * B3_PI * hertz;
		let omegaH: f32 = omega * timeStep;

		mover.pogoVelocity =
			(mover.pogoVelocity - omega * omegaH * (pogoCurrentLength - pogoRestLength)) /
			(1.0 + 2.0 * zeta * omegaH + omegaH * omegaH);
	}

	let startPosition = mover.transform.p;
	let velocityDelta = b3MulSV(timeStep, mover.velocity);
	let pogoDelta = b3MulSV(timeStep * mover.pogoVelocity, b3Vec3_axisY);
	let target = b3Add(b3Add(mover.transform.p, velocityDelta), pogoDelta);

	// Want the mover to collide with allies
	let moverFilter = b3DefaultQueryFilter();
	moverFilter.categoryBits = 1;
	moverFilter.maskBits = 0xffffffff;
	moverFilter.id = 1;

	// The cast should ignore allies
	let castFilter = b3DefaultQueryFilter();
	castFilter.categoryBits = 1;
	castFilter.maskBits = 0xfffffffd;
	castFilter.id = 1;

	mover.totalIterations = 0;
	mover.moveIterations = 0;
	mover.castFraction = 1.0;
	let tolerance: f32 = 0.01;

	for (let iteration = 0; iteration < 5; ++iteration) {
		mover.planeCount = 0;
		mover.moveIterations = iteration + 1;

		let capsule = b3Capsule();
		capsule.center1 = mover.capsule.center1;
		capsule.center2 = mover.capsule.center2;
		capsule.radius = mover.capsule.radius;

		s_planeMover = mover;
		b3World_CollideMover(worldId, mover.transform.p, capsule, moverFilter, Scene_MoverPlaneResult);

		let targetDelta = b3Sub(target, mover.transform.p);
		let result = b3SolvePlanes(targetDelta, mover.planes, mover.planeCount);

		mover.totalIterations += result.iterationCount;

		let delta = result.delta;

		let fraction: f32 = b3World_CastMover(worldId, mover.transform.p, capsule, delta, castFilter, Scene_MoverFilter);
		mover.castFraction = fraction;

		delta = b3MulSV(fraction, delta);
		mover.transform.p = b3Add(mover.transform.p, delta);

		if (b3LengthSquared(delta) < tolerance * tolerance) {
			break;
		}
	}

	for (let i = 0; i < mover.planeCount; ++i) {
		let bodyId = b3Shape_GetBody(mover.planeShapes[i]);
		let bodyType: b3BodyType = b3Body_GetType(bodyId);
		if (bodyType != b3_dynamicBody) {
			continue;
		}

		let point = mover.planePoints[i];
		let normal = b3Neg(mover.planes[i].plane.normal);

		let invMassA: f32 = 0.0;
		let invMassB: f32 = b3Body_GetInverseMass(bodyId);
		let invIB = b3Body_GetWorldInverseRotationalInertia(bodyId);

		let pB = b3Body_GetWorldCenter(bodyId);
		let rB = b3SubPos(point, pB);

		let rnB = b3Cross(rB, normal);
		let kNormal: f32 = invMassA + invMassB + b3Dot(rnB, b3MulMV(invIB, rnB));
		let normalMass: f32 = kNormal > 0.0 ? <f32>1.0 / kNormal : <f32>0.0;

		let vB = b3Body_GetLinearVelocity(bodyId);
		let omegaB = b3Body_GetAngularVelocity(bodyId);
		let vrB = b3Add(vB, b3Cross(omegaB, rB));
		let vn: f32 = b3Dot(b3Sub(vrB, mover.velocity), normal);
		let impulse: f32 = b3MaxFloat(-normalMass * vn, 0.0);

		let P = b3MulSV(impulse, normal);
		mover.velocity = b3MulSub(mover.velocity, invMassA, P);

		b3Body_ApplyLinearImpulse(bodyId, P, point, true);
	}

	if (clipVelocity) {
		// Using the velocity clipper can avoid picking up velocity from depenetration.
		// This allows the mover to avoid velocity from soft collision depenetration.
		mover.velocity = b3ClipVector(mover.velocity, mover.planes, mover.planeCount);
	} else if (timeStep > 0.0) {
		// Using the position delta is more holistic and intuitive in some cases.
		mover.velocity = b3MulSV(<f32>1.0 / timeStep, b3Sub(mover.transform.p, startPosition));
	}
}

// samples/mover.cpp CharacterMover::Step with scripted keys in place of the keyboard and camera
function Scene_StepMover(scene: Scene, stepIndex: i32, timeStep: f32): void {
	let mover = scene.mover;

	let throttle = b3Vec2(0.0, 0.0);
	let forward = b3Vec3(1.0, 0.0, 0.0);
	let right = b3Vec3(0.0, 0.0, 1.0);
	let jumpKey = false;
	let sprintKey = false;

	if (stepIndex < 140) {
		throttle.x = 1.0;
	} else if (stepIndex < 200) {
		throttle.x = 1.0;
		throttle.y = 1.0;
		jumpKey = stepIndex == 170;
	} else if (stepIndex < 260) {
		throttle.x = -1.0;
		sprintKey = true;
	}

	if (jumpKey && mover.onGround == true) {
		mover.velocity.y = s_moverJumpSpeed;
		mover.onGround = false;
	}

	if (mover.onGround == true) {
		mover.sprint = sprintKey;
	} else {
		mover.sprint = false;
	}

	let clipVelocity = true;
	Scene_SolveMove(scene, mover, timeStep, forward, right, throttle, clipVelocity);
}

// shared/benchmarks.c CreateSmallPyramid
function CreateSmallPyramid(scene: Scene, baseCount: i32, extent: f32, centerX: f32, baseZ: f32): void {
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

			let bodyId = Scene_AddBody(scene, bodyDef);
			b3CreateHullShape(bodyId, shapeDef, box.base);
		}
	}
}

// shared/benchmarks.c CreateManyPyramids, release sizes
function CreateManyPyramids(scene: Scene): void {
	let baseCount = 10;
	let extent: f32 = 0.5;
	let rowCount = 14;
	let columnCount = 14;
	let groundExtent: f32 = extent * <f32>columnCount * (<f32>baseCount + 1.0);

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, -1.0, 0.0);
		let groundId = Scene_AddBody(scene, bodyDef);

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
			CreateSmallPyramid(scene, baseCount, extent, centerX, baseZ);
		}

		baseZ += deltaZ;
	}
}

// shared/benchmarks.c CreateMeshDropBenchmark at release size: 100 x 100 boxes that only collide
// with a 120 x 120 wave mesh. RandomVec3Uniform is replaced by the scene LCG.
function CreateMeshDrop(scene: Scene): void {
	let meshGridCount = 120;
	let borderCount = 10;
	let cellWidth: f32 = 1.0;

	{
		let bodyDef = b3DefaultBodyDef();
		let groundId = Scene_AddBody(scene, bodyDef);

		let rowHz: f32 = 0.1;
		let columnHz: f32 = 0.2;
		let groundAmplitude: f32 = 0.5;

		scene.mesh = b3CreateWaveMesh(meshGridCount, meshGridCount, cellWidth, groundAmplitude, rowHz, columnHz);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		b3CreateMeshShape(groundId, shapeDef, scene.mesh!, b3Vec3_one);
	}

	let box = b3MakeBoxHull(0.02, 0.2, 0.04);

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	let shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = 0.1;

	// Don't allow shapes to collide with each other.
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = 1;

	scene.seed = 3963634789;

	let gridCount = 100;
	let spacing: f32 = (cellWidth * <f32>(meshGridCount - 2 * borderCount)) / <f32>gridCount;
	let offset: f32 = 0.5 * spacing * <f32>(1 - gridCount);

	for (let i = 0; i < gridCount; ++i) {
		for (let j = 0; j < gridCount; ++j) {
			let linearVelocity = b3Vec3();
			linearVelocity.x = Scene_RandomFloat(scene, -1.0, 1.0);
			linearVelocity.y = Scene_RandomFloat(scene, -1.0, 1.0);
			linearVelocity.z = Scene_RandomFloat(scene, -1.0, 1.0);

			let angularVelocity = b3Vec3();
			angularVelocity.x = Scene_RandomFloat(scene, -5.0, 5.0);
			angularVelocity.y = Scene_RandomFloat(scene, -5.0, 5.0);
			angularVelocity.z = Scene_RandomFloat(scene, -5.0, 5.0);

			bodyDef.position.x = offset + spacing * <f32>i;
			bodyDef.position.y = 5.0;
			bodyDef.position.z = offset + spacing * <f32>j;
			bodyDef.linearVelocity = linearVelocity;
			bodyDef.angularVelocity = angularVelocity;
			let bodyId = Scene_AddBody(scene, bodyDef);

			b3CreateHullShape(bodyId, shapeDef, box.base);
		}
	}
}

function Scene_HashQuery(scene: Scene, kind: i32, value: u32): void {
	let h: u32 = scene.queryHash[kind] ^ value;
	scene.queryHash[kind] = h * 16777619;
}

function Scene_HashVec3(scene: Scene, kind: i32, v: b3Vec3): void {
	Scene_HashQuery(scene, kind, reinterpret<u32>(v.x));
	Scene_HashQuery(scene, kind, reinterpret<u32>(v.y));
	Scene_HashQuery(scene, kind, reinterpret<u32>(v.z));
}

// The static compound of shared/benchmarks.c CreateVillage: a grid of box tiles with a sparse set
// of wave mesh patches, and a capsule or sphere prop on every eighth tile
function Scene_CreateVillageGround(scene: Scene, gridCount: i32, angle: f32): void {
	let tileWidth: f32 = 4.0;
	let halfSpan: f32 = 0.5 * tileWidth * <f32>gridCount;
	scene.halfSpan = halfSpan;

	let hulls = new Array<b3CompoundHullDef>();
	let meshes = new Array<b3CompoundMeshDef>();
	let capsules = new Array<b3CompoundCapsuleDef>();
	let spheres = new Array<b3CompoundSphereDef>();

	let material = b3DefaultSurfaceMaterial();
	let tile = b3MakeBoxHull(0.5 * tileWidth, 0.5, 0.5 * tileWidth);

	// A wave patch replaces a sparse set of tiles so the compound carries mesh children
	let wave = b3CreateWaveMesh(8, 8, tileWidth / 8.0, 0.25, 0.25, 0.25)!;
	let materials = new StaticArray<b3SurfaceMaterial>(1);
	materials[0] = material;

	for (let i = 0; i < gridCount; ++i) {
		let x: f32 = -halfSpan + (<f32>i + 0.5) * tileWidth;

		for (let j = 0; j < gridCount; ++j) {
			let z: f32 = -halfSpan + (<f32>j + 0.5) * tileWidth;

			if (i % 16 == 8 && j % 16 == 8) {
				let meshDef = new b3CompoundMeshDef(wave);
				meshDef.transform.p = b3Vec3(x, 0.0, z);
				meshDef.transform.q = b3Quat_identity;
				meshDef.scale = b3Vec3_one;
				meshDef.materials = materials;
				meshDef.materialCount = 1;
				meshes.push(meshDef);
				continue;
			}

			let transform = b3Transform();
			transform.p = b3Vec3(x, -0.5, z);
			transform.q = b3Quat_identity;
			hulls.push(new b3CompoundHullDef(tile.base, transform, material));

			let tileIndex = i * gridCount + j;
			if (tileIndex % 8 != 0) {
				continue;
			}

			if (((tileIndex / 8) & 1) != 0) {
				let radius: f32 = 0.25;
				let capsuleDef = b3CompoundCapsuleDef();
				capsuleDef.capsule.center1 = b3Vec3(x - 1.0, radius, z);
				capsuleDef.capsule.center2 = b3Vec3(x + 1.0, radius, z);
				capsuleDef.capsule.radius = radius;
				capsuleDef.material = material;
				capsules.push(capsuleDef);
			} else {
				let radius: f32 = 0.5;
				let sphereDef = b3CompoundSphereDef();
				sphereDef.sphere.center = b3Vec3(x, radius, z);
				sphereDef.sphere.radius = radius;
				sphereDef.material = material;
				spheres.push(sphereDef);
			}
		}
	}

	let def = new b3CompoundDef();
	// StaticArray.fromArray traps on arrays of structs (notes/findings.md)
	let capsuleDefs = new StaticArray<b3CompoundCapsuleDef>(capsules.length);
	for (let i = 0; i < capsules.length; ++i) {
		capsuleDefs[i] = capsules[i];
	}
	let sphereDefs = new StaticArray<b3CompoundSphereDef>(spheres.length);
	for (let i = 0; i < spheres.length; ++i) {
		sphereDefs[i] = spheres[i];
	}

	def.capsules = capsuleDefs;
	def.capsuleCount = capsules.length;
	def.hulls = StaticArray.fromArray<b3CompoundHullDef>(hulls);
	def.hullCount = hulls.length;
	def.meshes = StaticArray.fromArray<b3CompoundMeshDef>(meshes);
	def.meshCount = meshes.length;
	def.spheres = sphereDefs;
	def.sphereCount = spheres.length;

	scene.compound = b3CreateCompound(def);

	// The compound clones its inputs
	b3DestroyMesh(wave);

	// Rotated so broad-phase lookups transform into the compound frame
	scene.compoundRotation = b3MakeQuatFromAxisAngle(b3Vec3_axisY, angle);

	let bodyDef = b3DefaultBodyDef();
	bodyDef.rotation = scene.compoundRotation;
	let groundId = Scene_AddBody(scene, bodyDef);

	let shapeDef = b3DefaultShapeDef();
	b3CreateBakedCompoundShape(groundId, shapeDef, scene.compound!);
}

// shared/benchmarks.c CreateVillage at release size: 70 x 70 mixed bodies, every twentieth fast
// enough to need continuous, onto a rotated 100 x 100 tile compound. RandomFloatRange and
// RandomVec3Uniform are replaced by the scene LCG.
function CreateVillage(scene: Scene): void {
	scene.seed = 2718281828;

	Scene_CreateVillageGround(scene, 100, <f32>-1.15 * B3_PI);

	let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.3);
	let capsule = b3Capsule(b3Vec3(-0.3, 0.0, 0.0), b3Vec3(0.3, 0.0, 0.0), 0.2);
	let box = b3MakeCubeHull(0.3);

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	let shapeDef = b3DefaultShapeDef();

	let bodyGrid = 70;
	let spacing: f32 = 1.6;
	let offset: f32 = 0.5 * spacing * <f32>(1 - bodyGrid);

	for (let i = 0; i < bodyGrid; ++i) {
		for (let j = 0; j < bodyGrid; ++j) {
			let index = i * bodyGrid + j;

			bodyDef.position.x = offset + spacing * <f32>i;
			bodyDef.position.y = Scene_RandomFloat(scene, 2.0, 6.0);
			bodyDef.position.z = offset + spacing * <f32>j;
			bodyDef.linearVelocity = b3Vec3_zero;
			bodyDef.angularVelocity.x = Scene_RandomFloat(scene, -2.0, 2.0);
			bodyDef.angularVelocity.y = Scene_RandomFloat(scene, -2.0, 2.0);
			bodyDef.angularVelocity.z = Scene_RandomFloat(scene, -2.0, 2.0);

			// Fast enough to need continuous collision against the compound
			if (index % 20 == 0) {
				bodyDef.position.y = 20.0;
				bodyDef.linearVelocity = b3Vec3(0.0, -80.0, 0.0);
			}

			let bodyId = Scene_AddBody(scene, bodyDef);

			let kind = index % 3;
			if (kind == 0) {
				b3CreateSphereShape(bodyId, shapeDef, sphere);
			} else if (kind == 1) {
				b3CreateCapsuleShape(bodyId, shapeDef, capsule);
			} else {
				b3CreateHullShape(bodyId, shapeDef, box.base);
			}
		}
	}
}

class SceneCast {
	point: b3Pos = b3Vec3();
	normal: b3Vec3 = b3Vec3();
	fraction: f32 = 0;
	shapeIndex: i32 = 0;
	hit: bool = false;
}

function Scene_ResetCast(cast: SceneCast): void {
	cast.point = b3Vec3();
	cast.normal = b3Vec3();
	cast.fraction = 0.0;
	cast.shapeIndex = 0;
	cast.hit = false;
}

// The query callbacks carry no context, so their targets sit in module globals
let s_cast: SceneCast = new SceneCast();
let s_queryScene: Scene = new Scene();

function Scene_ClosestCastCallback(
	shapeId: b3ShapeId,
	point: b3Pos,
	normal: b3Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
): f32 {
	// Clipping makes each report closer than the last, so the final one is the closest
	let cast = s_cast;
	cast.point = point;
	cast.normal = normal;
	cast.fraction = fraction;
	cast.shapeIndex = shapeId.index1;
	cast.hit = true;
	return fraction;
}

function Scene_HashCast(scene: Scene, kind: i32, cast: SceneCast): void {
	if (cast.hit == false) {
		Scene_HashQuery(scene, kind, 0);
		return;
	}

	scene.queryHits[kind] += 1;
	Scene_HashQuery(scene, kind, <u32>cast.shapeIndex);
	Scene_HashQuery(scene, kind, reinterpret<u32>(cast.fraction));
	Scene_HashVec3(scene, kind, cast.point);
	Scene_HashVec3(scene, kind, cast.normal);
}

// shared/benchmarks.c StepVillage: a curtain of tilted sphere casts, one per meter, sweeps across
// the compound once per 600 steps
function Scene_StepVillage(scene: Scene, stepIndex: i32): void {
	let ticks = b3GetTicks();

	let villageStepCount = 600;
	let castRadius: f32 = 0.05;
	let castHeight: f32 = 10.0;
	let castLength: f32 = 12.0;

	let halfSpan = scene.halfSpan;
	let t: f32 = <f32>(stepIndex % villageStepCount) / <f32>(villageStepCount - 1);
	let x: f32 = -halfSpan + 2.0 * halfSpan * t;

	let tilt = b3ComputeCosSin((<f32>20.0 / <f32>180.0) * B3_PI);
	let localTranslation = b3Vec3();
	localTranslation.x = castLength * tilt.sine;
	localTranslation.y = -castLength * tilt.cosine;
	localTranslation.z = 0.0;

	// The curtain is laid out in the compound frame
	let q = scene.compoundRotation;
	let translation = b3RotateVector(q, localTranslation);

	let points = new StaticArray<b3Vec3>(1);
	let proxy = new b3ShapeProxy(points, 1, castRadius);
	let filter = b3DefaultQueryFilter();

	let castCount = <i32>(2.0 * halfSpan) + 1;

	for (let i = 0; i < castCount; ++i) {
		let localOrigin = b3Vec3(x, castHeight, -halfSpan + <f32>i);

		Scene_ResetCast(s_cast);
		s_cast.fraction = 1.0;
		let origin = b3ToPos(b3RotateVector(q, localOrigin));

		b3World_CastShape(scene.worldId, origin, proxy, translation, filter, Scene_ClosestCastCallback);
		Scene_HashCast(scene, SCENE_QUERY_SHAPE_CAST, s_cast);
	}

	scene.queryMs[SCENE_QUERY_SHAPE_CAST] += <f64>b3GetMilliseconds(ticks);
}

// A query workload over a static world with every shape type: a rotated village compound, a wave
// mesh, a height field, single hulls, capsules and spheres on a ground box, and a few dynamic
// bodies resting on them. Each step runs closest ray casts, all-hit ray casts, sphere and box
// casts, capsule overlaps and capsule movers, timing and hashing each kind.
function CreateQueries(scene: Scene): void {
	scene.seed = 1234567;

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, -2.0, 10.0);
		let groundId = Scene_AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		let box = b3MakeBoxHull(100.0, 0.5, 70.0);
		b3CreateHullShape(groundId, shapeDef, box.base);
	}

	Scene_CreateVillageGround(scene, 16, 0.3);

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(52.0, 0.0, 0.0);
		let meshBodyId = Scene_AddBody(scene, bodyDef);
		scene.mesh = b3CreateWaveMesh(32, 32, 1.0, 0.5, 0.1, 0.2);
		let shapeDef = b3DefaultShapeDef();
		b3CreateMeshShape(meshBodyId, shapeDef, scene.mesh!, b3Vec3_one);
	}

	{
		let fieldScale = b3Vec3(1.0, 0.6, 1.0);
		scene.heightField = b3CreateWave(21, 21, fieldScale, 0.08, 0.06, false);
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(-72.0, 0.0, -10.0);
		let fieldBodyId = Scene_AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		b3CreateHeightFieldShape(fieldBodyId, shapeDef, scene.heightField!);
	}

	let box = b3MakeBoxHull(0.5, 0.5, 0.5);
	let capsule = b3Capsule(b3Vec3(0.0, -0.5, 0.0), b3Vec3(0.0, 0.5, 0.0), 0.4);
	let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.5);

	{
		let shapeDef = b3DefaultShapeDef();
		for (let i = 0; i < 8; ++i) {
			for (let j = 0; j < 4; ++j) {
				let bodyDef = b3DefaultBodyDef();
				bodyDef.position.x = -28.0 + 8.0 * <f32>i;
				bodyDef.position.y = -1.0;
				bodyDef.position.z = 44.0 + 6.0 * <f32>j;
				let bodyId = Scene_AddBody(scene, bodyDef);

				let kind = (i + j) % 3;
				if (kind == 0) {
					b3CreateHullShape(bodyId, shapeDef, box.base);
				} else if (kind == 1) {
					b3CreateCapsuleShape(bodyId, shapeDef, capsule);
				} else {
					b3CreateSphereShape(bodyId, shapeDef, sphere);
				}
			}
		}
	}

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		let shapeDef = b3DefaultShapeDef();

		for (let i = 0; i < 10; ++i) {
			for (let j = 0; j < 10; ++j) {
				bodyDef.position.x = -27.0 + 6.0 * <f32>i;
				bodyDef.position.y = Scene_RandomFloat(scene, 2.0, 5.0);
				bodyDef.position.z = -27.0 + 6.0 * <f32>j;
				let bodyId = Scene_AddBody(scene, bodyDef);

				let kind = (i * 10 + j) % 3;
				if (kind == 0) {
					b3CreateHullShape(bodyId, shapeDef, box.base);
				} else if (kind == 1) {
					b3CreateCapsuleShape(bodyId, shapeDef, capsule);
				} else {
					b3CreateSphereShape(bodyId, shapeDef, sphere);
				}
			}
		}
	}

	scene.moverCount = SCENE_QUERY_MOVER_COUNT;
	for (let m = 0; m < SCENE_QUERY_MOVER_COUNT; ++m) {
		let position = b3Vec3();
		position.x = -62.0 + 16.0 * <f32>m;
		position.y = 3.0;
		position.z = -4.0 + 2.0 * <f32>m;
		let mover = new SceneMover();
		Scene_InitializeMover(mover, position);
		scene.movers.push(mover);
	}
}

function Scene_RayAllCallback(
	shapeId: b3ShapeId,
	point: b3Pos,
	normal: b3Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
): f32 {
	let scene = s_queryScene;
	scene.queryHits[SCENE_QUERY_RAY_ALL] += 1;
	Scene_HashQuery(scene, SCENE_QUERY_RAY_ALL, <u32>shapeId.index1);
	Scene_HashQuery(scene, SCENE_QUERY_RAY_ALL, <u32>triangleIndex);
	Scene_HashQuery(scene, SCENE_QUERY_RAY_ALL, <u32>childIndex);
	Scene_HashQuery(scene, SCENE_QUERY_RAY_ALL, reinterpret<u32>(fraction));
	Scene_HashVec3(scene, SCENE_QUERY_RAY_ALL, normal);

	// A query from inside a callback, as games do
	if (scene.queryNested == false && scene.queryHits[SCENE_QUERY_RAY_ALL] % 5 == 0) {
		scene.queryNested = true;
		let down = b3Vec3(0.0, -4.0, 0.0);
		let above = b3Vec3(point.x, point.y + 2.0, point.z);
		let result = b3World_CastRayClosest(scene.worldId, above, down, b3DefaultQueryFilter());
		Scene_HashQuery(scene, SCENE_QUERY_RAY_ALL, reinterpret<u32>(result.fraction));
		Scene_HashQuery(scene, SCENE_QUERY_RAY_ALL, <u32>result.shapeId.index1);
		scene.queryNested = false;
	}

	return 1.0;
}

function Scene_OverlapCallback(shapeId: b3ShapeId): bool {
	let scene = s_queryScene;
	scene.queryHits[SCENE_QUERY_OVERLAP] += 1;
	Scene_HashQuery(scene, SCENE_QUERY_OVERLAP, <u32>shapeId.index1);

	// An overlap from inside an overlap callback
	if (scene.queryNested == false && scene.queryHits[SCENE_QUERY_OVERLAP] % 7 == 0) {
		scene.queryNested = true;
		let points = new StaticArray<b3Vec3>(1);
		let proxy = new b3ShapeProxy(points, 1, 1.5);
		let origin = b3Shape_GetAABB(shapeId).upperBound;
		b3World_OverlapShape(scene.worldId, origin, proxy, b3DefaultQueryFilter(), Scene_OverlapCallback);
		scene.queryNested = false;
	}

	return true;
}

function Scene_StepQueries(scene: Scene, stepIndex: i32, timeStep: f32): void {
	let worldId = scene.worldId;
	let filter = b3DefaultQueryFilter();
	let phase: f32 = <f32>(stepIndex % 64) / 64.0;
	s_queryScene = scene;

	{
		let ticks = b3GetTicks();
		let translation = b3Vec3(3.0, -30.0, 2.0);
		for (let i = 0; i < 16; ++i) {
			for (let j = 0; j < 16; ++j) {
				let origin = b3Vec3();
				origin.x = -80.0 + 10.0 * (<f32>i + phase);
				origin.y = 15.0;
				origin.z = -40.0 + 7.0 * (<f32>j + phase);
				let result = b3World_CastRayClosest(worldId, origin, translation, filter);

				let cast = s_cast;
				cast.point = result.point;
				cast.normal = result.normal;
				cast.fraction = result.fraction;
				cast.shapeIndex = result.shapeId.index1;
				cast.hit = result.hit;
				Scene_HashCast(scene, SCENE_QUERY_RAY, cast);
			}
		}
		scene.queryMs[SCENE_QUERY_RAY] += <f64>b3GetMilliseconds(ticks);
	}

	{
		let ticks = b3GetTicks();
		let translation = b3Vec3(180.0, 0.0, 0.0);
		for (let k = 0; k < 32; ++k) {
			let origin = b3Vec3();
			origin.x = -90.0;
			origin.y = 0.3 + 0.1 * <f32>(k % 8);
			origin.z = -40.0 + 3.5 * (<f32>k + phase);
			b3World_CastRay(worldId, origin, translation, filter, Scene_RayAllCallback);
		}
		scene.queryMs[SCENE_QUERY_RAY_ALL] += <f64>b3GetMilliseconds(ticks);
	}

	{
		let ticks = b3GetTicks();
		let spherePoints = new StaticArray<b3Vec3>(1);
		let sphereProxy = new b3ShapeProxy(spherePoints, 1, 0.3);

		let boxPoints = new StaticArray<b3Vec3>(8);
		for (let n = 0; n < 8; ++n) {
			boxPoints[n].x = (n & 1) != 0 ? <f32>0.25 : <f32>-0.25;
			boxPoints[n].y = (n & 2) != 0 ? <f32>0.25 : <f32>-0.25;
			boxPoints[n].z = (n & 4) != 0 ? <f32>0.25 : <f32>-0.25;
		}
		let boxProxy = new b3ShapeProxy(boxPoints, 8, 0.0);

		let translation = b3Vec3(0.0, -20.0, 1.5);
		for (let k = 0; k < 128; ++k) {
			let origin = b3Vec3();
			origin.x = -80.0 + 1.25 * <f32>k + 5.0 * phase;
			origin.y = 8.0;
			origin.z = -30.0 + 0.7 * <f32>k;

			Scene_ResetCast(s_cast);
			s_cast.fraction = 1.0;
			let proxy = (k & 1) != 0 ? boxProxy : sphereProxy;
			b3World_CastShape(worldId, origin, proxy, translation, filter, Scene_ClosestCastCallback);
			Scene_HashCast(scene, SCENE_QUERY_SHAPE_CAST, s_cast);
		}
		scene.queryMs[SCENE_QUERY_SHAPE_CAST] += <f64>b3GetMilliseconds(ticks);
	}

	{
		let ticks = b3GetTicks();
		let capsulePoints = new StaticArray<b3Vec3>(2);
		capsulePoints[0] = b3Vec3(0.0, -0.4, 0.0);
		capsulePoints[1] = b3Vec3(0.0, 0.4, 0.0);
		let proxy = new b3ShapeProxy(capsulePoints, 2, 0.3);
		for (let k = 0; k < 64; ++k) {
			let origin = b3Vec3();
			origin.x = -78.0 + 2.4 * <f32>k + 3.0 * phase;
			origin.y = 0.5;
			origin.z = -20.0 + 1.1 * <f32>k;
			b3World_OverlapShape(worldId, origin, proxy, filter, Scene_OverlapCallback);
		}
		scene.queryMs[SCENE_QUERY_OVERLAP] += <f64>b3GetMilliseconds(ticks);
	}

	{
		let ticks = b3GetTicks();
		for (let m = 0; m < scene.moverCount; ++m) {
			let mover = scene.movers[m];
			let cs = b3ComputeCosSin(<f32>0.785 * <f32>m);
			let forward = b3Vec3(cs.cosine, 0.0, cs.sine);
			let right = b3Vec3(-cs.sine, 0.0, cs.cosine);
			let throttle = b3Vec2(1.0, 0.0);
			if (stepIndex % 120 >= 60) {
				throttle.x = 0.5;
				throttle.y = 1.0;
			}

			if (stepIndex % 90 == 45 && mover.onGround == true) {
				mover.velocity.y = s_moverJumpSpeed;
				mover.onGround = false;
			}

			Scene_SolveMove(scene, mover, timeStep, forward, right, throttle, true);

			Scene_HashVec3(scene, SCENE_QUERY_MOVER, mover.transform.p);
			Scene_HashVec3(scene, SCENE_QUERY_MOVER, mover.velocity);
			Scene_HashQuery(scene, SCENE_QUERY_MOVER, <u32>mover.totalIterations);
			scene.queryHits[SCENE_QUERY_MOVER] += mover.planeCount;
		}
		scene.queryMs[SCENE_QUERY_MOVER] += <f64>b3GetMilliseconds(ticks);
	}
}

// Returns false for an unknown scene name
// Not a sample: two layers of 16 x 16 boxes, each under a dynamic plank that touches all 256, so the plank's
// contacts overflow the graph colors, plus a hub with 40 distance joint spokes that overflows the joint colors.
// Sleep is off so every step solves the whole pile.
function CreatePlankPile(scene: Scene): void {
	b3World_EnableSleeping(scene.worldId, false);
	Scene_AddGroundBox(scene, 30.0);

	let shapeDef = b3DefaultShapeDef();
	let box = b3MakeBoxHull(0.4, 0.4, 0.4);
	let plank = b3MakeBoxHull(8.2, 0.25, 8.2);

	let y: f32 = 0.4;
	for (let layer = 0; layer < 2; ++layer) {
		for (let i = 0; i < 16; ++i) {
			for (let j = 0; j < 16; ++j) {
				let bodyDef = b3DefaultBodyDef();
				bodyDef.type = b3_dynamicBody;
				bodyDef.position = b3Vec3(-7.5 + 1.0 * <f32>i, y, -7.5 + 1.0 * <f32>j);
				let bodyId = Scene_AddBody(scene, bodyDef);
				b3CreateHullShape(bodyId, shapeDef, box.base);
			}
		}

		y += 0.4 + 0.25;
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(0.0, y, 0.0);
		let plankId = Scene_AddBody(scene, bodyDef);
		b3CreateHullShape(plankId, shapeDef, plank.base);
		y += 0.25 + 0.4;
	}

	let hubDef = b3DefaultBodyDef();
	hubDef.type = b3_dynamicBody;
	hubDef.position = b3Vec3(0.0, 10.0, 15.0);
	hubDef.gravityScale = 0.0;
	hubDef.angularVelocity = b3Vec3(0.0, 1.0, 0.2);
	let hubId = Scene_AddBody(scene, hubDef);
	b3CreateSphereShape(hubId, shapeDef, b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.5));

	for (let i = 0; i < 40; ++i) {
		let angle: f32 = (2.0 * B3_PI * <f32>i) / 40.0;
		let cs = b3ComputeCosSin(angle);

		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3Vec3(3.0 * cs.cosine, 10.0 + 0.1 * <f32>(i % 3), 15.0 + 3.0 * cs.sine);
		bodyDef.gravityScale = 0.0;
		let bodyId = Scene_AddBody(scene, bodyDef);
		b3CreateSphereShape(bodyId, shapeDef, b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.15));

		let jointDef = b3DefaultDistanceJointDef();
		jointDef.base.bodyIdA = hubId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.length = 3.0;
		jointDef.enableSpring = i % 2 == 0;
		jointDef.hertz = 2.0;
		jointDef.dampingRatio = 0.3;
		b3CreateDistanceJoint(scene.worldId, jointDef);
	}
}

export function Scene_Create(scene: Scene, name: string, workerCount: i32): bool {
	scene.name = name;
	scene.bulletId = b3_nullBodyId;

	let worldDef = b3DefaultWorldDef();
	worldDef.workerCount = <u32>workerCount;
	scene.worldId = b3CreateWorld(worldDef);

	if (name == "pyramid") {
		CreatePyramid(scene, 10);
	} else if (name == "large_pyramid") {
		CreatePyramid(scene, 100);
	} else if (name == "card_house") {
		CreateCardHouse(scene);
	} else if (name == "chains") {
		CreateChains(scene);
	} else if (name == "capsule_mesh") {
		CreateCapsuleMesh(scene);
	} else if (name == "heightfield") {
		CreateHeightField(scene);
	} else if (name == "bullets") {
		CreateBullets(scene);
	} else if (name == "mover") {
		CreateMover(scene);
	} else if (name == "many_bodies") {
		CreateManyPyramids(scene);
	} else if (name == "plank_pile") {
		CreatePlankPile(scene);
	} else if (name == "mesh_drop") {
		CreateMeshDrop(scene);
	} else if (name == "village") {
		CreateVillage(scene);
	} else if (name == "queries") {
		CreateQueries(scene);
	} else {
		b3DestroyWorld(scene.worldId);
		scene.worldId = b3WorldId();
		return false;
	}

	return true;
}

// Scripted input for this step, then b3World_Step at 60 Hz with 4 sub-steps
export function Scene_Step(scene: Scene, stepIndex: i32): void {
	let timeStep: f32 = <f32>1.0 / <f32>60.0;

	if (scene.name == "bullets" && (stepIndex == 20 || stepIndex == 80)) {
		LaunchBullet(scene);
	}

	if (scene.hasMover) {
		Scene_StepMover(scene, stepIndex, timeStep);
	}

	if (scene.name == "village") {
		Scene_StepVillage(scene, stepIndex);
	} else if (scene.name == "queries") {
		Scene_StepQueries(scene, stepIndex, timeStep);
	}

	b3World_Step(scene.worldId, timeStep, SCENE_SUBSTEP_COUNT);
}

export function Scene_Destroy(scene: Scene): void {
	b3DestroyWorld(scene.worldId);
	scene.worldId = b3WorldId();

	if (scene.mesh != null) {
		b3DestroyMesh(scene.mesh!);
		scene.mesh = null;
	}

	if (scene.heightField != null) {
		b3DestroyHeightField(scene.heightField!);
		scene.heightField = null;
	}

	if (scene.compound != null) {
		b3DestroyCompound(scene.compound!);
		scene.compound = null;
	}

	scene.bodies = new Array<b3BodyId>(0);
	scene.bodyCount = 0;
}
