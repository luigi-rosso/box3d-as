// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/contact.as, lib/sensor.as and lib/broad_phase.as, mirrors verify/cref/contact.c.
// The world lives in module globals and the host steps it one call at a time.

import { vtag, vi32, vu32, vu64, vbool, vf32, vend, vseed, vrandf } from "./common";
import { B3_NULL_INDEX } from "../../lib/core";
import {
	b3MakeQuatFromAxisAngle,
	b3MulAdd,
	b3Normalize,
	b3Quat_identity,
	b3Transform,
	b3Vec3,
} from "../../lib/math_functions";
import {
	b3Capsule,
	b3CompoundCapsuleDef,
	b3CompoundData,
	b3CompoundDef,
	b3CompoundHullDef,
	b3CompoundMeshDef,
	b3CompoundSphereDef,
	b3HeightFieldData,
	b3MeshData,
	b3Sphere,
} from "../../lib/collision";
import {
	b3BodyId,
	b3ContactId,
	b3ShapeId,
	b3SurfaceMaterial,
	b3ShapeType,
	b3DefaultBodyDef,
	b3DefaultFilter,
	b3DefaultShapeDef,
	b3DefaultSurfaceMaterial,
	b3DefaultWorldDef,
	b3WorldId,
	b3_bodyTypeCount,
	b3_compoundShape,
	b3_dynamicBody,
	b3_hullShape,
	b3_kinematicBody,
	b3_nullBodyId,
	b3_nullShapeId,
} from "../../lib/types";
import { b3GetIdCount } from "../../lib/id_pool";
import { b3HasTreeMoved } from "../../lib/dynamic_tree";
import { b3MakeBoxHull } from "../../lib/hull";
import { b3CreateWaveMesh, b3DestroyMesh } from "../../lib/mesh";
import { b3CreateWave, b3DestroyHeightField } from "../../lib/height_field";
import { b3CreateCompound, b3DestroyCompound, b3GetCompoundChild } from "../../lib/compound";
import {
	b3CreateWorld,
	b3DestroyWorld,
	b3GetWorldFromId,
	b3World,
	b3World_GetContactEvents,
	b3World_GetSensorEvents,
	b3World_SetCustomFilterCallback,
	b3World_SetPreSolveCallback,
	b3World_Step,
	b3Contact_IsValid,
} from "../../lib/physics_world";
import {
	b3Body_GetPosition,
	b3Body_GetRotation,
	b3Body_SetLinearVelocity,
	b3Body_SetTransform,
	b3CreateBody,
	b3DestroyBody,
} from "../../lib/body";
import {
	b3CreateBakedCompoundShape,
	b3CreateCapsuleShape,
	b3CreateHeightFieldShape,
	b3CreateHullShape,
	b3CreateSphereShape,
	b3DestroyShape,
	b3Shape_SetFilter,
} from "../../lib/shape";
import { b3Contact, b3Contact_GetData, b3_simMeshContact } from "../../lib/contact";
import { b3BodyDef, b3ShapeDef } from "../../lib/types";

export { frameBoundary } from "./common";

const MAX_BODIES: i32 = 128;

class DriverScene {
	worldId: b3WorldId = b3WorldId();
	bodies: StaticArray<b3BodyId> = new StaticArray<b3BodyId>(MAX_BODIES);
	shapes: StaticArray<b3ShapeId> = new StaticArray<b3ShapeId>(MAX_BODIES);
	bodyCount: i32 = 0;
	sensorShapeId: b3ShapeId = b3ShapeId();
	heightField: b3HeightFieldData? = null;
	mesh: b3MeshData? = null;
	compound: b3CompoundData? = null;
	moverStart: i32 = 0;
	base: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(MAX_BODIES);
	velocity: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(MAX_BODIES);
	axis: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(MAX_BODIES);
	rate: StaticArray<f32> = new StaticArray<f32>(MAX_BODIES);
}

function vVec3(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function vShapeId(id: b3ShapeId): void {
	vi32(id.index1);
	vu32(<u32>id.world0);
	vu32(<u32>id.generation);
}

function vContactId(id: b3ContactId): void {
	vi32(id.index1);
	vu32(<u32>id.world0);
	vi32(<i32>id.padding);
	vu32(id.generation);
}

function AddBody(scene: DriverScene, def: b3BodyDef): b3BodyId {
	let id = b3CreateBody(scene.worldId, def);
	scene.bodies[scene.bodyCount] = id;
	scene.shapes[scene.bodyCount] = b3_nullShapeId;
	scene.bodyCount += 1;
	return id;
}

// Shape kind 0 box, 1 sphere, 2 capsule
function AddShape(scene: DriverScene, bodyId: b3BodyId, shapeDef: b3ShapeDef, kind: i32): b3ShapeId {
	let shapeId: b3ShapeId;
	if (kind == 0) {
		let box = b3MakeBoxHull(0.4, 0.4, 0.4);
		shapeId = b3CreateHullShape(bodyId, shapeDef, box.base);
	} else if (kind == 1) {
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.4);
		shapeId = b3CreateSphereShape(bodyId, shapeDef, sphere);
	} else {
		let capsule = b3Capsule(b3Vec3(0.0, -0.3, 0.0), b3Vec3(0.0, 0.3, 0.0), 0.3);
		shapeId = b3CreateCapsuleShape(bodyId, shapeDef, capsule);
	}
	scene.shapes[scene.bodyCount - 1] = shapeId;
	return shapeId;
}

// Set while creating scripted movers.
let s_customFiltering = false;
let s_preSolve = false;

function CustomFilter(shapeIdA: b3ShapeId, shapeIdB: b3ShapeId): bool {
	return (shapeIdA.index1 + shapeIdB.index1) % 3 != 0;
}

function PreSolve(shapeIdA: b3ShapeId, shapeIdB: b3ShapeId, point: b3Vec3, normal: b3Vec3): bool {
	return point.y > -0.25 || normal.y < -0.5;
}

function AddDynamic(
	scene: DriverScene,
	x: f32,
	y: f32,
	z: f32,
	kind: i32,
	sensorEvents: bool,
	hitEvents: bool,
): b3BodyId {
	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	let dx = vrandf(-0.1, 0.1);
	let dz = vrandf(-0.1, 0.1);
	let angle = vrandf(-3.0, 3.0);
	bodyDef.position = b3Vec3(x + dx, y, z + dz);
	bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3(0.0, 1.0, 0.0), angle);
	let bodyId = AddBody(scene, bodyDef);

	let shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.6;
	let rolling: f32 = kind == 1 ? 0.1 : 0.0;
	shapeDef.baseMaterial.rollingResistance = rolling;
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = hitEvents;
	shapeDef.enableSensorEvents = sensorEvents;
	shapeDef.enableCustomFiltering = s_customFiltering;
	shapeDef.enablePreSolveEvents = s_preSolve;
	AddShape(scene, bodyId, shapeDef, kind);
	return bodyId;
}

function AddGround(scene: DriverScene): void {
	let bodyDef = b3DefaultBodyDef();
	bodyDef.position = b3Vec3(0.0, -1.0, 0.0);
	let groundId = AddBody(scene, bodyDef);

	let shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	let box = b3MakeBoxHull(20.0, 1.0, 20.0);
	scene.shapes[scene.bodyCount - 1] = b3CreateHullShape(groundId, shapeDef, box.base);
}

function CreateDriverWorld(): b3WorldId {
	let worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	return b3CreateWorld(worldDef);
}

function CreateMixed(scene: DriverScene): void {
	vseed(101);
	scene.worldId = CreateDriverWorld();
	AddGround(scene);

	{
		let groundId = scene.bodies[0];
		let shapeDef = b3DefaultShapeDef();
		let sphere = b3Sphere(b3Vec3(5.0, 1.5, 0.0), 1.0);
		b3CreateSphereShape(groundId, shapeDef, sphere);
		let capsule = b3Capsule(b3Vec3(-5.0, 1.5, -2.0), b3Vec3(-5.0, 1.5, 2.0), 0.5);
		b3CreateCapsuleShape(groundId, shapeDef, capsule);
	}

	for (let i = 0; i < 4; ++i) {
		for (let j = 0; j < 3; ++j) {
			for (let k = 0; k < 4; ++k) {
				let x: f32 = -3.0 + 2.0 * <f32>i;
				let y: f32 = 1.0 + 1.2 * <f32>j;
				let z: f32 = -3.0 + 2.0 * <f32>k;
				let kind = (i + j + k) % 3;
				AddDynamic(scene, x, y, z, kind, false, (i + k) % 4 == 0);
			}
		}
	}

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = b3Vec3(-8.0, 0.6, 0.0);
		bodyDef.linearVelocity = b3Vec3(2.0, 0.0, 0.0);
		let bodyId = AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		let box = b3MakeBoxHull(0.5, 0.5, 2.0);
		scene.shapes[scene.bodyCount - 1] = b3CreateHullShape(bodyId, shapeDef, box.base);
	}
}

function StepMixed(scene: DriverScene, stepIndex: i32): void {
	if (stepIndex == 50) {
		b3DestroyBody(scene.bodies[6]);
		scene.bodies[6] = b3_nullBodyId;
		b3DestroyBody(scene.bodies[18]);
		scene.bodies[18] = b3_nullBodyId;
	} else if (stepIndex == 70) {
		let filter = b3DefaultFilter();
		filter.maskBits = 0;
		b3Shape_SetFilter(scene.shapes[21], filter, true);
	} else if (stepIndex == 90) {
		b3Body_SetLinearVelocity(scene.bodies[31], b3Vec3(0.0, 12.0, 0.0));
	}
}

function CreateSensor(scene: DriverScene): void {
	vseed(202);
	scene.worldId = CreateDriverWorld();
	AddGround(scene);

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, 1.5, 0.0);
		let bodyId = AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		let box = b3MakeBoxHull(2.0, 1.0, 2.0);
		scene.shapes[scene.bodyCount - 1] = b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = b3Vec3(-6.0, 1.0, 0.0);
		bodyDef.linearVelocity = b3Vec3(1.5, 0.0, 0.0);
		let bodyId = AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 1.0);
		scene.sensorShapeId = b3CreateSphereShape(bodyId, shapeDef, sphere);
		scene.shapes[scene.bodyCount - 1] = scene.sensorShapeId;
	}

	for (let i = 0; i < 3; ++i) {
		for (let k = 0; k < 3; ++k) {
			for (let j = 0; j < 2; ++j) {
				let x: f32 = -1.5 + 1.5 * <f32>i;
				let y: f32 = 4.0 + 1.5 * <f32>j;
				let z: f32 = -1.5 + 1.5 * <f32>k;
				let kind = (i + j + k) % 3;
				AddDynamic(scene, x, y, z, kind, i + k != 2, false);
			}
		}
	}

	for (let i = 0; i < 4; ++i) {
		let x: f32 = -4.5 + 0.9 * <f32>i;
		AddDynamic(scene, x, 0.4, 1.2, i % 3, true, false);
	}
}

function StepSensor(scene: DriverScene, stepIndex: i32): void {
	if (stepIndex == 60) {
		b3DestroyShape(scene.sensorShapeId, false);
		scene.shapes[2] = b3_nullShapeId;
	} else if (stepIndex == 75) {
		b3DestroyBody(scene.bodies[5]);
		scene.bodies[5] = b3_nullBodyId;
	}
}

// Capsule, hull, sphere and mesh children.
function MakeCompound(scene: DriverScene): void {
	let material = b3DefaultSurfaceMaterial();
	material.friction = 0.5;

	let capsules = new StaticArray<b3CompoundCapsuleDef>(2);
	capsules[0].capsule = b3Capsule(b3Vec3(-3.0, 0.5, -2.0), b3Vec3(-3.0, 0.5, 2.0), 0.5);
	capsules[0].material = material;
	capsules[1].capsule = b3Capsule(b3Vec3(-2.0, 0.5, 3.0), b3Vec3(2.0, 0.5, 3.0), 0.4);
	capsules[1].material = material;
	capsules[1].material.friction = 0.8;

	let box = b3MakeBoxHull(1.0, 0.5, 1.0);
	let material1 = material;
	material1.restitution = 0.3;
	let hulls = new StaticArray<b3CompoundHullDef>(2);
	hulls[0] = new b3CompoundHullDef(box.base, b3Transform(b3Vec3(0.0, 0.5, 0.0), b3Quat_identity), material);
	hulls[1] = new b3CompoundHullDef(
		box.base,
		b3Transform(b3Vec3(3.0, 0.5, -1.0), b3MakeQuatFromAxisAngle(b3Vec3(0.0, 1.0, 0.0), 0.5)),
		material1,
	);

	let spheres = new StaticArray<b3CompoundSphereDef>(2);
	spheres[0].sphere = b3Sphere(b3Vec3(3.0, 0.6, 3.0), 0.6);
	spheres[0].material = material;
	spheres[1].sphere = b3Sphere(b3Vec3(-3.0, 0.6, -3.5), 0.6);
	spheres[1].material = material;

	scene.mesh = b3CreateWaveMesh(10, 10, 1.0, 0.2, 0.3, 0.2);
	let meshes = new StaticArray<b3CompoundMeshDef>(1);
	meshes[0] = new b3CompoundMeshDef(scene.mesh!);
	meshes[0].transform = b3Transform(b3Vec3(-5.0, -0.5, -5.0), b3Quat_identity);
	meshes[0].scale = b3Vec3(1.0, 1.0, 1.0);
	let meshMaterials = new StaticArray<b3SurfaceMaterial>(1);
	meshMaterials[0] = material;
	meshes[0].materials = meshMaterials;
	meshes[0].materialCount = 1;

	let compoundDef = new b3CompoundDef();
	compoundDef.capsules = capsules;
	compoundDef.capsuleCount = 2;
	compoundDef.hulls = hulls;
	compoundDef.hullCount = 2;
	compoundDef.meshes = meshes;
	compoundDef.meshCount = 1;
	compoundDef.spheres = spheres;
	compoundDef.sphereCount = 2;
	scene.compound = b3CreateCompound(compoundDef);
}

function CreateCompound(scene: DriverScene): void {
	vseed(303);
	scene.worldId = CreateDriverWorld();
	MakeCompound(scene);

	{
		let bodyDef = b3DefaultBodyDef();
		let groundId = AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		scene.shapes[scene.bodyCount - 1] = b3CreateBakedCompoundShape(groundId, shapeDef, scene.compound!);
	}

	for (let i = 0; i < 4; ++i) {
		for (let k = 0; k < 4; ++k) {
			for (let j = 0; j < 2; ++j) {
				let x: f32 = -3.0 + 2.0 * <f32>i;
				let y: f32 = 2.5 + 1.2 * <f32>j;
				let z: f32 = -3.0 + 2.0 * <f32>k;
				let kind = (i + j + 2 * k) % 3;
				AddDynamic(scene, x, y, z, kind, false, i == k);
			}
		}
	}
}

function CreateHeightField(scene: DriverScene): void {
	vseed(404);
	scene.worldId = CreateDriverWorld();

	let fieldCount = 13;
	let fieldScale = b3Vec3(1.0, 0.6, 1.0);
	scene.heightField = b3CreateWave(fieldCount, fieldCount, fieldScale, 0.08, 0.06, false);

	{
		let extent: f32 = fieldScale.x * <f32>(fieldCount - 1);
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position.x = -0.5 * extent;
		bodyDef.position.z = -0.5 * extent;
		let groundId = AddBody(scene, bodyDef);

		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableHitEvents = true;
		scene.shapes[scene.bodyCount - 1] = b3CreateHeightFieldShape(groundId, shapeDef, scene.heightField!);
	}

	for (let i = 0; i < 4; ++i) {
		for (let k = 0; k < 4; ++k) {
			let x: f32 = -3.0 + 2.0 * <f32>i;
			let z: f32 = -3.0 + 2.0 * <f32>k;
			let kind = (i + k) % 3;
			AddDynamic(scene, x, 2.0, z, kind, false, false);
		}
	}
}

function CreateScripted(scene: DriverScene): void {
	vseed(505);
	scene.worldId = CreateDriverWorld();
	AddGround(scene);

	{
		let groundId = scene.bodies[0];
		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		let sphere = b3Sphere(b3Vec3(5.0, 1.5, 0.0), 1.0);
		b3CreateSphereShape(groundId, shapeDef, sphere);
		let capsule = b3Capsule(b3Vec3(-5.0, 1.5, -2.0), b3Vec3(-5.0, 1.5, 2.0), 0.5);
		b3CreateCapsuleShape(groundId, shapeDef, capsule);
	}

	MakeCompound(scene);
	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, 0.0, 8.0);
		let bodyId = AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		scene.shapes[scene.bodyCount - 1] = b3CreateBakedCompoundShape(bodyId, shapeDef, scene.compound!);
	}

	{
		let fieldScale = b3Vec3(1.0, 0.6, 1.0);
		scene.heightField = b3CreateWave(13, 13, fieldScale, 0.08, 0.06, false);
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(-18.0, -0.5, -6.0);
		let bodyId = AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.enableContactEvents = true;
		scene.shapes[scene.bodyCount - 1] = b3CreateHeightFieldShape(bodyId, shapeDef, scene.heightField!);
	}

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(0.0, 1.5, -6.0);
		let bodyId = AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		let box = b3MakeBoxHull(2.0, 1.0, 2.0);
		scene.shapes[scene.bodyCount - 1] = b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	{
		let bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_kinematicBody;
		bodyDef.position = b3Vec3(-8.0, 1.0, 0.0);
		let bodyId = AddBody(scene, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		let sphere = b3Sphere(b3Vec3(0.0, 0.0, 0.0), 1.0);
		scene.sensorShapeId = b3CreateSphereShape(bodyId, shapeDef, sphere);
		scene.shapes[scene.bodyCount - 1] = scene.sensorShapeId;
	}

	scene.moverStart = scene.bodyCount;
	for (let i = 0; i < 3; ++i) {
		for (let j = 0; j < 3; ++j) {
			for (let k = 0; k < 3; ++k) {
				let x: f32 = -4.0 + 4.0 * <f32>i;
				let y: f32 = 0.5 + 1.5 * <f32>j;
				let z: f32 = -4.0 + 4.0 * <f32>k;
				let kind = (i + j + k) % 3;
				let moverIndex = 9 * i + 3 * j + k;
				s_customFiltering = moverIndex % 4 == 1;
				s_preSolve = moverIndex % 4 == 2;
				let bodyId = AddDynamic(scene, x, y, z, kind, true, (i + j) % 3 == 0);
				let index = scene.bodyCount - 1;
				scene.base[index] = b3Body_GetPosition(bodyId);
				let vx = vrandf(-2.0, 2.0);
				let vy = vrandf(-1.0, 1.0);
				let vz = vrandf(-2.0, 2.0);
				scene.velocity[index] = b3Vec3(vx, vy, vz);
				let ax = vrandf(0.2, 1.0);
				let ay = vrandf(0.2, 1.0);
				let az = vrandf(0.2, 1.0);
				scene.axis[index] = b3Normalize(b3Vec3(ax, ay, az));
				scene.rate[index] = vrandf(-2.0, 2.0);
			}
		}
	}

	s_customFiltering = false;
	s_preSolve = false;
	b3World_SetCustomFilterCallback(scene.worldId, CustomFilter);
	b3World_SetPreSolveCallback(scene.worldId, PreSolve);
}

function StepScripted(scene: DriverScene, stepIndex: i32): void {
	let one: f32 = 1.0;
	let hertz: f32 = 60.0;
	let t: f32 = <f32>(stepIndex + 1) * (one / hertz);

	if (scene.sensorShapeId.index1 != 0) {
		let p = b3Vec3(-8.0 + 4.0 * t, 1.0, 0.0);
		b3Body_SetTransform(scene.bodies[4], p, b3Quat_identity);
	}

	for (let i = scene.moverStart; i < scene.bodyCount; ++i) {
		if (scene.bodies[i].index1 == 0) {
			continue;
		}

		let p = b3MulAdd(scene.base[i], t, scene.velocity[i]);
		let q = b3MakeQuatFromAxisAngle(scene.axis[i], scene.rate[i] * t);
		b3Body_SetTransform(scene.bodies[i], p, q);
	}

	let mover = scene.moverStart;
	if (stepIndex == 80) {
		b3DestroyBody(scene.bodies[mover + 4]);
		scene.bodies[mover + 4] = b3_nullBodyId;
	} else if (stepIndex == 100) {
		let filter = b3DefaultFilter();
		filter.maskBits = 0;
		b3Shape_SetFilter(scene.shapes[mover + 10], filter, true);
	} else if (stepIndex == 120) {
		b3DestroyShape(scene.shapes[3], false);
		scene.shapes[3] = b3_nullShapeId;
	} else if (stepIndex == 140) {
		b3Shape_SetFilter(scene.shapes[mover + 10], b3DefaultFilter(), true);
	} else if (stepIndex == 160) {
		b3DestroyShape(scene.sensorShapeId, false);
		scene.sensorShapeId = b3_nullShapeId;
		scene.shapes[4] = b3_nullShapeId;
	}
}


function UsesSatCache(world: b3World, contact: b3Contact): bool {
	let shapeA = world.shapes[contact.shapeIdA];
	let shapeB = world.shapes[contact.shapeIdB];
	let typeA: b3ShapeType = shapeA.type;
	if (typeA == b3_compoundShape) {
		let child = b3GetCompoundChild(shapeA.compound!, contact.childIndex);
		typeA = child.type;
	}
	return typeA == b3_hullShape && shapeB.type == b3_hullShape;
}

function DumpWorld(scene: DriverScene): void {
	let world = b3GetWorldFromId(scene.worldId);
	let bp = world.broadPhase;

	vtag("bp");
	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		vi32(bp.trees[i].proxyCount);
		vi32(bp.trees[i].nodeEnd);
		vbool(b3HasTreeMoved(bp.trees[i]));
	}
	vu32(bp.pairSet.count);
	vend();

	{
		let taskContext = world.taskContexts[0];
		vtag("pairs");
		vi32(taskContext.pairKeys.length);
		for (let i = 0; i < taskContext.pairKeys.length; ++i) {
			vu64(taskContext.pairKeys[i]);
		}
		vend();
	}

	vtag("contacts");
	vi32(world.contacts.length);
	vi32(b3GetIdCount(world.contactIdPool));
	vend();

	for (let i = 0; i < world.contacts.length; ++i) {
		let c = world.contacts[i];
		vtag("c");
		vi32(i);
		vu32(c.generation);
		vi32(c.contactId);
		if (c.contactId == B3_NULL_INDEX) {
			vend();
			continue;
		}

		vi32(c.setIndex);
		vi32(c.colorIndex);
		vi32(c.localIndex);
		vi32(c.shapeIdA);
		vi32(c.shapeIdB);
		vi32(c.childIndex);
		vi32(c.islandId);
		vi32(c.encodedBodySimA);
		vi32(c.encodedBodySimB);
		vu32(c.flags);
		for (let e = 0; e < 2; ++e) {
			vi32(c.edges[e].bodyId);
			vi32(c.edges[e].prevKey);
			vi32(c.edges[e].nextKey);
		}
		vf32(c.friction);
		vf32(c.restitution);
		vf32(c.rollingResistance);
		vVec3(c.tangentVelocity);
		vi32(c.manifoldCount);
		vend();

		if ((c.flags & b3_simMeshContact) != 0) {
			vtag("mesh");
			vi32(c.meshContact.triangleCache.length);
			vend();
		} else if (UsesSatCache(world, c)) {
			let cache = c.convexContact.cache.satCache;
			vtag("sat");
			vf32(cache.separation);
			vu32(<u32>cache.type);
			vu32(<u32>cache.indexA);
			vu32(<u32>cache.indexB);
			vu32(<u32>cache.hit);
			vend();
		} else {
			let cache = c.convexContact.cache.simplexCache;
			vtag("simplex");
			vu32(<u32>cache.count);
			for (let k = 0; k < <i32>cache.count; ++k) {
				vu32(<u32>cache.indexA[k]);
				vu32(<u32>cache.indexB[k]);
			}
			vend();
		}

		for (let m = 0; m < c.manifoldCount; ++m) {
			let manifold = c.manifolds[m];
			vtag("m");
			vVec3(manifold.normal);
			vi32(manifold.pointCount);
			vf32(manifold.twistImpulse);
			vVec3(manifold.frictionImpulse);
			vVec3(manifold.rollingImpulse);
			vend();
			for (let p = 0; p < manifold.pointCount; ++p) {
				let mp = manifold.points[p];
				vtag("p");
				vVec3(mp.anchorA);
				vVec3(mp.anchorB);
				vf32(mp.separation);
				vf32(mp.normalImpulse);
				vf32(mp.totalNormalImpulse);
				vf32(mp.normalVelocity);
				vu32(mp.featureId);
				vi32(mp.triangleIndex);
				vbool(mp.persisted);
				vend();
			}
		}
	}

	vtag("bodies");
	vi32(world.bodies.length);
	vend();
	for (let i = 0; i < world.bodies.length; ++i) {
		let body = world.bodies[i];
		vtag("b");
		vi32(i);
		vi32(body.id);
		if (body.id == B3_NULL_INDEX) {
			vend();
			continue;
		}
		vi32(body.setIndex);
		vi32(body.headContactKey);
		vi32(body.contactCount);
		let bodyId = b3BodyId();
		bodyId.index1 = i + 1;
		bodyId.world0 = world.worldId;
		bodyId.generation = body.generation;
		vVec3(b3Body_GetPosition(bodyId));
		let q = b3Body_GetRotation(bodyId);
		vVec3(q.v);
		vf32(q.s);
		vend();
	}

	let contactEvents = b3World_GetContactEvents(scene.worldId);
	vtag("begin");
	vi32(contactEvents.beginCount);
	vend();
	for (let i = 0; i < contactEvents.beginCount; ++i) {
		let event = contactEvents.beginEvents[i];
		vtag("cb");
		vShapeId(event.shapeIdA);
		vShapeId(event.shapeIdB);
		vContactId(event.contactId);
		let valid = b3Contact_IsValid(event.contactId);
		vbool(valid);
		if (valid) {
			let data = b3Contact_GetData(event.contactId);
			vShapeId(data.shapeIdA);
			vShapeId(data.shapeIdB);
			vi32(data.manifoldCount);
			if (data.manifoldCount > 0) {
				let manifolds = data.manifolds!;
				vi32(manifolds[0].pointCount);
				vVec3(manifolds[0].normal);
			}
		}
		vend();
	}

	vtag("end");
	vi32(contactEvents.endCount);
	vend();
	for (let i = 0; i < contactEvents.endCount; ++i) {
		let event = contactEvents.endEvents[i];
		vtag("ce");
		vShapeId(event.shapeIdA);
		vShapeId(event.shapeIdB);
		vContactId(event.contactId);
		vend();
	}

	vtag("hit");
	vi32(contactEvents.hitCount);
	vend();
	for (let i = 0; i < contactEvents.hitCount; ++i) {
		let event = contactEvents.hitEvents[i];
		vtag("ch");
		vShapeId(event.shapeIdA);
		vShapeId(event.shapeIdB);
		vContactId(event.contactId);
		vVec3(event.point);
		vVec3(event.normal);
		vf32(event.approachSpeed);
		vend();
	}

	let sensorEvents = b3World_GetSensorEvents(scene.worldId);
	vtag("sensor");
	vi32(sensorEvents.beginCount);
	vi32(sensorEvents.endCount);
	vend();
	for (let i = 0; i < sensorEvents.beginCount; ++i) {
		vtag("sb");
		vShapeId(sensorEvents.beginEvents[i].sensorShapeId);
		vShapeId(sensorEvents.beginEvents[i].visitorShapeId);
		vend();
	}
	for (let i = 0; i < sensorEvents.endCount; ++i) {
		vtag("se");
		vShapeId(sensorEvents.endEvents[i].sensorShapeId);
		vShapeId(sensorEvents.endEvents[i].visitorShapeId);
		vend();
	}

	vtag("sensors");
	vi32(world.sensors.length);
	vend();
	for (let i = 0; i < world.sensors.length; ++i) {
		let sensor = world.sensors[i];
		vtag("s");
		vi32(sensor.shapeId);
		vi32(sensor.hits.length);
		vi32(sensor.overlaps1.length);
		vi32(sensor.overlaps2.length);
		for (let k = 0; k < sensor.overlaps2.length; ++k) {
			vi32(sensor.overlaps2[k].shapeId);
			vu32(<u32>sensor.overlaps2[k].generation);
		}
		vend();
	}
}

// Host protocol: begin(scene) once, then step(i) per step with a frame boundary between calls,
// then finish(). Scene order and step counts are in verify/contact.mjs.
let g_scene = new DriverScene();
let g_sceneIndex = 0;

// The C driver built with CONTACT_COLLIDE_ONLY runs only the scripted scene, index 4.
export function sceneCount(): i32 {
	return 5;
}

export function sceneSteps(index: i32): i32 {
	if (index == 0) return 150;
	if (index == 4) return 200;
	return 120;
}

export function begin(index: i32): void {
	g_scene = new DriverScene();
	g_sceneIndex = index;
	let name = "";
	if (index == 0) {
		CreateMixed(g_scene);
		name = "mixed";
	} else if (index == 1) {
		CreateSensor(g_scene);
		name = "sensor";
	} else if (index == 2) {
		CreateCompound(g_scene);
		name = "compound";
	} else if (index == 3) {
		CreateHeightField(g_scene);
		name = "heightfield";
	} else {
		CreateScripted(g_scene);
		name = "scripted";
	}

	vtag("scene");
	vtag(name);
	vi32(g_scene.bodyCount);
	vend();
	DumpWorld(g_scene);
}

export function step(stepIndex: i32): void {
	if (g_sceneIndex == 0) {
		StepMixed(g_scene, stepIndex);
	} else if (g_sceneIndex == 1) {
		StepSensor(g_scene, stepIndex);
	} else if (g_sceneIndex == 4) {
		StepScripted(g_scene, stepIndex);
	}

	let one: f32 = 1.0;
	let hertz: f32 = 60.0;
	b3World_Step(g_scene.worldId, one / hertz, 4);
	vtag("step");
	vi32(stepIndex);
	vend();
	DumpWorld(g_scene);
}

export function finish(): void {
	b3DestroyWorld(g_scene.worldId);

	let heightField = g_scene.heightField;
	if (heightField != null) {
		b3DestroyHeightField(heightField);
	}

	let compound = g_scene.compound;
	if (compound != null) {
		b3DestroyCompound(compound);
	}

	let mesh = g_scene.mesh;
	if (mesh != null) {
		b3DestroyMesh(mesh);
	}
}
