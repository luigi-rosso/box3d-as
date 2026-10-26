// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Every shape kind: a height field floor, a triangle mesh ledge and a compound table, all static, and spheres,
// capsules and hulls dropped onto them.

import {
	B3_PI,
	b3BodyId,
	b3Body_GetPosition,
	b3Body_IsAwake,
	b3Capsule,
	b3CompoundCapsuleDef,
	b3CompoundDef,
	b3CompoundHullDef,
	b3CreateBakedCompoundShape,
	b3CreateBody,
	b3CreateCapsuleShape,
	b3CreateCompound,
	b3CreateCylinder,
	b3CreateHeightField,
	b3CreateHeightFieldShape,
	b3CreateHull,
	b3CreateHullShape,
	b3CreateMesh,
	b3CreateMeshShape,
	b3CreateSphereShape,
	b3CreateWorld,
	b3DefaultBodyDef,
	b3DefaultShapeDef,
	b3DefaultSurfaceMaterial,
	b3DefaultWorldDef,
	b3DestroyWorld,
	b3HeightFieldDef,
	b3MakeBoxHull,
	b3MakeQuatFromAxisAngle,
	b3MeshDef,
	b3Quat_identity,
	b3Sphere,
	b3Transform,
	b3Vec3,
	b3Vec3_axisZ,
	b3Vec3_one,
	b3WorldId,
	b3World_Step,
	b3_dynamicBody,
} from "../lib/box3d";

export class ShapesResult {
	// Rest heights of the sphere on the compound table and of the box on the mesh ledge
	sphereHeight: f32 = 0;
	boxHeight: f32 = 0;
	// Lowest rest height of the bodies in the height field bowl
	lowestInBowl: f32 = 0;
	awakeCount: i32 = 0;
}

// A 17 x 17 height field 1 m apart, a bowl 2.56 m deep at the corners, centered on the origin
function createBowl(worldId: b3WorldId): void {
	let count = 17;
	let heights = new StaticArray<f32>(count * count);
	for (let z = 0; z < count; ++z) {
		for (let x = 0; x < count; ++x) {
			let dx: f32 = <f32>(x - 8);
			let dz: f32 = <f32>(z - 8);
			heights[z * count + x] = 0.02 * (dx * dx + dz * dz);
		}
	}

	let def = new b3HeightFieldDef();
	def.heights = heights;
	def.scale = b3Vec3(1.0, 1.0, 1.0);
	def.countX = count;
	def.countZ = count;
	// Heights are quantized over this range
	def.globalMinimumHeight = 0.0;
	def.globalMaximumHeight = 4.0;
	let heightField = b3CreateHeightField(def);

	// Height fields and compounds go on static bodies
	let bodyDef = b3DefaultBodyDef();
	bodyDef.position = b3Vec3(-8.0, 0.0, -8.0);
	let bodyId = b3CreateBody(worldId, bodyDef);
	b3CreateHeightFieldShape(bodyId, b3DefaultShapeDef(), heightField);
}

// Two triangles, a 4 m square ledge at y = 3 over the bowl's +x side
function createLedge(worldId: b3WorldId): void {
	let def = new b3MeshDef();
	let vertices = new StaticArray<b3Vec3>(4);
	vertices[0] = b3Vec3(4.0, 3.0, -2.0);
	vertices[1] = b3Vec3(4.0, 3.0, 2.0);
	vertices[2] = b3Vec3(8.0, 3.0, 2.0);
	vertices[3] = b3Vec3(8.0, 3.0, -2.0);
	def.vertices = vertices;
	def.vertexCount = 4;
	// Wound counter clockwise about +y, so the triangles face up
	def.indices = [0, 1, 2, 0, 2, 3];
	def.triangleCount = 2;
	let mesh = b3CreateMesh(def, null, 0)!;

	let bodyId = b3CreateBody(worldId, b3DefaultBodyDef());
	b3CreateMeshShape(bodyId, b3DefaultShapeDef(), mesh, b3Vec3_one);
}

// A compound table over the bowl's -x side: a hull top on two capsule legs
function createTable(worldId: b3WorldId): void {
	let material = b3DefaultSurfaceMaterial();
	let top = b3MakeBoxHull(1.5, 0.1, 1.5);
	let topTransform = b3Transform(b3Vec3(-5.0, 4.0, 0.0), b3Quat_identity);

	let def = new b3CompoundDef();
	def.hulls = [new b3CompoundHullDef(top.base, topTransform, material)];
	def.hullCount = 1;

	let legs = new StaticArray<b3CompoundCapsuleDef>(2);
	for (let i = 0; i < 2; ++i) {
		let x: f32 = i == 0 ? -6.2 : -3.8;
		legs[i].capsule = b3Capsule(b3Vec3(x, 0.5, 0.0), b3Vec3(x, 3.8, 0.0), 0.1);
		legs[i].material = material;
	}
	def.capsules = legs;
	def.capsuleCount = 2;

	// The compound copies what it needs from the definition
	let compound = b3CreateCompound(def)!;
	let bodyId = b3CreateBody(worldId, b3DefaultBodyDef());
	b3CreateBakedCompoundShape(bodyId, b3DefaultShapeDef(), compound);
}

export function shapes(): ShapesResult {
	let worldId = b3CreateWorld(b3DefaultWorldDef());
	createBowl(worldId);
	createLedge(worldId);
	createTable(worldId);

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	let shapeDef = b3DefaultShapeDef();
	// Keeps round shapes from rolling around the bowl forever
	shapeDef.baseMaterial.rollingResistance = 0.2;

	bodyDef.position = b3Vec3(-5.0, 6.0, 0.0);
	let sphereId = b3CreateBody(worldId, bodyDef);
	b3CreateSphereShape(sphereId, shapeDef, b3Sphere(b3Vec3(0.0, 0.0, 0.0), 0.5));

	bodyDef.position = b3Vec3(6.0, 5.0, 0.0);
	let boxId = b3CreateBody(worldId, bodyDef);
	let box = b3MakeBoxHull(0.5, 0.5, 0.5);
	b3CreateHullShape(boxId, shapeDef, box.base);

	// Into the bowl: a capsule, a cylinder and a hull built from points
	let bowlBodies = new Array<b3BodyId>();

	bodyDef.position = b3Vec3(-1.0, 4.0, 1.0);
	bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.25 * B3_PI);
	let capsuleId = b3CreateBody(worldId, bodyDef);
	b3CreateCapsuleShape(capsuleId, shapeDef, b3Capsule(b3Vec3(0.0, -0.4, 0.0), b3Vec3(0.0, 0.4, 0.0), 0.3));
	bowlBodies.push(capsuleId);

	bodyDef.position = b3Vec3(1.0, 5.0, -1.0);
	let cylinderId = b3CreateBody(worldId, bodyDef);
	b3CreateHullShape(cylinderId, shapeDef, b3CreateCylinder(0.6, 0.4, 0.0, 12)!);
	bowlBodies.push(cylinderId);

	// A wedge: quickhull keeps the six points, all on the hull
	let points = new StaticArray<b3Vec3>(6);
	for (let i = 0; i < 2; ++i) {
		let x: f32 = i == 0 ? -0.5 : 0.5;
		points[3 * i] = b3Vec3(x, 0.0, -0.4);
		points[3 * i + 1] = b3Vec3(x, 0.0, 0.4);
		points[3 * i + 2] = b3Vec3(x, 0.6, 0.0);
	}
	bodyDef.position = b3Vec3(0.0, 7.0, 0.0);
	bodyDef.rotation = b3Quat_identity;
	let wedgeId = b3CreateBody(worldId, bodyDef);
	b3CreateHullShape(wedgeId, shapeDef, b3CreateHull(points, points.length, 8)!);
	bowlBodies.push(wedgeId);

	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < 600; ++i) {
		b3World_Step(worldId, timeStep, 4);
	}

	let result = new ShapesResult();
	result.sphereHeight = b3Body_GetPosition(sphereId).y;
	result.boxHeight = b3Body_GetPosition(boxId).y;
	result.lowestInBowl = b3Body_GetPosition(bowlBodies[0]).y;
	let all: Array<b3BodyId> = [sphereId, boxId, capsuleId, cylinderId, wedgeId];
	for (let i = 0; i < all.length; ++i) {
		if (b3Body_IsAwake(all[i])) {
			result.awakeCount += 1;
		}
	}
	for (let i = 1; i < bowlBodies.length; ++i) {
		result.lowestInBowl = Mathf.min(result.lowestInBowl, b3Body_GetPosition(bowlBodies[i]).y);
	}

	b3DestroyWorld(worldId);
	return result;
}
