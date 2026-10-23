// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/mesh_contact.as, mirrors verify/cref/mesh_contact.c

import { vtag, vi32, vu32, vbool, vf32, vend, vseed, vrand64, vrandint, vrandf } from "./common";
import {
	b3AABB,
	b3Add,
	b3MakeQuatFromAxisAngle,
	b3MaxFloat,
	b3MulQuat,
	b3Normalize,
	b3NormalizeQuat,
	b3Quat,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Transform_identity,
	b3Vec3,
} from "../../lib/math_functions";
import { B3_HEIGHT_FIELD_HOLE, b3HeightFieldData, b3HeightFieldDef, b3HullData, b3Mesh, b3MeshData } from "../../lib/collision";
import { b3SurfaceMaterial, b3_capsuleShape, b3_heightShape, b3_hullShape, b3_meshShape, b3_sphereShape } from "../../lib/types";
import { b3ComputeHullAABB, b3CreateCone, b3CreateCylinder, b3CreateHull, b3MakeBoxHull } from "../../lib/hull";
import {
	b3ComputeMeshAABB,
	b3CreateBoxMesh,
	b3CreateGridMesh,
	b3CreatePlatformMesh,
	b3CreateTorusMesh,
	b3CreateWaveMesh,
} from "../../lib/mesh";
import { b3ComputeHeightFieldAABB, b3CreateGrid, b3CreateHeightField, b3CreateWave } from "../../lib/height_field";
import { b3ComputeSphereAABB } from "../../lib/sphere";
import { b3ComputeCapsuleAABB } from "../../lib/capsule";
import { b3Contact, b3_enableSpeculativePoints } from "../../lib/contact";
import { b3TaskContext, b3World } from "../../lib/physics_world";
import { b3Shape } from "../../lib/shape";
import { b3ComputeMeshManifolds } from "../../lib/mesh_contact";

export { frameBoundary } from "./common";

function pv3(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function pab(a: b3AABB): void {
	pv3(a.lowerBound);
	pv3(a.upperBound);
}

function randVec(lo: f32, hi: f32): b3Vec3 {
	let v = b3Vec3();
	v.x = vrandf(lo, hi);
	v.y = vrandf(lo, hi);
	v.z = vrandf(lo, hi);
	return v;
}

function randQuat(): b3Quat {
	let q = b3Quat();
	q.v = randVec(-1.0, 1.0);
	q.s = vrandf(-1.0, 1.0);
	return b3NormalizeQuat(q);
}

// A small rotation about a random axis
function smallRotation(maxAngle: f32): b3Quat {
	let axis = randVec(-1.0, 1.0);
	let angle = vrandf(-maxAngle, maxAngle);
	return b3MakeQuatFromAxisAngle(b3Normalize(axis), angle);
}

const MESH_COUNT = 6;
const FIELD_COUNT = 4;
const HULL_COUNT = 5;

let g_meshes = new Array<b3MeshData>();
let g_fields = new Array<b3HeightFieldData>();
let g_hulls = new Array<b3HullData>();
let g_materials = new StaticArray<b3SurfaceMaterial>(4);
let g_world = new b3World();

export function buildPools(): void {
	g_world.taskContexts.push(new b3TaskContext());
	g_world.frictionCallback = (frictionA: f32, materialA: u64, frictionB: f32, materialB: u64): f32 =>
		Mathf.sqrt(frictionA * frictionB);
	g_world.restitutionCallback = (restitutionA: f32, materialA: u64, restitutionB: f32, materialB: u64): f32 =>
		b3MaxFloat(restitutionA, restitutionB);

	vseed(91);

	g_meshes.push(b3CreateGridMesh(12, 12, 0.5, 3, true)!);
	g_meshes.push(b3CreateWaveMesh(16, 16, 0.4, 0.4, 0.5, 0.7)!);
	g_meshes.push(b3CreateTorusMesh(16, 10, 2.0, 0.6)!);
	g_meshes.push(b3CreateBoxMesh(b3Vec3(0.0, 0.0, 0.0), b3Vec3(2.0, 0.5, 2.0), true)!);
	g_meshes.push(b3CreateGridMesh(60, 60, 0.1, 0, true)!);
	g_meshes.push(b3CreatePlatformMesh(b3Vec3(0.0, 0.0, 0.0), 1.0, 2.0, 4.0)!);

	g_fields.push(b3CreateWave(20, 20, b3Vec3(0.5, 1.0, 0.5), 0.3, 0.4, true));
	g_fields.push(b3CreateGrid(16, 16, b3Vec3(0.6, 1.0, 0.6), false));
	{
		let countX = 14;
		let countZ = 12;
		let heights = new StaticArray<f32>(14 * 12);
		let materials = new StaticArray<u8>(13 * 11);
		for (let i = 0; i < countX * countZ; ++i) {
			heights[i] = vrandf(-0.4, 0.4);
		}
		for (let i = 0; i < (countX - 1) * (countZ - 1); ++i) {
			let m = vrandint(0, 12);
			materials[i] = m == 11 ? <u8>B3_HEIGHT_FIELD_HOLE : <u8>(m % 3);
		}
		let def = new b3HeightFieldDef();
		def.heights = heights;
		def.materialIndices = materials;
		def.scale = b3Vec3(0.5, 1.0, 0.5);
		def.countX = countX;
		def.countZ = countZ;
		def.globalMinimumHeight = -0.5;
		def.globalMaximumHeight = 0.5;
		g_fields.push(b3CreateHeightField(def));
	}

	// Fine enough that a large hull overlaps more triangles than the cache holds
	g_fields.push(b3CreateWave(90, 90, b3Vec3(0.08, 0.3, 0.08), 0.2, 0.3, false));

	g_hulls.push(b3MakeBoxHull(0.4, 0.3, 0.5).base);
	g_hulls.push(b3MakeBoxHull(1.2, 0.4, 1.2).base);
	g_hulls.push(b3CreateCylinder(0.8, 0.4, 0.0, 10)!);
	g_hulls.push(b3CreateCone(0.8, 0.5, 0.1, 8)!);
	{
		let points = new StaticArray<b3Vec3>(24);
		for (let i = 0; i < 24; ++i) {
			points[i] = randVec(-0.5, 0.5);
		}
		g_hulls.push(b3CreateHull(points, 24, 32)!);
	}

	for (let i = 0; i < 4; ++i) {
		let m = b3SurfaceMaterial();
		m.friction = vrandf(0.0, 1.0);
		m.restitution = vrandf(0.0, 1.0);
		m.rollingResistance = vrandf(0.0, 0.3);
		m.tangentVelocity = randVec(-1.0, 1.0);
		m.userMaterialId = vrand64();
		g_materials[i] = m;
	}
}

function printContact(seq: i32, step: i32, touching: bool, contact: b3Contact, shapeTypeB: i32): void {
	let context = g_world.taskContexts[0];
	let triangleCache = contact.meshContact.triangleCache;
	vtag("step");
	vi32(seq);
	vi32(step);
	vbool(touching);
	vi32(contact.manifoldCount);
	vf32(contact.friction);
	vf32(contact.restitution);
	vf32(contact.rollingResistance);
	pv3(contact.tangentVelocity);
	pab(contact.meshContact.queryBounds);
	vi32(triangleCache.length);
	vi32(context.satCallCount);
	vi32(context.satCacheHitCount);
	vend();

	for (let i = 0; i < contact.manifoldCount; ++i) {
		ref m = contact.manifolds[i];
		vtag("m");
		pv3(m.normal);
		vi32(m.pointCount);
		vf32(m.twistImpulse);
		pv3(m.frictionImpulse);
		pv3(m.rollingImpulse);
		vend();
		for (let j = 0; j < m.pointCount; ++j) {
			ref p = m.points[j];
			vtag("p");
			pv3(p.anchorA);
			pv3(p.anchorB);
			vf32(p.separation);
			vf32(p.normalImpulse);
			vf32(p.totalNormalImpulse);
			vf32(p.normalVelocity);
			vf32(p.baseSeparation);
			vu32(p.featureId);
			vi32(p.triangleIndex);
			vbool(p.persisted);
			vend();
		}
	}

	for (let i = 0; i < triangleCache.length; ++i) {
		let tc = triangleCache[i];
		vtag("c");
		vi32(tc.triangleIndex);
		if (shapeTypeB == b3_hullShape) {
			vf32(tc.cache.satCache.separation);
			vu32(tc.cache.satCache.type);
			vu32(tc.cache.satCache.indexA);
			vu32(tc.cache.satCache.indexB);
			vu32(tc.cache.satCache.hit);
		} else if (shapeTypeB == b3_capsuleShape) {
			vf32(tc.cache.simplexCache.metric);
			vu32(tc.cache.simplexCache.count);
			for (let k = 0; k < 4; ++k) {
				vu32(tc.cache.simplexCache.indexA[k]);
				vu32(tc.cache.simplexCache.indexB[k]);
			}
		}
		vend();
	}
}

function computeAABB(shape: b3Shape, xf: b3Transform): b3AABB {
	if (shape.type == b3_sphereShape) {
		return b3ComputeSphereAABB(shape.sphere, xf);
	}
	if (shape.type == b3_capsuleShape) {
		return b3ComputeCapsuleAABB(shape.capsule, xf);
	}
	return b3ComputeHullAABB(shape.hull!, xf);
}

function oneMaterial(material: b3SurfaceMaterial): StaticArray<b3SurfaceMaterial> {
	let materials = new StaticArray<b3SurfaceMaterial>(1);
	materials[0] = material;
	return materials;
}

export function runSequence(seq: i32): void {
	vseed(<u64>(9000 + seq));

	let shapeA = new b3Shape();
	let shapeB = new b3Shape();

	let materialsA = new StaticArray<b3SurfaceMaterial>(3);
	let localBounds: b3AABB;

	let kindA = vrandint(0, 4);
	let isMesh = kindA != 0;
	if (isMesh) {
		let m = vrandint(0, MESH_COUNT);
		let scale = b3Vec3();
		scale.x = vrandf(0.6, 1.6);
		scale.y = vrandf(0.6, 1.6);
		scale.z = vrandf(0.6, 1.6);
		let flip = vrandint(0, 4);
		if (flip == 0) {
			scale.x = -scale.x;
		}
		shapeA.type = b3_meshShape;
		shapeA.mesh = new b3Mesh(g_meshes[m], scale);
		localBounds = b3ComputeMeshAABB(g_meshes[m], b3Transform_identity, scale);
	} else {
		let f = vrandint(0, FIELD_COUNT);
		shapeA.type = b3_heightShape;
		shapeA.heightField = g_fields[f];
		localBounds = b3ComputeHeightFieldAABB(g_fields[f], b3Transform_identity);
	}

	// 0: no materials (the else branch), 1: inline, 2: three materials, 3: three materials through a map
	let materialMode = vrandint(0, 4);
	let materialMap = new Uint16Array(8);
	materialMap[0] = 2;
	materialMap[1] = 0;
	materialMap[2] = 1;
	materialMap[3] = 2;
	materialMap[4] = 1;
	materialMap[5] = 0;
	materialMap[6] = 2;
	materialMap[7] = 1;
	let map: Uint16Array? = null;
	for (let i = 0; i < 3; ++i) {
		let k = vrandint(0, 4);
		materialsA[i] = g_materials[k];
	}
	if (materialMode == 0) {
		shapeA.materialCount = 0;
		shapeA.materials = oneMaterial(materialsA[0]);
	} else if (materialMode == 1) {
		shapeA.materialCount = 1;
		shapeA.materials = oneMaterial(materialsA[0]);
	} else {
		shapeA.materialCount = 3;
		shapeA.materials = materialsA;
		if (materialMode == 3 && isMesh) {
			map = materialMap;
		}
	}

	let flat = false;
	let kindB = vrandint(0, 3);
	if (kindB == 0) {
		shapeB.type = b3_sphereShape;
		shapeB.sphere.center = randVec(-0.1, 0.1);
		shapeB.sphere.radius = vrandf(0.2, 0.8);
	} else if (kindB == 1) {
		shapeB.type = b3_capsuleShape;
		let c = randVec(-0.1, 0.1);
		let h = randVec(-0.7, 0.7);
		shapeB.capsule.center1 = b3Sub(c, h);
		shapeB.capsule.center2 = b3Add(c, h);
		shapeB.capsule.radius = vrandf(0.1, 0.5);

		// A capsule lying flat along the A frame x axis gives collinear cluster points
		let lying = vrandint(0, 3);
		if (lying == 0) {
			flat = true;
			shapeB.capsule.center1 = b3Vec3(-h.x - 0.5, 0.0, 0.0);
			shapeB.capsule.center2 = b3Vec3(h.x + 0.5, 0.0, 0.0);
		}
	} else {
		let h = vrandint(0, HULL_COUNT);
		shapeB.type = b3_hullShape;
		shapeB.hull = g_hulls[h];
	}
	{
		let k = vrandint(0, 4);
		shapeB.materials = oneMaterial(g_materials[k]);
		shapeB.materialCount = 1;
	}

	let xfA = b3Transform_identity;
	let rotatedA = vrandint(0, 3);
	if (rotatedA == 0) {
		xfA.p = randVec(-2.0, 2.0);
		xfA.q = randQuat();
	} else if (rotatedA == 1) {
		xfA.p = randVec(-2.0, 2.0);
	}

	let contact = new b3Contact();
	let speculative = vrandint(0, 3);
	contact.flags = speculative != 0 ? b3_enableSpeculativePoints : 0;

	// Start over the surface in the A frame
	let local = b3Vec3();
	local.x = vrandf(localBounds.lowerBound.x, localBounds.upperBound.x);
	local.z = vrandf(localBounds.lowerBound.z, localBounds.upperBound.z);
	local.y = vrandf(localBounds.lowerBound.y - 0.2, localBounds.upperBound.y + 0.5);
	let qB = randQuat();
	if (flat) {
		qB = xfA.q;
	}
	let velocity = randVec(-0.05, 0.05);
	velocity.y = vrandf(-0.06, 0.0);

	vtag("seq");
	vi32(seq);
	vi32(shapeA.type);
	vi32(shapeB.type);
	vi32(materialMode);
	vi32(speculative);
	vend();

	let stepCount = 40;
	for (let step = 0; step < stepCount; ++step) {
		let jump = vrandint(0, 25);
		if (jump == 0) {
			local.x = vrandf(localBounds.lowerBound.x, localBounds.upperBound.x);
			local.z = vrandf(localBounds.lowerBound.z, localBounds.upperBound.z);
			local.y = vrandf(localBounds.lowerBound.y - 0.2, localBounds.upperBound.y + 0.5);
		} else {
			let noise = randVec(-0.02, 0.02);
			local = b3Add(local, b3Add(velocity, noise));
		}
		let turn = smallRotation(0.15);
		if (flat == false) {
			qB = b3NormalizeQuat(b3MulQuat(turn, qB));
		}

		let xfB = b3Transform();
		xfB.p = b3TransformPoint(xfA, local);
		xfB.q = qB;
		shapeB.aabb = computeAABB(shapeB, xfB);

		let fast = vrandint(0, 4);
		let isFast = fast == 0;

		let touching = b3ComputeMeshManifolds(g_world, 0, contact, shapeA, map, xfA, shapeB, xfB, isFast);
		printContact(seq, step, touching, contact, shapeB.type);

		// Fake solver impulses so the next call has something to preserve
		for (let i = 0; i < contact.manifoldCount; ++i) {
			ref m = contact.manifolds[i];
			m.twistImpulse = vrandf(-1.0, 1.0);
			m.frictionImpulse = randVec(-1.0, 1.0);
			m.rollingImpulse = randVec(-1.0, 1.0);
			for (let j = 0; j < m.pointCount; ++j) {
				m.points[j].normalImpulse = vrandf(0.0, 2.0);
			}
		}
	}
}
