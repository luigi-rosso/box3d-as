// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for the compound part of lib/compound.as, mirrors verify/cref/compound.c

import { vtag, vi32, vu32, vbool, vf32, vu64, vend, vseed, vrand32, vrand64, vrandint, vrandf } from "./common";
import { B3_MAX_CHILD_SHAPES } from "../../lib/constants";
import {
	b3AABB,
	b3Add,
	b3MulSV,
	b3NormalizeQuat,
	b3Quat,
	b3RotateVector,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Vec3,
	b3Vec3_zero,
} from "../../lib/math_functions";
import {
	b3Capsule,
	b3CastOutput,
	b3CompoundCapsuleDef,
	b3CompoundData,
	b3CompoundDef,
	b3CompoundHullDef,
	b3CompoundMeshDef,
	b3CompoundSphereDef,
	b3HullData,
	b3MeshData,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeProxy,
	b3TOIInput,
} from "../../lib/collision";
import { b3SurfaceMaterial, b3_capsuleShape, b3_hullShape, b3_meshShape, b3_sphereShape } from "../../lib/types";
import { b3CreateCone, b3CreateCylinder, b3CreateHull, b3MakeBoxHull, b3MakeOffsetBoxHull } from "../../lib/hull";
import { b3CreateBoxMesh, b3CreateGridMesh, b3CreatePlatformMesh, b3CreateTorusMesh, b3CreateWaveMesh } from "../../lib/mesh";
import { b3MakePointProxy, b3MakeSegmentProxy, b3TimeOfImpact } from "../../lib/distance";
import {
	b3CollideMoverAndCompound,
	b3ComputeCompoundAABB,
	b3CreateCompound,
	b3DestroyCompound,
	b3GetCompoundCapsule,
	b3GetCompoundChild,
	b3GetCompoundHull,
	b3GetCompoundMaterials,
	b3GetCompoundMesh,
	b3GetCompoundSphere,
	b3MakeCompoundChildSweep,
	b3OverlapCompound,
	b3QueryCompound,
	b3RayCastCompound,
	b3ShapeCastCompound,
} from "../../lib/compound";

export { frameBoundary } from "./common";

function pv3(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function pq(q: b3Quat): void {
	pv3(q.v);
	vf32(q.s);
}

function pxf(t: b3Transform): void {
	pv3(t.p);
	pq(t.q);
}

function pab(a: b3AABB): void {
	pv3(a.lowerBound);
	pv3(a.upperBound);
}

function pcast(o: b3CastOutput): void {
	vbool(o.hit);
	vf32(o.fraction);
	pv3(o.normal);
	pv3(o.point);
	vi32(o.iterations);
	vi32(o.triangleIndex);
	vi32(o.childIndex);
	vi32(o.materialIndex);
}

function pmat(m: b3SurfaceMaterial): void {
	vf32(m.friction);
	vf32(m.restitution);
	vf32(m.rollingResistance);
	pv3(m.tangentVelocity);
	vu64(m.userMaterialId);
	vu32(m.customColor);
	vu32(m.padding);
}

function phull(h: b3HullData): void {
	vu64(h.version);
	vu64(h.hash);
	vi32(h.byteCount);
	vi32(h.vertexCount);
	vi32(h.edgeCount);
	vi32(h.faceCount);
	pab(h.aabb);
	vf32(h.volume);
}

function pmesh(m: b3MeshData): void {
	vu64(m.version);
	vu64(m.hash);
	vi32(m.byteCount);
	vi32(m.vertexCount);
	vi32(m.triangleCount);
	vi32(m.materialCount);
	pab(m.bounds);
}

function randVec(lo: f32, hi: f32): b3Vec3 {
	let v = b3Vec3();
	v.x = vrandf(lo, hi);
	v.y = vrandf(lo, hi);
	v.z = vrandf(lo, hi);
	return v;
}

function randInBox(box: b3AABB, pad: f32): b3Vec3 {
	let v = b3Vec3();
	v.x = vrandf(box.lowerBound.x - pad, box.upperBound.x + pad);
	v.y = vrandf(box.lowerBound.y - pad, box.upperBound.y + pad);
	v.z = vrandf(box.lowerBound.z - pad, box.upperBound.z + pad);
	return v;
}

function randQuat(): b3Quat {
	let q = b3Quat();
	q.v = randVec(-1.0, 1.0);
	q.s = vrandf(-1.0, 1.0);
	return b3NormalizeQuat(q);
}

function randTransform(range: f32): b3Transform {
	let t = b3Transform();
	t.p = randVec(-range, range);
	t.q = randQuat();
	return t;
}

const HULL_COUNT = 10;
const MESH_COUNT = 7;
const MATERIAL_COUNT = 7;

let g_hulls: Array<b3HullData> = new Array<b3HullData>();
let g_meshes: Array<b3MeshData> = new Array<b3MeshData>();
let g_materials = new StaticArray<b3SurfaceMaterial>(MATERIAL_COUNT);

export function buildPools(): void {
	vseed(77);

	g_hulls.push(b3MakeBoxHull(0.5, 0.5, 0.5).base);
	g_hulls.push(b3MakeBoxHull(0.5, 0.5, 0.5).base);
	g_hulls.push(b3MakeOffsetBoxHull(0.3, 0.6, 0.2, b3Vec3(0.1, -0.2, 0.3)).base);
	g_hulls.push(b3CreateCylinder(1.0, 0.4, 0.0, 12)!);
	g_hulls.push(b3CreateCone(1.0, 0.5, 0.2, 10)!);

	let points = new StaticArray<b3Vec3>(40);
	for (let i = 5; i < HULL_COUNT; ++i) {
		let count = vrandint(8, 40);
		for (let j = 0; j < count; ++j) {
			points[j] = randVec(-0.8, 0.8);
		}
		g_hulls.push(b3CreateHull(points, count, 32)!);
	}

	// the same point set twice gives identical content
	g_hulls[9] = b3CreateHull(g_hulls[8].points, g_hulls[8].vertexCount, 32)!;

	g_meshes.push(b3CreateBoxMesh(b3Vec3(0.0, 0.0, 0.0), b3Vec3(1.0, 0.5, 1.0), true)!);
	g_meshes.push(b3CreateBoxMesh(b3Vec3(0.0, 0.0, 0.0), b3Vec3(1.0, 0.5, 1.0), true)!);
	g_meshes.push(b3CreateGridMesh(6, 5, 0.5, 3, true)!);
	g_meshes.push(b3CreateTorusMesh(12, 8, 1.0, 0.25)!);
	g_meshes.push(b3CreateWaveMesh(8, 8, 0.4, 0.3, 0.5, 0.7)!);
	g_meshes.push(b3CreatePlatformMesh(b3Vec3(0.0, -0.5, 0.0), 0.5, 1.0, 2.0)!);
	g_meshes.push(b3CreateGridMesh(4, 4, 0.7, 5, false)!);

	for (let i = 0; i < 5; ++i) {
		let m = b3SurfaceMaterial();
		m.friction = vrandf(0.0, 1.0);
		m.restitution = vrandf(0.0, 1.0);
		m.rollingResistance = vrandf(0.0, 0.2);
		m.tangentVelocity = randVec(-1.0, 1.0);
		m.userMaterialId = vrand64();
		m.customColor = vrand32();
		g_materials[i] = m;
	}

	// a repeat and a signed zero friction, which memcmp tells apart from +0
	g_materials[0].friction = 0.0;
	g_materials[5] = g_materials[2];
	g_materials[6] = g_materials[0];
	g_materials[6].friction = -0.0;

	for (let i = 0; i < HULL_COUNT; ++i) {
		vtag("poolhull");
		vi32(i);
		phull(g_hulls[i]);
		vend();
	}

	for (let i = 0; i < MESH_COUNT; ++i) {
		vtag("poolmesh");
		vi32(i);
		pmesh(g_meshes[i]);
		vend();
	}
}

function printChild(compound: b3CompoundData, childIndex: i32): void {
	let child = b3GetCompoundChild(compound, childIndex);
	vtag("child");
	vi32(childIndex);
	vi32(child.type);
	pxf(child.transform);
	vi32(child.materialCount);
	let indices = child.materialIndices!;
	for (let i = 0; i < child.materialCount; ++i) {
		vu32(<u32>indices[i]);
	}

	switch (child.type) {
		case b3_capsuleShape:
			pv3(child.capsule.center1);
			pv3(child.capsule.center2);
			vf32(child.capsule.radius);
			break;
		case b3_hullShape:
			phull(child.hull!);
			break;
		case b3_meshShape:
			pv3(child.mesh!.scale);
			pmesh(child.mesh!.data);
			break;
		case b3_sphereShape:
			pv3(child.sphere.center);
			vf32(child.sphere.radius);
			break;
		default:
			break;
	}
	vend();
}

function align8(x: i32): i32 {
	return (x + 7) & ~7;
}

function printCompound(c: b3CompoundData?): void {
	vtag("compound");
	vbool(c != null);
	vend();
	if (c == null) {
		return;
	}

	vtag("head");
	vu64(c.version);
	vi32(c.byteCount);
	pab(c.bounds);
	vi32(c.treeHeight);
	vi32(c.nodeOffset);
	vi32(c.nodeCount);
	vi32(c.materialOffset);
	vi32(c.materialCount);
	vi32(c.capsuleOffset);
	vi32(c.capsuleCount);
	vi32(c.hullOffset);
	vi32(c.hullCount);
	vi32(c.sharedHullCount);
	vi32(c.meshOffset);
	vi32(c.meshCount);
	vi32(c.sharedMeshCount);
	vi32(c.sphereOffset);
	vi32(c.sphereCount);
	vend();

	let nodes = c.nodes;
	for (let i = 0; i < c.nodeCount; ++i) {
		vtag("node");
		pab(nodes[i].aabb);
		vu32(nodes[i].flagIndex);
		// C's union: the shape index on a leaf, the height otherwise
		vi32((nodes[i].flagIndex & 0x80000000) != 0 ? nodes[i].shapeIndex : nodes[i].height);
		vend();
	}

	let materials = b3GetCompoundMaterials(c)!;
	for (let i = 0; i < c.materialCount; ++i) {
		vtag("mat");
		pmat(materials[i]);
		vend();
	}

	for (let i = 0; i < c.capsuleCount; ++i) {
		let cc = b3GetCompoundCapsule(c, i);
		vtag("cap");
		pv3(cc.capsule.center1);
		pv3(cc.capsule.center2);
		vf32(cc.capsule.radius);
		vu32(cc.materialIndex);
		vu32(cc.padding);
		vend();
	}

	for (let i = 0; i < c.sharedHullCount; ++i) {
		vtag("shull");
		phull(c.hulls[i]);
		vend();
	}

	for (let i = 0; i < c.hullCount; ++i) {
		let ch = b3GetCompoundHull(c, i);
		let shared = -1;
		for (let j = 0; j < c.sharedHullCount; ++j) {
			if (c.hulls[j] == ch.hull) {
				shared = j;
			}
		}
		vtag("hull");
		pxf(ch.transform);
		vi32(shared);
		vu32(ch.materialIndex);
		vu32(ch.padding);
		vend();
	}

	let meshMaterialTotal = 0;
	for (let i = 0; i < c.meshCount; ++i) {
		meshMaterialTotal += b3GetCompoundMesh(c, i).materialCount;
	}
	let meshMaterialMapOffset = c.meshOffset + align8(c.meshCount * 52);

	{
		let offset = meshMaterialMapOffset + align8(meshMaterialTotal * 2);
		for (let i = 0; i < c.sharedMeshCount; ++i) {
			vtag("smesh");
			pmesh(c.meshes[i]);
			vend();
			offset += align8(c.meshes[i].byteCount);
		}
		if (c.sharedMeshCount > 0) {
			vtag("sphereoff");
			vbool(offset == c.sphereOffset);
			vend();
		}
	}

	let mmap = c.meshMaterialIndices;
	vtag("mmap");
	vi32(meshMaterialTotal);
	for (let i = 0; i < meshMaterialTotal; ++i) {
		vu32(<u32>mmap[i]);
	}
	vend();

	for (let i = 0; i < c.meshCount; ++i) {
		let cm = b3GetCompoundMesh(c, i);
		let shared = -1;
		for (let j = 0; j < c.sharedMeshCount; ++j) {
			if (c.meshes[j] == cm.meshData) {
				shared = j;
			}
		}
		vtag("mesh");
		pxf(cm.transform);
		pv3(cm.scale);
		vi32(shared);
		vi32(<i32>c.meshInstances[i].materialOffset);
		vi32(cm.materialCount);
		vend();
	}

	for (let i = 0; i < c.sphereCount; ++i) {
		let cs = b3GetCompoundSphere(c, i);
		vtag("sph");
		pv3(cs.sphere.center);
		vf32(cs.sphere.radius);
		vu32(cs.materialIndex);
		vu32(cs.padding);
		vend();
	}

	let childCount = c.capsuleCount + c.hullCount + c.meshCount + c.sphereCount;
	for (let i = 0; i < childCount; ++i) {
		printChild(c, i);
	}
}

function randMaterial(): b3SurfaceMaterial {
	let index = vrandint(0, MATERIAL_COUNT);
	return g_materials[index];
}

class QueryContext {
	stopAfter: i32 = 0;
	count: i32 = 0;
}

// Proxies in a given buffer: a point, a segment, or the corners of an oriented box
function randProxy(buffer: StaticArray<b3Vec3>, center: b3Vec3): b3ShapeProxy {
	let kind = vrandint(0, 3);
	let proxy = new b3ShapeProxy(buffer, 0, 0.0);
	if (kind == 0) {
		buffer[0] = center;
		proxy.count = 1;
		proxy.radius = vrandf(0.05, 0.8);
	} else if (kind == 1) {
		buffer[0] = center;
		let d = randVec(-1.0, 1.0);
		buffer[1] = b3Add(center, d);
		proxy.count = 2;
		proxy.radius = vrandf(0.0, 0.5);
	} else {
		let h = randVec(0.05, 0.7);
		let q = randQuat();
		for (let i = 0; i < 8; ++i) {
			let corner = b3Vec3((i & 1) != 0 ? h.x : -h.x, (i & 2) != 0 ? h.y : -h.y, (i & 4) != 0 ? h.z : -h.z);
			buffer[i] = b3Add(center, b3RotateVector(q, corner));
		}
		proxy.count = 8;
		let round = vrandint(0, 2);
		proxy.radius = round == 1 ? vrandf(0.0, 0.2) : <f32>0.0;
	}
	return proxy;
}

// A point on or inside a child, in the compound frame
function childPoint(compound: b3CompoundData, childIndex: i32): b3Vec3 {
	let child = b3GetCompoundChild(compound, childIndex);
	if (child.type == b3_capsuleShape) {
		return child.capsule.center1;
	}
	if (child.type == b3_sphereShape) {
		return child.sphere.center;
	}
	return child.transform.p;
}

function randTarget(compound: b3CompoundData, childCount: i32, bounds: b3AABB): b3Vec3 {
	let aimed = vrandint(0, 2);
	if (aimed == 0) {
		return randInBox(bounds, 0.0);
	}
	let childIndex = vrandint(0, childCount);
	let p = childPoint(compound, childIndex);
	let jitter = randVec(-0.3, 0.3);
	return b3Add(p, jitter);
}

const forcedHulls: StaticArray<i32> = [0, 1, 8, 9];

export function runCompound(index: i32): void {
	vseed(<u64>(5000 + index));

	let def = new b3CompoundDef();
	let big = index % 10 == 9;
	def.capsuleCount = big ? 40 : vrandint(0, 6);
	def.hullCount = big ? 30 : vrandint(0, 6);
	def.meshCount = big ? 4 : vrandint(0, 3);
	def.sphereCount = big ? 60 : vrandint(0, 6);
	if (def.capsuleCount + def.hullCount + def.meshCount + def.sphereCount == 0) {
		def.sphereCount = 1;
	}

	// The first compound shares hulls and meshes by pointer and by identical content
	if (index == 0) {
		def.hullCount = 4;
		def.meshCount = 2;
	}
	let capsules = new StaticArray<b3CompoundCapsuleDef>(def.capsuleCount);
	let hulls = new Array<b3CompoundHullDef>();
	let meshes = new Array<b3CompoundMeshDef>();
	let spheres = new StaticArray<b3CompoundSphereDef>(def.sphereCount);

	let range: f32 = big ? 10.0 : 4.0;
	for (let i = 0; i < def.capsuleCount; ++i) {
		let c1 = randVec(-range, range);
		let d = randVec(-1.0, 1.0);
		capsules[i].capsule.center1 = c1;
		capsules[i].capsule.center2 = b3Add(c1, d);
		capsules[i].capsule.radius = vrandf(0.05, 0.5);
		capsules[i].material = randMaterial();
	}

	for (let i = 0; i < def.hullCount; ++i) {
		let h = vrandint(0, HULL_COUNT);
		if (index == 0) {
			h = forcedHulls[i];
		}
		let transform = randTransform(range);
		let material = randMaterial();
		hulls.push(new b3CompoundHullDef(g_hulls[h], transform, material));
	}

	for (let i = 0; i < def.meshCount; ++i) {
		let m = vrandint(0, MESH_COUNT);
		if (index == 0) {
			m = i;
		}
		let meshDef = new b3CompoundMeshDef(g_meshes[m]);
		meshDef.transform = randTransform(range);
		let scale = b3Vec3();
		scale.x = vrandf(0.5, 2.0);
		scale.y = vrandf(0.5, 2.0);
		scale.z = vrandf(0.5, 2.0);
		let flip = vrandint(0, 4);
		if (flip == 0) {
			scale.y = -scale.y;
		}
		meshDef.scale = scale;
		meshDef.materialCount = g_meshes[m].materialCount;
		meshDef.materials = new StaticArray<b3SurfaceMaterial>(meshDef.materialCount);
		for (let j = 0; j < meshDef.materialCount; ++j) {
			meshDef.materials[j] = randMaterial();
		}
		meshes.push(meshDef);
	}

	for (let i = 0; i < def.sphereCount; ++i) {
		spheres[i].sphere.center = randVec(-range, range);
		spheres[i].sphere.radius = vrandf(0.05, 0.6);
		spheres[i].material = randMaterial();
	}

	def.capsules = capsules;
	def.hulls = StaticArray.fromArray<b3CompoundHullDef>(hulls);
	def.meshes = StaticArray.fromArray<b3CompoundMeshDef>(meshes);
	def.spheres = spheres;

	let compoundOrNull = b3CreateCompound(def);
	vtag("build");
	vi32(index);
	vend();
	printCompound(compoundOrNull);
	if (compoundOrNull == null) {
		return;
	}
	let compound = compoundOrNull;

	let bounds = compound.bounds;
	let childCount = def.capsuleCount + def.hullCount + def.meshCount + def.sphereCount;

	for (let i = 0; i < 4; ++i) {
		let xf = randTransform(5.0);
		vtag("aabb");
		pab(b3ComputeCompoundAABB(compound, xf));
		vend();
	}

	for (let i = 0; i < 12; ++i) {
		let center = randInBox(bounds, 1.0);
		let h = randVec(0.1, 3.0);
		let box = b3AABB(b3Sub(center, h), b3Add(center, h));
		let qc = new QueryContext();
		qc.stopAfter = vrandint(0, 6);
		b3QueryCompound(compound, box, (c: b3CompoundData, childIndex: i32): bool => {
			qc.count += 1;
			vtag("qhit");
			vi32(childIndex);
			vend();
			return qc.stopAfter == 0 || qc.count < qc.stopAfter;
		});
		vtag("query");
		vi32(qc.count);
		vend();
	}

	for (let i = 0; i < 40; ++i) {
		let input = b3RayCastInput(b3Vec3_zero, b3Vec3_zero, 0.0);
		input.origin = randInBox(bounds, 3.0);
		let target = randTarget(compound, childCount, bounds);
		let stretch = vrandf(0.5, 2.0);
		input.translation = b3MulSV(stretch, b3Sub(target, input.origin));
		let partial = vrandint(0, 3);
		input.maxFraction = partial == 0 ? vrandf(0.1, 1.0) : <f32>1.0;
		let output = b3RayCastCompound(compound, input);
		vtag("ray");
		pcast(output);
		vend();
	}

	for (let i = 0; i < 30; ++i) {
		let buffer = new StaticArray<b3Vec3>(8);
		let center = randInBox(bounds, 2.0);
		let input = new b3ShapeCastInput();
		input.proxy = randProxy(buffer, center);
		let target = randTarget(compound, childCount, bounds);
		input.translation = b3Sub(target, center);
		let partial = vrandint(0, 3);
		input.maxFraction = partial == 0 ? vrandf(0.1, 1.0) : <f32>1.0;
		let encroach = vrandint(0, 2);
		input.canEncroach = encroach == 1;
		let output = b3ShapeCastCompound(compound, input);
		vtag("cast");
		pcast(output);
		vend();
	}

	{
		let empty = new b3ShapeCastInput();
		empty.translation = b3Vec3(1.0, 0.0, 0.0);
		empty.maxFraction = 1.0;
		vtag("castempty");
		pcast(b3ShapeCastCompound(compound, empty));
		vend();
	}

	for (let i = 0; i < 30; ++i) {
		let buffer = new StaticArray<b3Vec3>(8);
		let xf = randTransform(2.0);
		let local = randTarget(compound, childCount, bounds);
		let jitter = randVec(-0.7, 0.7);
		let center = b3Add(b3TransformPoint(xf, local), jitter);
		let proxy = randProxy(buffer, center);
		let overlap = b3OverlapCompound(compound, xf, proxy);
		vtag("overlap");
		vbool(overlap);
		vend();
	}

	for (let i = 0; i < 30; ++i) {
		let mover = b3Capsule();
		mover.center1 = randTarget(compound, childCount, bounds);
		let d = randVec(-1.0, 1.0);
		mover.center2 = b3Add(mover.center1, d);
		mover.radius = vrandf(0.05, 1.0);
		let capacity = vrandint(1, 17);
		let planes = new StaticArray<b3PlaneResult>(16);
		let count = b3CollideMoverAndCompound(planes, capacity, compound, mover);
		vtag("mover");
		vi32(capacity);
		vi32(count);
		vend();
		for (let j = 0; j < count; ++j) {
			vtag("plane");
			pv3(planes[j].plane.normal);
			vf32(planes[j].plane.offset);
			pv3(planes[j].point);
			vi32(planes[j].triangleIndex);
			vi32(planes[j].childIndex);
			vi32(planes[j].materialIndex);
			vend();
		}
	}

	for (let i = 0; i < 20; ++i) {
		let xf = randTransform(3.0);
		let childIndex = vrandint(0, childCount);
		let child = b3GetCompoundChild(compound, childIndex);
		let sweepA = b3MakeCompoundChildSweep(xf, child.transform);
		vtag("sweep");
		vi32(childIndex);
		pv3(sweepA.localCenter);
		pv3(sweepA.c1);
		pv3(sweepA.c2);
		pq(sweepA.q1);
		pq(sweepA.q2);
		vend();

		let input = new b3TOIInput();
		if (child.type == b3_capsuleShape) {
			input.proxyA = b3MakeSegmentProxy(child.capsule.center1, child.capsule.center2, child.capsule.radius);
		} else if (child.type == b3_hullShape) {
			input.proxyA = new b3ShapeProxy(child.hull!.points, child.hull!.vertexCount, 0.0);
		} else if (child.type == b3_sphereShape) {
			input.proxyA = b3MakePointProxy(child.sphere.center, child.sphere.radius);
		} else {
			continue;
		}

		let buffer = new StaticArray<b3Vec3>(8);
		input.proxyB = randProxy(buffer, b3Vec3_zero);
		input.sweepA = sweepA;
		input.sweepB.localCenter = randVec(-0.2, 0.2);
		let near = b3TransformPoint(xf, child.transform.p);
		let offset = randVec(-3.0, 3.0);
		input.sweepB.c1 = b3Add(near, offset);
		let miss = randVec(-0.5, 0.5);
		input.sweepB.c2 = b3Add(b3Sub(near, offset), miss);
		input.sweepB.q1 = randQuat();
		input.sweepB.q2 = randQuat();
		input.maxFraction = 1.0;
		let output = b3TimeOfImpact(input);
		vtag("toi");
		vi32(output.state);
		pv3(output.point);
		pv3(output.normal);
		vf32(output.fraction);
		vf32(output.distance);
		vend();
	}

	b3DestroyCompound(compound);
}

export function tooMany(): void {
	let def = new b3CompoundDef();
	def.sphereCount = B3_MAX_CHILD_SHAPES;
	def.spheres = new StaticArray<b3CompoundSphereDef>(def.sphereCount);
	let compound = b3CreateCompound(def);
	vtag("toomany");
	vbool(compound != null);
	vend();
}
