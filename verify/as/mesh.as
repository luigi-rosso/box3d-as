// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/mesh.as, mirrors verify/cref/mesh.c

import { vtag, vi32, vu32, vbool, vf32, vu64, vend, vseed, vrandint, vrandf } from "./common";
import { B3_MAX_SHAPE_CAST_POINTS } from "../../lib/constants";
import {
	b3AABB,
	b3Add,
	b3Lerp,
	b3Max,
	b3Min,
	b3Mul,
	b3MulSV,
	b3NormalizeQuat,
	b3Quat,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Vec3,
} from "../../lib/math_functions";
import { b3Capsule, b3CastOutput, b3Mesh, b3MeshData, b3MeshDef, b3PlaneResult, b3RayCastInput, b3ShapeCastInput, b3ShapeProxy } from "../../lib/collision";
import {
	b3CollideMoverAndMesh,
	b3ComputeMeshAABB,
	b3CreateBoxMesh,
	b3CreateGridMesh,
	b3CreateHollowBoxMesh,
	b3CreateMesh,
	b3CreatePlatformMesh,
	b3CreateTorusMesh,
	b3CreateWaveMesh,
	b3GetHeight,
	b3GetMeshNodeBits,
	b3IsValidMesh,
	b3OverlapMesh,
	b3QueryMesh,
	b3RayCastMesh,
	b3ShapeCastMesh,
} from "../../lib/mesh";
import { b3GetMeshTriangle } from "../../lib/shape";

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

function randTransform(range: f32): b3Transform {
	let t = b3Transform();
	t.p = randVec(-range, range);
	let q = b3Quat();
	q.v = randVec(-1.0, 1.0);
	q.s = vrandf(-1.0, 1.0);
	t.q = b3NormalizeQuat(q);
	return t;
}

function printMesh(mesh: b3MeshData?): void {
	vtag("mesh");
	vbool(mesh != null);
	vbool(b3IsValidMesh(mesh));
	vend();
	if (mesh == null) {
		return;
	}

	vtag("head");
	vu64(mesh.version);
	vu64(mesh.hash);
	vi32(mesh.byteCount);
	pab(mesh.bounds);
	vf32(mesh.surfaceArea);
	vi32(mesh.treeHeight);
	vi32(mesh.degenerateCount);
	vi32(mesh.nodeOffset);
	vi32(mesh.nodeCount);
	vi32(mesh.vertexOffset);
	vi32(mesh.vertexCount);
	vi32(mesh.triangleOffset);
	vi32(mesh.triangleCount);
	vi32(mesh.materialOffset);
	vi32(mesh.materialCount);
	vi32(mesh.flagsOffset);
	vi32(mesh.padding);
	vi32(b3GetHeight(mesh));
	vend();

	let nodes = mesh.nodes;
	for (let i = 0; i < mesh.nodeCount; ++i) {
		vtag("n");
		pv3(nodes[i].lowerBound);
		vu32(b3GetMeshNodeBits(nodes[i]));
		pv3(nodes[i].upperBound);
		vu32(nodes[i].triangleOffset);
		vend();
	}

	let vertices = mesh.vertices;
	for (let i = 0; i < mesh.vertexCount; ++i) {
		vtag("v");
		pv3(vertices[i]);
		vend();
	}

	let triangles = mesh.triangles;
	let materials = mesh.materialIndices;
	let flags = mesh.flags;
	for (let i = 0; i < mesh.triangleCount; ++i) {
		vtag("t");
		vi32(triangles[i].index1);
		vi32(triangles[i].index2);
		vi32(triangles[i].index3);
		vu32(materials[i]);
		vu32(flags[i]);
		vend();
	}
}

const MAX_MESHES = 128;
let g_meshes = new Array<b3MeshData>();

function addMesh(mesh: b3MeshData?): void {
	printMesh(mesh);
	if (mesh != null && g_meshes.length < MAX_MESHES) {
		g_meshes.push(mesh);
	}
}

// A seeded soup: random vertices, optional exact and near duplicates, random and degenerate triangles.
function soup(seed: i32): void {
	vseed(1000 + seed);

	let baseCount = vrandint(3, 120);
	let dupCount = vrandint(0, 40);
	let vertexCount = baseCount + dupCount;
	let extent = vrandf(0.5, 20.0);
	let jitter = vrandf(0.0, 0.02);

	let vertices = new StaticArray<b3Vec3>(vertexCount);
	for (let i = 0; i < baseCount; ++i) {
		vertices[i] = randVec(-extent, extent);
	}

	for (let i = 0; i < dupCount; ++i) {
		let src = vrandint(0, baseCount);
		let v = vertices[src];
		let mode = vrandint(0, 3);
		if (mode == 1) {
			v.x += vrandf(-jitter, jitter);
			v.y += vrandf(-jitter, jitter);
			v.z += vrandf(-jitter, jitter);
		} else if (mode == 2) {
			// collinear with two others
			let other = vrandint(0, baseCount);
			let t = vrandf(0.0, 1.0);
			v = b3Lerp(vertices[src], vertices[other], t);
		}
		vertices[baseCount + i] = v;
	}

	let triangleCount = vrandint(1, 260);
	let indices = new StaticArray<i32>(3 * triangleCount);
	let materials = new StaticArray<u8>(triangleCount);
	for (let i = 0; i < triangleCount; ++i) {
		let kind = vrandint(0, 10);
		let a = vrandint(0, vertexCount);
		let b = vrandint(0, vertexCount);
		let c = vrandint(0, vertexCount);
		if (kind == 0) {
			b = a;
		} else if (kind == 1) {
			c = b;
		}
		indices[3 * i + 0] = a;
		indices[3 * i + 1] = b;
		indices[3 * i + 2] = c;
		materials[i] = <u8>vrandint(0, 7);
	}

	let def = new b3MeshDef();
	def.vertices = vertices;
	def.vertexCount = vertexCount;
	def.indices = indices;
	def.triangleCount = triangleCount;
	let useMaterials = vrandint(0, 2);
	def.materialIndices = useMaterials != 0 ? materials : null;
	def.weldVertices = vrandint(0, 2) == 1;
	def.weldTolerance = vrandf(0.0, 0.05);
	def.useMedianSplit = vrandint(0, 2) == 1;
	def.identifyEdges = vrandint(0, 2) == 1;
	def.clockWiseWinding = vrandint(0, 2) == 1;

	let degenerateCapacity = vrandint(0, 8);
	let degenerates = new StaticArray<i32>(8);

	let mesh = b3CreateMesh(def, degenerateCapacity > 0 ? degenerates : null, degenerateCapacity);
	vtag("soup");
	vi32(seed);
	for (let i = 0; i < 8; ++i) {
		vi32(degenerates[i]);
	}
	vend();
	addMesh(mesh);
}

// A grid where every triangle has its own vertices, welded back together, plus noise in height.
function weldedGrid(seed: i32): void {
	vseed(5000 + seed);
	let nx = vrandint(1, 14);
	let nz = vrandint(1, 14);
	let cell = vrandf(0.2, 2.0);
	let noise = vrandf(0.0, 0.5);

	let triangleCount = 2 * nx * nz;
	let vertices = new StaticArray<b3Vec3>(3 * triangleCount);
	let indices = new StaticArray<i32>(3 * triangleCount);
	let heights = new StaticArray<f32>((nx + 1) * (nz + 1));
	for (let i = 0; i < (nx + 1) * (nz + 1); ++i) {
		heights[i] = vrandf(-noise, noise);
	}

	let k = 0;
	for (let ix = 0; ix < nx; ++ix) {
		for (let iz = 0; iz < nz; ++iz) {
			let p11 = b3Vec3(cell * <f32>ix, heights[ix * (nz + 1) + iz], cell * <f32>iz);
			let p12 = b3Vec3(cell * <f32>ix, heights[ix * (nz + 1) + iz + 1], cell * <f32>(iz + 1));
			let p21 = b3Vec3(cell * <f32>(ix + 1), heights[(ix + 1) * (nz + 1) + iz], cell * <f32>iz);
			let p22 = b3Vec3(cell * <f32>(ix + 1), heights[(ix + 1) * (nz + 1) + iz + 1], cell * <f32>(iz + 1));
			vertices[k + 0] = p11;
			vertices[k + 1] = p12;
			vertices[k + 2] = p22;
			vertices[k + 3] = p22;
			vertices[k + 4] = p21;
			vertices[k + 5] = p11;
			for (let j = 0; j < 6; ++j) {
				indices[k + j] = k + j;
			}
			k += 6;
		}
	}

	let def = new b3MeshDef();
	def.vertices = vertices;
	def.vertexCount = 3 * triangleCount;
	def.indices = indices;
	def.triangleCount = triangleCount;
	def.weldVertices = true;
	def.weldTolerance = vrandf(0.001, 0.1);
	def.useMedianSplit = vrandint(0, 2) == 1;
	def.identifyEdges = true;
	def.clockWiseWinding = vrandint(0, 2) == 1;

	vtag("welded");
	vi32(seed);
	vend();
	addMesh(b3CreateMesh(def, null, 0));
}

function failures(): void {
	let vertices = new StaticArray<b3Vec3>(4);
	vertices[0] = b3Vec3(0, 0, 0);
	vertices[1] = b3Vec3(1, 0, 0);
	vertices[2] = b3Vec3(0, 0, 1);
	vertices[3] = b3Vec3(1, 0, 1);
	let indices: StaticArray<i32> = [0, 1, 2, 2, 1, 3];

	let def = new b3MeshDef();
	def.vertices = vertices;
	def.vertexCount = 4;
	def.indices = indices;
	def.triangleCount = 2;

	let strides: StaticArray<i32> = [0, 12, 5, 8, 14, 5000];
	for (let i = 0; i < 6; ++i) {
		def.stride = strides[i];
		let mesh = b3CreateMesh(def, null, 0);
		vtag("stride");
		vi32(strides[i]);
		vend();
		printMesh(mesh);
	}
	def.stride = 0;

	def.vertexCount = 2;
	printMesh(b3CreateMesh(def, null, 0));
	def.vertexCount = 4;
	def.triangleCount = 0;
	printMesh(b3CreateMesh(def, null, 0));

	// all degenerate
	let degenerateIndices: StaticArray<i32> = [0, 0, 1, 2, 2, 3];
	def.indices = degenerateIndices;
	def.triangleCount = 2;
	printMesh(b3CreateMesh(def, null, 0));
}

export function buildAll(): i32 {
	addMesh(b3CreateGridMesh(4, 4, 1.0, 0, false));
	addMesh(b3CreateGridMesh(4, 4, 1.0, 0, true));
	addMesh(b3CreateGridMesh(10, 7, 0.5, 3, true));
	addMesh(b3CreateGridMesh(1, 1, 2.0, 1, true));
	addMesh(b3CreateGridMesh(16, 16, 1.0, 0, true));
	addMesh(b3CreateGridMesh(33, 20, 0.25, 5, false));
	addMesh(b3CreateWaveMesh(10, 10, 1.0, 0.5, 0.2, 0.3));
	addMesh(b3CreateWaveMesh(20, 13, 0.7, 2.0, 0.1, 0.05));
	addMesh(b3CreateWaveMesh(32, 32, 0.5, 1.5, 0.17, 0.23));
	addMesh(b3CreateTorusMesh(8, 6, 2.0, 0.5));
	addMesh(b3CreateTorusMesh(16, 12, 3.0, 1.0));
	addMesh(b3CreateTorusMesh(32, 16, 5.0, 1.5));
	addMesh(b3CreateTorusMesh(3, 3, 1.0, 0.3));
	addMesh(b3CreateBoxMesh(b3Vec3(0, 0, 0), b3Vec3(1, 1, 1), true));
	addMesh(b3CreateBoxMesh(b3Vec3(0, 0, 0), b3Vec3(1, 1, 1), false));
	addMesh(b3CreateBoxMesh(b3Vec3(1, -2, 3), b3Vec3(0.5, 2, 0.1), true));
	addMesh(b3CreateHollowBoxMesh(b3Vec3(0, 0, 0), b3Vec3(2, 1, 3)));
	addMesh(b3CreateHollowBoxMesh(b3Vec3(-1, 0.5, 2), b3Vec3(0.3, 0.3, 0.3)));
	addMesh(b3CreatePlatformMesh(b3Vec3(0, 0, 0), 1.0, 2.0, 4.0));
	addMesh(b3CreatePlatformMesh(b3Vec3(3, 1, -2), 0.5, 6.0, 6.5));

	for (let i = 0; i < 40; ++i) {
		soup(i);
	}

	for (let i = 0; i < 12; ++i) {
		weldedGrid(i);
	}

	failures();

	vtag("built");
	vi32(g_meshes.length);
	vend();
	return g_meshes.length;
}

const g_scales: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(6);

function initScales(): void {
	g_scales[0] = b3Vec3(1, 1, 1);
	g_scales[1] = b3Vec3(2, 0.5, 1.5);
	g_scales[2] = b3Vec3(-1, 1, 1);
	g_scales[3] = b3Vec3(1, -2, -0.5);
	g_scales[4] = b3Vec3(-1, -1, -1);
	g_scales[5] = b3Vec3(0.3, 3, -0.7);
}

let g_queryLimit = 0;
let g_queryCount = 0;

function queryCallback(a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32): bool {
	vtag("q");
	vi32(triangleIndex);
	pv3(a);
	pv3(b);
	pv3(c);
	vend();
	g_queryCount += 1;
	return g_queryLimit < 0 || g_queryCount < g_queryLimit;
}

function centroid(mesh: b3Mesh, ti: i32): b3Vec3 {
	let t = b3GetMeshTriangle(mesh, ti);
	return b3MulSV(<f32>1.0 / <f32>3.0, b3Add(b3Add(t.vertices[0], t.vertices[1]), t.vertices[2]));
}

function scaledBounds(mesh: b3Mesh): b3AABB {
	let s = mesh.scale;
	let l = b3Mul(s, mesh.data.bounds.lowerBound);
	let u = b3Mul(s, mesh.data.bounds.upperBound);
	return b3AABB(b3Min(l, u), b3Max(l, u));
}

// Everything that does not need GJK
export function queries(meshIndex: i32, scaleIndex: i32): void {
	initScales();
	let data = g_meshes[meshIndex];
	let mesh = new b3Mesh(data, g_scales[scaleIndex]);
	vseed(<u64>(20000 + 16 * meshIndex + scaleIndex));

	vtag("queries");
	vi32(meshIndex);
	vi32(scaleIndex);
	vend();

	for (let i = 0; i < data.triangleCount; ++i) {
		let t = b3GetMeshTriangle(mesh, i);
		vtag("tri");
		pv3(t.vertices[0]);
		pv3(t.vertices[1]);
		pv3(t.vertices[2]);
		vi32(t.i1);
		vi32(t.i2);
		vi32(t.i3);
		vi32(t.flags);
		vend();
	}

	for (let i = 0; i < 4; ++i) {
		let xf = randTransform(10.0);
		vtag("aabb");
		pab(b3ComputeMeshAABB(data, xf, mesh.scale));
		vend();
	}

	let box = scaledBounds(mesh);
	let size = b3Sub(box.upperBound, box.lowerBound);
	let pad: f32 = 0.25 * (size.x + size.y + size.z) + 0.5;

	for (let i = 0; i < 40; ++i) {
		let input = b3RayCastInput();
		let kind = vrandint(0, 3);
		if (kind == 0) {
			input.origin = randInBox(box, pad);
			input.translation = randVec(-3.0 * pad, 3.0 * pad);
		} else {
			// aim at a triangle centroid
			let ti = vrandint(0, data.triangleCount);
			let c = centroid(mesh, ti);
			input.origin = b3Add(c, randVec(-pad, pad));
			input.translation = b3MulSV(vrandf(1.0, 2.5), b3Sub(c, input.origin));
		}
		input.maxFraction = vrandint(0, 3) == 0 ? vrandf(0.1, 1.0) : 1.0;

		vtag("ray");
		pcast(b3RayCastMesh(mesh, input));
		vend();
	}

	for (let i = 0; i < 12; ++i) {
		let c = randInBox(box, 0.2 * pad);
		let e = randVec(0.0, 0.4 * pad);
		let bounds = b3AABB(b3Sub(c, e), b3Add(c, e));
		g_queryLimit = vrandint(0, 4) == 0 ? vrandint(1, 6) : -1;
		g_queryCount = 0;
		vtag("query");
		pab(bounds);
		vi32(g_queryLimit);
		vend();
		b3QueryMesh(mesh, bounds, queryCallback);
		vtag("queryEnd");
		vi32(g_queryCount);
		vend();
	}
}

function randProxy(points: StaticArray<b3Vec3>, scale: f32): b3ShapeProxy {
	let kind = vrandint(0, 4);
	let count = kind == 0 ? 1 : kind == 1 ? 2 : vrandint(3, 9);
	for (let i = 0; i < count; ++i) {
		points[i] = randVec(-scale, scale);
	}
	let radius: f32 = kind == 3 ? 0.0 : vrandf(0.0, scale);
	return new b3ShapeProxy(points, count, radius);
}

// Overlap, shape cast and mover, which need GJK
export function casts(meshIndex: i32, scaleIndex: i32): void {
	initScales();
	let data = g_meshes[meshIndex];
	let mesh = new b3Mesh(data, g_scales[scaleIndex]);
	vseed(<u64>(40000 + 16 * meshIndex + scaleIndex));

	vtag("casts");
	vi32(meshIndex);
	vi32(scaleIndex);
	vend();

	let box = scaledBounds(mesh);
	let size = b3Sub(box.upperBound, box.lowerBound);
	let pad: f32 = 0.25 * (size.x + size.y + size.z) + 0.5;
	let shapeScale: f32 = 0.05 * (size.x + size.y + size.z) + 0.1;

	let points = new StaticArray<b3Vec3>(B3_MAX_SHAPE_CAST_POINTS);

	for (let i = 0; i < 12; ++i) {
		let proxy = randProxy(points, shapeScale);
		let xf = randTransform(pad);
		let p = randInBox(box, 0.3 * pad);
		// place the proxy in mesh space through the inverse of xf
		for (let k = 0; k < proxy.count; ++k) {
			points[k] = b3TransformPoint(xf, b3Add(points[k], p));
		}
		vtag("overlap");
		vbool(b3OverlapMesh(mesh, xf, proxy));
		vend();
	}

	for (let i = 0; i < 16; ++i) {
		let input = new b3ShapeCastInput();
		input.proxy = randProxy(points, shapeScale);
		let origin: b3Vec3;
		let target: b3Vec3;
		if (vrandint(0, 2) == 0) {
			origin = randInBox(box, pad);
			target = randInBox(box, 0.0);
		} else {
			let ti = vrandint(0, data.triangleCount);
			target = centroid(mesh, ti);
			origin = b3Add(target, randVec(-pad, pad));
		}
		for (let k = 0; k < input.proxy.count; ++k) {
			points[k] = b3Add(points[k], origin);
		}
		input.translation = b3MulSV(vrandf(0.5, 2.0), b3Sub(target, origin));
		input.maxFraction = vrandint(0, 3) == 0 ? vrandf(0.2, 1.0) : 1.0;
		input.canEncroach = vrandint(0, 2) == 1;

		vtag("cast");
		pcast(b3ShapeCastMesh(mesh, input));
		vend();
	}

	let planes = new StaticArray<b3PlaneResult>(16);
	for (let i = 0; i < 16; ++i) {
		let mover = b3Capsule();
		mover.center1 = randInBox(box, 0.2 * pad);
		mover.center2 = b3Add(mover.center1, randVec(-shapeScale, shapeScale));
		mover.radius = vrandf(0.05, 2.0 * shapeScale);
		let offset = vrandint(0, 4);
		let capacity = vrandint(0, 12);
		for (let k = 0; k < 16; ++k) {
			planes[k] = b3PlaneResult();
		}
		let count = b3CollideMoverAndMesh(planes, offset, capacity, mesh, mover);
		vtag("mover");
		vi32(offset);
		vi32(capacity);
		vi32(count);
		vend();
		for (let k = 0; k < count; ++k) {
			let r = planes[offset + k];
			vtag("plane");
			pv3(r.plane.normal);
			vf32(r.plane.offset);
			pv3(r.point);
			vi32(r.triangleIndex);
			vi32(r.childIndex);
			vi32(r.materialIndex);
			vend();
		}
	}
}
