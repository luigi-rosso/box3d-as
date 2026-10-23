// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/hull.as, mirrors verify/cref/hull.c phase by phase.

import { vtag, vi32, vu32, vbool, vf32, vu64, vend, vseed, vrandint, vrandf, vspecialf, vbitsf } from "./common";
import { b3SetLogFcn } from "../../lib/core";
import { HULL_SET_COUNT, hullPointBits, hullSets } from "./hull_data";
import {
	b3AABB,
	b3Add,
	b3Clamp,
	b3Matrix3,
	b3Mul,
	b3MulSV,
	b3Neg,
	b3Normalize,
	b3NormalizeQuat,
	b3Plane,
	b3Quat,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Vec3,
	b3Vec3_axisX,
	b3Vec3_axisY,
	b3Vec3_axisZ,
	b3Vec3_one,
	b3Vec3_zero,
} from "../../lib/math_functions";
import {
	b3Capsule,
	b3CastOutput,
	b3HullData,
	b3PlaneResult,
	b3Point2D,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeProxy,
} from "../../lib/collision";
import {
	b3CloneAndTransformHull,
	b3CloneHull,
	b3CollideMoverAndHull,
	b3CompareHullData,
	b3ComputeHullAABB,
	b3ComputeHullExtent,
	b3ComputeHullMass,
	b3ComputeHullProjectedArea,
	b3ComputeSweptHullAABB,
	b3CreateComplexHull,
	b3CreateCone,
	b3CreateCylinder,
	b3CreateHull,
	b3CreateRock,
	b3FindHullSupportFace,
	b3FindHullSupportVertex,
	b3GetHullSoaStride,
	b3HashHullData,
	b3Hull2D,
	b3HullMap,
	b3HullMap_cleanup,
	b3HullMap_erase_itr,
	b3HullMap_first,
	b3HullMap_get,
	b3HullMap_get_or_insert,
	b3HullMap_init,
	b3HullMap_insert,
	b3HullMap_is_end,
	b3HullMap_key,
	b3HullMap_next,
	b3HullMap_reserve,
	b3HullMap_setVal,
	b3HullMap_size,
	b3HullMap_val,
	b3IsValidHull,
	b3MakeBoxHull,
	b3MakeCubeHull,
	b3MakeOffsetBoxHull,
	b3MakeScaledBoxHull,
	b3MakeTransformedBoxHull,
	b3OverlapHull,
	b3RayCastHull,
	b3ScaleBox,
	b3SerializeHullData,
	b3ShapeCastHull,
	b3SimplifyHull2D,
	b3ValidateHull,
} from "../../lib/hull";

export { frameBoundary } from "./common";

// Log lines become vectors, so the hull limit messages are compared too.
function logToVectors(message: string): void {
	vtag("log");
	vtag(message);
	vend();
}

b3SetLogFcn(logToVectors);

function vvec(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function vplane(p: b3Plane): void {
	vvec(p.normal);
	vf32(p.offset);
}

function vaabb(a: b3AABB): void {
	vvec(a.lowerBound);
	vvec(a.upperBound);
}

function vmat(m: b3Matrix3): void {
	vvec(m.cx);
	vvec(m.cy);
	vvec(m.cz);
}

function vrandvec(lo: f32, hi: f32): b3Vec3 {
	let v = b3Vec3();
	v.x = vrandf(lo, hi);
	v.y = vrandf(lo, hi);
	v.z = vrandf(lo, hi);
	return v;
}

function vrandquat(): b3Quat {
	let q = b3Quat();
	q.v.x = vrandf(-1.0, 1.0);
	q.v.y = vrandf(-1.0, 1.0);
	q.v.z = vrandf(-1.0, 1.0);
	q.s = vrandf(-1.0, 1.0);
	return b3NormalizeQuat(q);
}

function vrandxf(range: f32): b3Transform {
	let xf = b3Transform();
	xf.p = vrandvec(-range, range);
	xf.q = vrandquat();
	return xf;
}

function bitsVec(pointIndex: i32): b3Vec3 {
	let v = b3Vec3();
	v.x = vbitsf(hullPointBits[3 * pointIndex + 0]);
	v.y = vbitsf(hullPointBits[3 * pointIndex + 1]);
	v.z = vbitsf(hullPointBits[3 * pointIndex + 2]);
	return v;
}

function printHull(tag: string, hull: b3HullData?): void {
	vtag(tag);
	if (hull == null) {
		vtag("null");
		vend();
		return;
	}

	vu64(hull.version);
	vu64(hull.hash);
	vaabb(hull.aabb);
	vf32(hull.surfaceArea);
	vf32(hull.volume);
	vf32(hull.innerRadius);
	vvec(hull.center);
	vmat(hull.centralInertia);
	vi32(hull.vertexCount);
	vi32(hull.vertexOffset);
	vi32(hull.pointOffset);
	vi32(hull.edgeCount);
	vi32(hull.edgeOffset);
	vi32(hull.faceCount);
	vi32(hull.planeOffset);
	vi32(hull.faceOffset);
	vi32(hull.soaVertexOffset);
	vi32(hull.soaNormalOffset);
	vi32(hull.edgeCosineOffset);
	vi32(hull.byteCount);
	vi32(hull.padding);
	vend();

	vtag("v");
	for (let i = 0; i < hull.vertexCount; ++i) {
		vi32(<i32>hull.vertices[i].edge);
	}
	vend();

	vtag("p");
	for (let i = 0; i < hull.vertexCount; ++i) {
		vvec(hull.points[i]);
	}
	vend();

	vtag("e");
	for (let i = 0; i < hull.edgeCount; ++i) {
		let e = hull.edges[i];
		vi32(<i32>e.next);
		vi32(<i32>e.twin);
		vi32(<i32>e.origin);
		vi32(<i32>e.face);
	}
	vend();

	vtag("pl");
	for (let i = 0; i < hull.faceCount; ++i) {
		vplane(hull.planes[i]);
	}
	vend();

	vtag("f");
	for (let i = 0; i < hull.faceCount; ++i) {
		vi32(<i32>hull.faces[i].edge);
	}
	vend();

	vtag("sv");
	let soaVertexCount = 3 * b3GetHullSoaStride(hull.vertexCount);
	for (let i = 0; i < soaVertexCount; ++i) {
		vf32(hull.soaVertices[i]);
	}
	vend();

	vtag("sn");
	let soaNormalCount = 3 * b3GetHullSoaStride(hull.faceCount);
	for (let i = 0; i < soaNormalCount; ++i) {
		vf32(hull.soaNormals[i]);
	}
	vend();

	vtag("ec");
	for (let i = 0; i < hull.edgeCount / 2; ++i) {
		vf32(hull.edgeCosines[i]);
	}
	vend();

	vtag("blob");
	let bytes = b3SerializeHullData(hull);
	for (let i = 0; i < hull.byteCount / 4; ++i) {
		let w =
			<u32>bytes[4 * i] |
			(<u32>bytes[4 * i + 1] << 8) |
			(<u32>bytes[4 * i + 2] << 16) |
			(<u32>bytes[4 * i + 3] << 24);
		vu32(w);
	}
	vend();

	vtag("valid");
	vbool(b3IsValidHull(hull));
	vbool(b3ValidateHull(hull));
	vu64(b3HashHullData(hull));
	vend();
}

function printCast(o: b3CastOutput): void {
	vvec(o.normal);
	vvec(o.point);
	vf32(o.fraction);
	vi32(o.iterations);
	vi32(o.triangleIndex);
	vi32(o.childIndex);
	vi32(o.materialIndex);
	vbool(o.hit);
}

function queryRay(hull: b3HullData, origin: b3Vec3, translation: b3Vec3, maxFraction: f32): void {
	let input = b3RayCastInput(origin, translation, maxFraction);
	let o = b3RayCastHull(hull, input);
	vtag("ray");
	printCast(o);
	vend();
}

function querySupport(hull: b3HullData, d: b3Vec3): void {
	vi32(b3FindHullSupportVertex(hull, d));
	vi32(b3FindHullSupportFace(hull, d));
}

function queryHull(hull: b3HullData, directionCount: i32, rayCount: i32): void {
	let lower = hull.aabb.lowerBound;
	let upper = hull.aabb.upperBound;
	let center = b3MulSV(0.5, b3Add(lower, upper));
	let extent = b3Sub(upper, lower);

	vtag("mass");
	let md = b3ComputeHullMass(hull, 1.0);
	vf32(md.mass);
	vvec(md.center);
	vmat(md.inertia);
	md = b3ComputeHullMass(hull, 0.37);
	vf32(md.mass);
	vvec(md.center);
	vmat(md.inertia);
	let density = vrandf(0.1, 10.0);
	md = b3ComputeHullMass(hull, density);
	vf32(md.mass);
	vvec(md.center);
	vmat(md.inertia);
	vend();

	vtag("aabb");
	for (let i = 0; i < 3; ++i) {
		let xf1 = vrandxf(10.0);
		let xf2 = vrandxf(10.0);
		vaabb(b3ComputeHullAABB(hull, xf1));
		vaabb(b3ComputeSweptHullAABB(hull, xf1, xf2));
	}
	vend();

	vtag("support");
	querySupport(hull, b3Vec3_axisX);
	querySupport(hull, b3Neg(b3Vec3_axisX));
	querySupport(hull, b3Vec3_axisY);
	querySupport(hull, b3Neg(b3Vec3_axisY));
	querySupport(hull, b3Vec3_axisZ);
	querySupport(hull, b3Neg(b3Vec3_axisZ));
	querySupport(hull, b3Vec3_zero);
	for (let i = 0; i < directionCount; ++i) {
		let d = vrandvec(-1.0, 1.0);
		querySupport(hull, d);
	}
	// NaN directions break the support vertex precondition, so an assert build skips them.
	if (!isDefined(HULL_NO_SPECIALS)) {
		for (let i = 0; i < 3; ++i) {
			let d = b3Vec3();
			d.x = vspecialf();
			d.y = vspecialf();
			d.z = vspecialf();
			querySupport(hull, d);
		}
	}
	vend();

	for (let i = 0; i < rayCount; ++i) {
		let o = vrandvec(-1.5, 1.5);
		let t = vrandvec(-3.0, 3.0);
		let maxFraction = vrandf(0.0, 1.5);
		queryRay(hull, b3Add(center, b3Mul(o, extent)), b3Mul(t, extent), maxFraction);
	}

	// Point queries and rays from the centroid
	queryRay(hull, center, b3Vec3_zero, 1.0);
	queryRay(hull, hull.center, b3Vec3_axisX, 1.0);
	queryRay(hull, b3Add(upper, b3Vec3_one), b3Vec3_zero, 1.0);
	queryRay(hull, b3Add(upper, b3Vec3_one), b3Neg(b3Add(extent, b3Add(b3Vec3_one, b3Vec3_one))), 1.0);

	// GJK based queries
	for (let i = 0; i < 4; ++i) {
		let proxyPoints = new StaticArray<b3Vec3>(8);
		let proxyCount = vrandint(1, 9);
		for (let j = 0; j < proxyCount; ++j) {
			let o = vrandvec(-1.0, 1.0);
			proxyPoints[j] = b3Add(center, b3Mul(o, extent));
		}
		let proxyRadius = vrandf(0.0, 0.3);
		let proxy = new b3ShapeProxy(proxyPoints, proxyCount, proxyRadius);
		let xf = vrandxf(0.5);

		vtag("overlap");
		vbool(b3OverlapHull(hull, xf, proxy));
		vend();

		let castInput = new b3ShapeCastInput();
		castInput.proxy = proxy;
		let t = vrandvec(-2.0, 2.0);
		castInput.translation = b3Mul(t, extent);
		castInput.maxFraction = vrandf(0.0, 1.0);
		castInput.canEncroach = vrandint(0, 2) == 1;
		vtag("shapecast");
		printCast(b3ShapeCastHull(hull, castInput));
		vend();

		let mover = b3Capsule();
		let m1 = vrandvec(-1.0, 1.0);
		let m2 = vrandvec(-1.0, 1.0);
		mover.center1 = b3Add(center, b3Mul(m1, extent));
		mover.center2 = b3Add(center, b3Mul(m2, extent));
		mover.radius = vrandf(0.01, 1.0);
		let result = b3PlaneResult();
		let planeCount = b3CollideMoverAndHull(result, hull, mover);
		vtag("mover");
		vi32(planeCount);
		vplane(result.plane);
		vvec(result.point);
		vi32(result.triangleIndex);
		vi32(result.childIndex);
		vi32(result.materialIndex);
		vend();
	}

	vtag("extent");
	for (let i = 0; i < 2; ++i) {
		let origin = vrandvec(-2.0, 2.0);
		let ex = b3ComputeHullExtent(hull, origin);
		vf32(ex.minExtent);
		vvec(ex.maxExtent);
	}
	vend();

	vtag("area");
	for (let i = 0; i < 6; ++i) {
		let d = vrandvec(-1.0, 1.0);
		vf32(b3ComputeHullProjectedArea(hull, d));
	}
	vf32(b3ComputeHullProjectedArea(hull, b3Vec3_axisY));
	vend();
}

function randomScale(kind: i32): b3Vec3 {
	let s: b3Vec3;
	if (kind == 0) {
		let u = vrandf(0.25, 3.0);
		s = b3Vec3(u, u, u);
	} else if (kind == 1) {
		s = vrandvec(0.2, 3.0);
	} else if (kind == 2) {
		s = vrandvec(0.2, 3.0);
		let axis = vrandint(0, 3);
		if (axis == 0) {
			s.x = -s.x;
		} else if (axis == 1) {
			s.y = -s.y;
		} else {
			s.z = -s.z;
		}
	} else {
		s = vrandvec(-3.0, -0.2);
		s.y = vrandf(0.0, 0.02);
	}
	return s;
}

function checkHull(tag: string, hull: b3HullData?): void {
	printHull(tag, hull);
	if (hull == null) {
		return;
	}

	queryHull(hull, 40, 40);

	let clone = b3CloneHull(hull);
	printHull("clone", clone);
	vtag("same");
	vbool(b3CompareHullData(clone!, hull));
	vbool(b3CompareHullData(hull, hull));
	vend();

	for (let kind = 0; kind < 4; ++kind) {
		let xf = vrandxf(5.0);
		let scale = randomScale(kind);
		let t = b3CloneAndTransformHull(hull, xf, scale);
		printHull("xf", t);
		if (t != null) {
			queryHull(t, 8, 8);
			vtag("same");
			vbool(b3CompareHullData(t, hull));
			vend();
		}
	}
}

function pickMaxCount(n: i32): i32 {
	let pick = vrandint(0, 10);
	switch (pick) {
		case 0:
			return 4;
		case 1:
			return 8;
		case 2:
			return 16;
		case 3:
			return 32;
		case 4:
			return 1;
		case 5:
			return 1000;
		case 6:
			return vrandint(4, 130);
		default:
			return n;
	}
}

let g_points = new StaticArray<b3Vec3>(4096);

export function phaseSample(set: i32): void {
	vseed(100 + set);
	let offset = hullSets[3 * set + 0];
	let count = hullSets[3 * set + 1];
	let maxCount = hullSets[3 * set + 2];
	for (let i = 0; i < count; ++i) {
		g_points[i] = bitsVec(offset + i);
	}

	vtag("sample");
	vi32(set);
	vi32(count);
	vi32(maxCount);
	vend();

	let hull = b3CreateHull(g_points, count, maxCount);
	checkHull("hull", hull);

	let limits: StaticArray<i32> = [4, 6, 12, 1000];
	for (let i = 0; i < 4; ++i) {
		let h = b3CreateHull(g_points, count, limits[i]);
		printHull("limit", h);
	}
}

export function setCount(): i32 {
	return HULL_SET_COUNT;
}

export function phaseWheel(): void {
	vseed(99);
	let buffer = new StaticArray<b3Vec3>(512);
	let bufferCount = 0;
	for (let h = 0; h < 37; ++h) {
		let set = 4 + h;
		let offset = hullSets[3 * set + 0];
		let count = hullSets[3 * set + 1];
		for (let i = 0; i < count; ++i) {
			g_points[i] = bitsVec(offset + i);
		}
		let piece = b3CreateHull(g_points, count, count)!;
		let points = piece.points;
		for (let j = 0; j < piece.vertexCount && bufferCount < 512; ++j) {
			buffer[bufferCount] = points[j];
			++bufferCount;
		}
	}

	vtag("wheel");
	vi32(bufferCount);
	vend();
	let hull = b3CreateHull(buffer, bufferCount, bufferCount);
	checkHull("hull", hull);
}

const s_skip: StaticArray<i32> = [3, 27, 267, 303, 627, 699];

export function phaseRandom(k: i32): void {
	vseed(5000 + k);
	let type = k % 12;
	let n = 0;

	if (type == 0) {
		n = vrandint(4, 400);
		for (let i = 0; i < n; ++i) {
			g_points[i] = vrandvec(-1.0, 1.0);
		}
	} else if (type == 1) {
		n = vrandint(4, 300);
		for (let i = 0; i < n; ++i) {
			let v = vrandvec(-1.0, 1.0);
			g_points[i] = b3Normalize(v);
		}
	} else if (type == 2) {
		// HullReduction box cloud
		n = vrandint(8, 256);
		let a: f32 = 0.001;
		for (let i = 0; i < n; ++i) {
			let p = vrandvec(-2.0, 2.0);
			p = b3Clamp(p, b3Vec3(-1.0, -1.0, -1.0), b3Vec3(1.0, 1.0, 1.0));
			let f = vrandvec(-a, a);
			g_points[i] = b3Add(p, f);
		}
	} else if (type == 3) {
		// Nearly coplanar
		n = vrandint(4, 100);
		let pick = vrandint(0, 5);
		let eps: f32 = pick == 0 ? <f32>0.0 : pick == 1 ? <f32>1e-7 : pick == 2 ? <f32>1e-6 : pick == 3 ? <f32>1e-5 : <f32>1e-3;
		for (let i = 0; i < n; ++i) {
			let p = b3Vec3();
			p.x = vrandf(-1.0, 1.0);
			p.y = vrandf(-1.0, 1.0);
			p.z = vrandf(-eps, eps);
			g_points[i] = p;
		}
	} else if (type == 4) {
		// Duplicates
		let m = vrandint(4, 40);
		let r = vrandint(2, 5);
		for (let i = 0; i < m; ++i) {
			let p = vrandvec(-1.0, 1.0);
			for (let j = 0; j < r; ++j) {
				g_points[n++] = p;
			}
		}
	} else if (type == 5) {
		// Nearly collinear
		n = vrandint(4, 60);
		let d = vrandvec(-1.0, 1.0);
		let eps: f32 = vrandint(0, 2) == 0 ? 0.0 : 1e-6;
		for (let i = 0; i < n; ++i) {
			let t = vrandf(-2.0, 2.0);
			let noise = vrandvec(-eps, eps);
			g_points[i] = b3Add(b3MulSV(t, d), noise);
		}
	} else if (type == 6) {
		// One point repeated
		n = vrandint(4, 20);
		let p = vrandvec(-1.0, 1.0);
		for (let i = 0; i < n; ++i) {
			g_points[i] = p;
		}
	} else if (type == 7) {
		// Scaled cloud
		n = vrandint(4, 200);
		let e = vrandint(-4, 5);
		let s: f32 = 1.0;
		for (let i = 0; i < e; ++i) {
			s *= 10.0;
		}
		for (let i = 0; i > e; --i) {
			s *= 0.1;
		}
		for (let i = 0; i < n; ++i) {
			let p = vrandvec(-1.0, 1.0);
			g_points[i] = b3MulSV(s, p);
		}
	} else if (type == 8) {
		// Far from the origin
		n = vrandint(4, 200);
		let offset = vrandvec(-5000.0, 5000.0);
		for (let i = 0; i < n; ++i) {
			let p = vrandvec(-1.0, 1.0);
			g_points[i] = b3Add(offset, p);
		}
	} else if (type == 9) {
		// Transformed box corners
		let h = vrandvec(0.01, 3.0);
		let xf = vrandxf(10.0);
		n = 8;
		g_points[0] = b3TransformPoint(xf, b3Vec3(h.x, h.y, h.z));
		g_points[1] = b3TransformPoint(xf, b3Vec3(h.x, h.y, -h.z));
		g_points[2] = b3TransformPoint(xf, b3Vec3(h.x, -h.y, h.z));
		g_points[3] = b3TransformPoint(xf, b3Vec3(h.x, -h.y, -h.z));
		g_points[4] = b3TransformPoint(xf, b3Vec3(-h.x, h.y, h.z));
		g_points[5] = b3TransformPoint(xf, b3Vec3(-h.x, h.y, -h.z));
		g_points[6] = b3TransformPoint(xf, b3Vec3(-h.x, -h.y, h.z));
		g_points[7] = b3TransformPoint(xf, b3Vec3(-h.x, -h.y, -h.z));
	} else if (type == 10) {
		// Few points
		n = vrandint(0, 7);
		for (let i = 0; i < n; ++i) {
			g_points[i] = vrandvec(-1.0, 1.0);
		}
	} else {
		// Lattice, many coplanar faces, optional jitter
		let a = vrandint(2, 6);
		let jitter: f32 = vrandint(0, 2) == 0 ? 0.0 : 1e-4;
		for (let i = 0; i < a; ++i) {
			for (let j = 0; j < a; ++j) {
				for (let l = 0; l < a; ++l) {
					let noise = vrandvec(-jitter, jitter);
					let p = b3Vec3(<f32>i, <f32>j, <f32>l);
					g_points[n++] = b3Add(p, noise);
				}
			}
		}
	}

	let maxCount = pickMaxCount(n);

	// Upstream b3CreateHull crashes (3, 27) or never returns (the rest) on these clouds within
	// 1e-5 of a plane. See notes/hull.md.
	for (let i = 0; i < 6; ++i) {
		if (k == s_skip[i]) {
			vtag("skip");
			vi32(k);
			vend();
			return;
		}
	}

	vtag("random");
	vi32(k);
	vi32(n);
	vi32(maxCount);
	vend();

	let hull = b3CreateHull(g_points, n, maxCount);
	checkHull("hull", hull);
}

export function phaseShapes(k: i32): void {
	vseed(9000 + k);

	let sides = 3 + (k % 30);
	let height = vrandf(0.05, 4.0);
	let radius = vrandf(0.05, 3.0);
	let yOffset = vrandf(-2.0, 2.0);
	vtag("cylinder");
	vi32(sides);
	vend();
	let cylinder = b3CreateCylinder(height, radius, yOffset, sides);
	checkHull("hull", cylinder);

	let slices = 4 + (k % 29);
	let radius1 = vrandf(0.05, 3.0);
	let radius2 = vrandf(0.05, 3.0);
	vtag("cone");
	vi32(slices);
	vend();
	let cone = b3CreateCone(height, radius1, radius2, slices);
	checkHull("hull", cone);

	let rockRadius = vrandf(0.01, 10.0);
	let rock = b3CreateRock(rockRadius);
	checkHull("rock", rock);

	let complexRadius = vrandf(0.01, 10.0);
	let complex = b3CreateComplexHull(complexRadius);
	checkHull("complex", complex);
}

function boxQueries(hull: b3HullData): void {
	// Axis aligned rays hit exactly zero denominators.
	let axes = new StaticArray<b3Vec3>(3);
	axes[0] = b3Vec3_axisX;
	axes[1] = b3Vec3_axisY;
	axes[2] = b3Vec3_axisZ;
	for (let i = 0; i < 3; ++i) {
		let o = vrandvec(-3.0, 3.0);
		queryRay(hull, o, b3MulSV(6.0, axes[i]), 1.0);
		queryRay(hull, o, b3MulSV(-6.0, axes[i]), 1.0);
		queryRay(hull, b3Vec3_zero, axes[i], 1.0);
	}
}

export function phaseBoxes(k: i32): void {
	vseed(12000 + k);

	let h = vrandvec(0.01, 3.0);
	if (k % 7 == 0) {
		h.y = 0.0001;
	}
	let xf = vrandxf(5.0);
	let offset = vrandvec(-5.0, 5.0);

	let box = b3MakeBoxHull(h.x, h.y, h.z);
	checkHull("box", box.base);
	boxQueries(box.base);

	let cube = b3MakeCubeHull(h.x);
	printHull("cube", cube.base);

	let offsetBox = b3MakeOffsetBoxHull(h.x, h.y, h.z, offset);
	printHull("offset", offsetBox.base);
	boxQueries(offsetBox.base);

	let xfBox = b3MakeTransformedBoxHull(h.x, h.y, h.z, xf);
	checkHull("xfbox", xfBox.base);

	let postScale = randomScale(k % 4);
	let scaled = b3MakeScaledBoxHull(h, xf, postScale);
	printHull("scaled", scaled.base);
	queryHull(scaled.base, 8, 8);

	let sh = h;
	let sxf = xf;
	let minHalfWidth = vrandf(0.001, 0.5);
	b3ScaleBox(sh, sxf, postScale, minHalfWidth);
	vtag("scalebox");
	vvec(sh);
	vvec(sxf.p);
	vvec(sxf.q.v);
	vf32(sxf.q.s);
	vend();
}

export function phaseMap(): void {
	vseed(777);

	let hulls = new Array<b3HullData>();
	for (let i = 0; i < 6; ++i) {
		let n = vrandint(8, 40);
		for (let j = 0; j < n; ++j) {
			g_points[j] = vrandvec(-1.0, 1.0);
		}
		hulls.push(b3CreateHull(g_points, n, n)!);
		// Same content, distinct allocation
		hulls.push(b3CreateHull(g_points, n, n)!);
		let clone = b3CloneHull(hulls[hulls.length - 1])!;
		hulls.push(clone);
	}
	let hullCount = hulls.length;
	let box1 = b3MakeBoxHull(1.0, 2.0, 3.0);
	let box2 = b3MakeBoxHull(1.0, 2.0, 3.0);
	let box3 = b3MakeBoxHull(1.0, 2.0, 3.5);

	let map = new b3HullMap();
	b3HullMap_init(map);

	// The world hull database: reference counts
	vtag("db");
	for (let i = 0; i < hullCount; ++i) {
		let itr = b3HullMap_get(map, hulls[i]);
		if (b3HullMap_is_end(itr) == false) {
			b3HullMap_setVal(map, itr, b3HullMap_val(map, itr) + 1);
			vi32(1);
			vi32(b3HullMap_val(map, itr));
			vbool(b3HullMap_key(map, itr) == hulls[i]);
		} else {
			b3HullMap_insert(map, hulls[i], 1);
			vi32(0);
		}
	}
	let boxes = new Array<b3HullData>();
	boxes.push(box1.base);
	boxes.push(box2.base);
	boxes.push(box3.base);
	for (let i = 0; i < 3; ++i) {
		let itr = b3HullMap_get_or_insert(map, boxes[i], 1);
		vi32(b3HullMap_val(map, itr));
		vbool(b3HullMap_key(map, itr) == boxes[i]);
		b3HullMap_setVal(map, itr, b3HullMap_val(map, itr) + 1);
	}
	vi32(b3HullMap_size(map));
	vend();

	vtag("iterate");
	let count = 0;
	let sum = 0;
	for (let itr = b3HullMap_first(map); b3HullMap_is_end(itr) == false; itr = b3HullMap_next(map, itr)) {
		count += 1;
		sum += b3HullMap_val(map, itr);
	}
	vi32(count);
	vi32(sum);
	vend();

	// Remove references
	vtag("remove");
	for (let i = 0; i < hullCount; i += 2) {
		let itr = b3HullMap_get(map, hulls[i]);
		if (b3HullMap_is_end(itr)) {
			vi32(-1);
			continue;
		}
		let val = b3HullMap_val(map, itr) - 1;
		b3HullMap_setVal(map, itr, val);
		if (val == 0) {
			b3HullMap_erase_itr(map, itr);
			vi32(0);
		} else {
			vi32(val);
		}
	}
	vi32(b3HullMap_size(map));
	for (let i = 0; i < hullCount; ++i) {
		let itr = b3HullMap_get(map, hulls[i]);
		vbool(b3HullMap_is_end(itr));
		if (b3HullMap_is_end(itr) == false) {
			vi32(b3HullMap_val(map, itr));
		}
	}
	vend();

	// Compound baking: unique hull indices
	vtag("compound");
	let shared = new b3HullMap();
	b3HullMap_init(shared);
	b3HullMap_reserve(shared, hullCount);
	let sharedCount = 0;
	for (let i = 0; i < hullCount; ++i) {
		let itr = b3HullMap_get_or_insert(shared, hulls[i], sharedCount);
		let index = b3HullMap_val(shared, itr);
		vi32(index);
		if (index == sharedCount) {
			sharedCount += 1;
		}
	}
	vi32(sharedCount);
	vi32(b3HullMap_size(shared));
	vend();
	b3HullMap_cleanup(shared);
	b3HullMap_cleanup(map);

	vtag("compare");
	for (let i = 0; i < hullCount; ++i) {
		for (let j = 0; j < hullCount; ++j) {
			vbool(b3CompareHullData(hulls[i], hulls[j]));
		}
	}
	vbool(b3CompareHullData(box1.base, box2.base));
	vbool(b3CompareHullData(box1.base, box3.base));
	vend();
}

export function phase2D(k: i32): void {
	vseed(15000 + k);
	let pts = new StaticArray<b3Point2D>(64);
	let hull = new StaticArray<b3Point2D>(130);
	let count = vrandint(0, 25);
	let grid = vrandint(0, 2);
	for (let i = 0; i < count; ++i) {
		if (grid == 1) {
			let gx = vrandint(0, 4);
			let gy = vrandint(0, 4);
			let jx = vrandf(-0.001, 0.001);
			let jy = vrandf(-0.001, 0.001);
			pts[i].p.x = 0.1 * <f32>gx + jx;
			pts[i].p.y = 0.1 * <f32>gy + jy;
		} else {
			pts[i].p.x = vrandf(-1.0, 1.0);
			pts[i].p.y = vrandf(-1.0, 1.0);
		}
		pts[i].separation = vrandf(-0.1, 0.1);
		pts[i].originalIndex = i;
	}

	let hullCount = b3Hull2D(pts, count, hull);
	vtag("hull2d");
	vi32(count);
	vi32(hullCount);
	for (let i = 0; i < hullCount; ++i) {
		vf32(hull[i].p.x);
		vf32(hull[i].p.y);
		vf32(hull[i].separation);
		vi32(hull[i].originalIndex);
	}
	vend();

	let target = 3 + vrandint(0, 6);
	let simplified = b3SimplifyHull2D(hull, hullCount, target);
	vtag("simplify");
	vi32(target);
	vi32(simplified);
	for (let i = 0; i < simplified; ++i) {
		vf32(hull[i].p.x);
		vf32(hull[i].p.y);
		vi32(hull[i].originalIndex);
	}
	vend();
}

export function phaseLimits(k: i32): void {
	vseed(18000 + k);

	// Dense sphere clouds overflow the face or edge limits at high vertex budgets.
	let n = 200 + 100 * k;
	for (let i = 0; i < n; ++i) {
		let v = vrandvec(-1.0, 1.0);
		g_points[i] = b3Normalize(v);
	}
	let limits: StaticArray<i32> = [128, 100, 80, 66, 64];
	for (let i = 0; i < 5; ++i) {
		let hull = b3CreateHull(g_points, n, limits[i]);
		vtag("limit");
		vi32(n);
		vi32(limits[i]);
		vend();
		printHull("hull", hull);
	}

	printHull("null", b3CloneHull(null));
	let xf = vrandxf(1.0);
	printHull("null", b3CloneAndTransformHull(null, xf, b3Vec3_one));
}

// Corrupts a clone and compares the validating b3IsValidHull body against the C copy.
export function phaseValidate(k: i32): void {
	vseed(21000 + k);
	let n = vrandint(8, 60);
	for (let i = 0; i < n; ++i) {
		g_points[i] = vrandvec(-1.0, 1.0);
	}
	let hull = b3CreateHull(g_points, n, n)!;
	let clone = b3CloneHull(hull)!;
	let vertices = clone.vertices;
	let edges = clone.edges;
	let planes = clone.planes;
	let kind = k % 10;
	let v = vrandint(0, clone.vertexCount);
	let e = vrandint(0, clone.edgeCount);
	let f = vrandint(0, clone.faceCount);
	if (kind == 0) {
		clone.version ^= 1;
	} else if (kind == 1) {
		vertices[v].edge = <u8>((<i32>vertices[v].edge + 1) % clone.edgeCount);
	} else if (kind == 2) {
		edges[e].twin = <u8>((<i32>edges[e].twin + 2) % clone.edgeCount);
	} else if (kind == 3) {
		edges[e].next = edges[e].twin;
	} else if (kind == 4) {
		planes[f].offset = -planes[f].offset - 10.0;
	} else if (kind == 5) {
		clone.volume = 0.0;
	} else if (kind == 6) {
		clone.surfaceArea = -1.0;
	} else if (kind == 7) {
		clone.innerRadius = 0.0;
	} else if (kind == 8) {
		edges[e].origin = <u8>((<i32>edges[e].origin + 1) % clone.vertexCount);
	} else {
		edges[e].face = <u8>((<i32>edges[e].face + 1) % clone.faceCount);
	}
	vtag("validate");
	vi32(kind);
	vbool(b3ValidateHull(hull));
	vbool(b3ValidateHull(clone));
	vbool(b3CompareHullData(hull, clone));
	vend();
}
