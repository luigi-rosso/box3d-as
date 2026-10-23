// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for the manifold.c part of lib/manifold.as, mirrors verify/cref/manifold.c

import { vtag, vi32, vf32, vend, vseed, vrand32, vrandint, vrandf } from "./common";
import { pv3, runit, rv } from "./distance_common";
import { axisDirection, buildHulls, g_hulls, ppair, pplane } from "./manifold_common";
import {
	B3_PI,
	b3ComputeCosSin,
	b3Cross,
	b3Dot,
	b3MulAdd,
	b3Normalize,
	b3Plane,
	b3Vec3,
} from "../../lib/math_functions";
import {
	B3_MAX_CLIP_POINTS,
	b3ClipPolygon,
	b3ClipVertex,
	b3FeaturePair_single,
	b3FindIncidentFace,
	b3FlipPair,
	b3MakeFeaturePair,
} from "../../lib/manifold";

export { frameBoundary } from "./common";

const CLIP_CAPACITY: i32 = 2 * B3_MAX_CLIP_POINTS + 2;

const g_polygon = new StaticArray<b3ClipVertex>(CLIP_CAPACITY);
const g_out = new StaticArray<b3ClipVertex>(CLIP_CAPACITY);

function pclip(v: b3ClipVertex): void {
	pv3(v.position);
	vf32(v.separation);
	ppair(v.pair);
}

function runPairs(): void {
	const indices: StaticArray<i32> = [0, 1, 2, 7, 127, 128, 200, 255];
	for (let o1 = 0; o1 < 2; ++o1) {
		for (let o2 = 0; o2 < 2; ++o2) {
			for (let i = 0; i < 8; ++i) {
				for (let j = 0; j < 8; ++j) {
					let pair = b3MakeFeaturePair(o1, indices[i], o2, indices[j]);
					let flip = b3FlipPair(pair);
					vtag("pair");
					ppair(pair);
					ppair(flip);
					ppair(b3FlipPair(flip));
					vend();
				}
			}
		}
	}

	vtag("single");
	ppair(b3FeaturePair_single);
	vend();
}

function runIncidentFaces(): void {
	for (let h = 0; h < g_hulls.length; ++h) {
		let hull = g_hulls[h];
		for (let k = 0; k < 40; ++k) {
			let normal: b3Vec3;
			if (k < 6) {
				normal = axisDirection(k);
			} else {
				normal = runit();
			}
			vtag("inc");
			vi32(h);
			pv3(normal);
			for (let v = 0; v < hull.vertexCount; ++v) {
				vi32(b3FindIncidentFace(hull, normal, v));
			}
			vend();
		}
	}
}

function runClip(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let n = vrandint(3, 24);
		if ((i & 15) == 0) {
			n = vrandint(60, 2 * B3_MAX_CLIP_POINTS);
		}
		let center = rv(-1.0, 1.0);
		let axis = runit();
		let tangent = b3Normalize(b3Cross(axis, runit()));
		let bitangent = b3Cross(axis, tangent);
		let radius = vrandf(0.05, 2.0);

		for (let k = 0; k < n; ++k) {
			let angle = 2.0 * B3_PI * <f32>k / <f32>n;
			let cs = b3ComputeCosSin(angle);
			let jitter = vrandf(0.9, 1.0);
			if (n > 24) {
				// Large polygons stay convex so the output fits the clip buffer
				jitter = 1.0;
			}
			let p = b3MulAdd(center, radius * jitter * cs.cosine, tangent);
			p = b3MulAdd(p, radius * jitter * cs.sine, bitangent);
			g_polygon[k].position = p;
			g_polygon[k].separation = vrandf(-1.0, 1.0);
			g_polygon[k].pair.owner1 = <u8>(vrand32() & 1);
			g_polygon[k].pair.index1 = <u8>vrand32();
			g_polygon[k].pair.owner2 = <u8>(vrand32() & 1);
			g_polygon[k].pair.index2 = <u8>vrand32();
		}

		let clipPlane: b3Plane;
		clipPlane.normal = runit();
		let kind = vrandint(0, 4);
		if (n > 24 && kind == 1) {
			// Noise around a coplanar plane alternates sides and would overflow the buffer
			kind = 2;
		}
		if (kind == 0) {
			// Through a polygon vertex
			let k = vrandint(0, n);
			clipPlane.offset = b3Dot(clipPlane.normal, g_polygon[k].position);
		} else if (kind == 1) {
			// In the polygon plane, so every distance is about zero
			clipPlane.normal = axis;
			clipPlane.offset = b3Dot(axis, center);
		} else {
			let shift = vrandf(-radius, radius);
			clipPlane.offset = b3Dot(clipPlane.normal, center) + shift;
		}

		let edge = vrandint(0, 256);
		let refPlane: b3Plane;
		refPlane.normal = runit();
		refPlane.offset = vrandf(-1.0, 1.0);

		let outCount = b3ClipPolygon(g_out, g_polygon, n, clipPlane, edge, refPlane);

		vtag("clip");
		vi32(i);
		vi32(n);
		pplane(clipPlane);
		vi32(edge);
		pplane(refPlane);
		vi32(outCount);
		vend();
		for (let k = 0; k < outCount; ++k) {
			vtag("cv");
			pclip(g_out[k]);
			vend();
		}
	}
}

export function begin(): void {
	buildHulls();
}

export function run(): void {
	vseed(0x6d616e69666f6c64);
	runPairs();
	runIncidentFaces();
	runClip(20000);
}
