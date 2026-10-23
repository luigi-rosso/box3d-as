// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/height_field.as, mirrors verify/cref/height_field.c

import { vtag, vi32, vu32, vbool, vf32, vu64, vend, vseed, vrandint, vrandf, vspecialf } from "./common";
import { B3_MAX_SHAPE_CAST_POINTS } from "../../lib/constants";
import { b3AABB, b3AABB_Center, b3Add, b3MulSV, b3NormalizeQuat, b3Quat, b3Sub, b3Transform, b3TransformPoint, b3Vec3 } from "../../lib/math_functions";
import {
	B3_HEIGHT_FIELD_HOLE,
	b3Capsule,
	b3CastOutput,
	b3HeightFieldData,
	b3HeightFieldDef,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeProxy,
} from "../../lib/collision";
import {
	b3CollideMoverAndHeightField,
	b3ComputeHeightFieldAABB,
	b3CreateGrid,
	b3CreateHeightField,
	b3CreateWave,
	b3GetHeightFieldMaterial,
	b3GetHeightFieldTriangle,
	b3OverlapHeightField,
	b3QueryHeightField,
	b3RayCastHeightField,
	b3ShapeCastHeightField,
} from "../../lib/height_field";
import { b3GetHeightFieldTriangleCount } from "../../lib/shape";

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

function printHeightField(hf: b3HeightFieldData): void {
	vtag("hf");
	vu64(hf.version);
	vu64(hf.hash);
	vi32(hf.byteCount);
	pab(hf.aabb);
	vf32(hf.minHeight);
	vf32(hf.maxHeight);
	vf32(hf.heightScale);
	pv3(hf.scale);
	vi32(hf.columnCount);
	vi32(hf.rowCount);
	vi32(hf.heightsOffset);
	vi32(hf.materialOffset);
	vi32(hf.flagsOffset);
	vu32(hf.clockwise);
	vi32(b3GetHeightFieldTriangleCount(hf));
	vend();

	let heights = hf.compressedHeights;
	let heightCount = hf.rowCount * hf.columnCount;
	for (let i = 0; i < heightCount; i += 16) {
		vtag("h");
		for (let j = i; j < i + 16 && j < heightCount; ++j) {
			vu32(heights[j]);
		}
		vend();
	}

	let materials = hf.materialIndices;
	let flags = hf.flags;
	let cellCount = (hf.rowCount - 1) * (hf.columnCount - 1);
	for (let i = 0; i < cellCount; ++i) {
		vtag("c");
		vu32(materials[i]);
		vu32(flags[2 * i]);
		vu32(flags[2 * i + 1]);
		vend();
	}

	for (let i = 0; i < 2 * cellCount; ++i) {
		if (materials[i >> 1] == B3_HEIGHT_FIELD_HOLE) {
			continue;
		}

		let t = b3GetHeightFieldTriangle(hf, i);
		vtag("tri");
		pv3(t.vertices[0]);
		pv3(t.vertices[1]);
		pv3(t.vertices[2]);
		vi32(t.i1);
		vi32(t.i2);
		vi32(t.i3);
		vi32(t.flags);
		vi32(b3GetHeightFieldMaterial(hf, i));
		vend();
	}
}

const MAX_FIELDS = 64;
let g_fields = new Array<b3HeightFieldData>();

function addField(hf: b3HeightFieldData): void {
	printHeightField(hf);
	if (g_fields.length < MAX_FIELDS) {
		g_fields.push(hf);
	}
}

// Seeded heights with holes. special adds infinities and NaN, clamped by construction.
function randomField(seed: i32, special: bool): b3HeightFieldData {
	vseed(<u64>(3000 + seed));
	let columns = vrandint(2, 40);
	let rows = vrandint(2, 40);
	let def = new b3HeightFieldDef();
	def.countX = columns;
	def.countZ = rows;
	def.scale.x = vrandf(0.2, 3.0);
	def.scale.y = vrandf(0.1, 4.0);
	def.scale.z = vrandf(0.2, 3.0);
	let amplitude = vrandf(0.0, 10.0);
	def.globalMinimumHeight = vrandf(-12.0, 0.0);
	def.globalMaximumHeight = vrandf(0.0, 12.0);
	if (vrandint(0, 5) == 0) {
		// degenerate range clamps to the linear slop
		def.globalMaximumHeight = def.globalMinimumHeight;
	}
	def.clockwiseWinding = vrandint(0, 2) == 1;

	let heights = new StaticArray<f32>(rows * columns);
	for (let i = 0; i < rows * columns; ++i) {
		heights[i] = vrandf(-amplitude, amplitude);
		if (special && vrandint(0, 8) == 0) {
			heights[i] = vspecialf();
		}
	}
	def.heights = heights;

	let cellCount = (rows - 1) * (columns - 1);
	let materials = new StaticArray<u8>(cellCount);
	let holeChance = vrandint(0, 6);
	for (let i = 0; i < cellCount; ++i) {
		materials[i] = <u8>vrandint(0, 5);
		if (holeChance > 0 && vrandint(0, holeChance * 3) == 0) {
			materials[i] = <u8>B3_HEIGHT_FIELD_HOLE;
		}
	}
	def.materialIndices = vrandint(0, 4) == 0 ? null : materials;

	return b3CreateHeightField(def);
}

export function buildAll(): i32 {
	addField(b3CreateGrid(2, 2, b3Vec3(1, 1, 1), false));
	addField(b3CreateGrid(20, 20, b3Vec3(1, 1, 1), true));
	addField(b3CreateGrid(9, 33, b3Vec3(0.5, 2, 1.5), true));
	addField(b3CreateWave(20, 20, b3Vec3(1, 1, 1), 0.1, 0.07, false));
	addField(b3CreateWave(40, 25, b3Vec3(0.75, 3, 0.5), 0.05, 0.13, true));
	addField(b3CreateWave(17, 64, b3Vec3(2, 0.5, 2), 0.21, 0.03, true));

	for (let i = 0; i < 30; ++i) {
		addField(randomField(i, false));
	}

	// construction only
	for (let i = 0; i < 6; ++i) {
		printHeightField(randomField(100 + i, true));
	}

	vtag("built");
	vi32(g_fields.length);
	vend();
	return g_fields.length;
}

let g_queryCount = 0;

function queryCallback(a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32): bool {
	vtag("q");
	vi32(triangleIndex);
	pv3(a);
	pv3(b);
	pv3(c);
	vend();
	g_queryCount += 1;
	return true;
}

// A hole triangle has no vertices; aim at the field center instead.
function triangleCenter(hf: b3HeightFieldData, ti: i32): b3Vec3 {
	if (hf.materialIndices[ti >> 1] == B3_HEIGHT_FIELD_HOLE) {
		return b3AABB_Center(hf.aabb);
	}

	let t = b3GetHeightFieldTriangle(hf, ti);
	return b3MulSV(<f32>1.0 / <f32>3.0, b3Add(b3Add(t.vertices[0], t.vertices[1]), t.vertices[2]));
}

function boxPad(box: b3AABB): f32 {
	let size = b3Sub(box.upperBound, box.lowerBound);
	return 0.1 * (size.x + size.y + size.z) + 0.5;
}

export function queries(fieldIndex: i32): void {
	let hf = g_fields[fieldIndex];
	vseed(<u64>(70000 + fieldIndex));

	vtag("queries");
	vi32(fieldIndex);
	vend();

	for (let i = 0; i < 4; ++i) {
		let xf = randTransform(10.0);
		vtag("aabb");
		pab(b3ComputeHeightFieldAABB(hf, xf));
		vend();
	}

	let box = hf.aabb;
	let pad = boxPad(box);
	let triangleCount = b3GetHeightFieldTriangleCount(hf);

	for (let i = 0; i < 200; ++i) {
		let input = b3RayCastInput();
		let kind = vrandint(0, 4);
		if (kind == 0) {
			input.origin = randInBox(box, pad);
			input.translation = randVec(-3.0 * pad, 3.0 * pad);
		} else if (kind == 1) {
			// vertical
			input.origin = randInBox(box, 0.0);
			input.origin.y = box.upperBound.y + vrandf(0.1, pad);
			input.translation = b3Vec3(0.0, -(box.upperBound.y - box.lowerBound.y) - 2.0 * pad, 0.0);
		} else {
			let c = triangleCenter(hf, vrandint(0, triangleCount));
			input.origin = b3Add(c, randVec(-pad, pad));
			input.translation = b3MulSV(vrandf(1.0, 2.5), b3Sub(c, input.origin));
		}
		input.maxFraction = vrandint(0, 3) == 0 ? vrandf(0.1, 1.0) : 1.0;

		vtag("ray");
		pcast(b3RayCastHeightField(hf, input));
		vend();
	}

	for (let i = 0; i < 10; ++i) {
		let c = randInBox(box, 0.2 * pad);
		let e = randVec(0.0, pad);
		let bounds = b3AABB(b3Sub(c, e), b3Add(c, e));
		g_queryCount = 0;
		vtag("query");
		pab(bounds);
		vend();
		b3QueryHeightField(hf, bounds, queryCallback);
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

export function casts(fieldIndex: i32): void {
	let hf = g_fields[fieldIndex];
	vseed(<u64>(90000 + fieldIndex));

	vtag("casts");
	vi32(fieldIndex);
	vend();

	let box = hf.aabb;
	let pad = boxPad(box);
	let size = b3Sub(box.upperBound, box.lowerBound);
	let shapeScale: f32 = 0.03 * (size.x + size.y + size.z) + 0.1;
	let triangleCount = b3GetHeightFieldTriangleCount(hf);

	let points = new StaticArray<b3Vec3>(B3_MAX_SHAPE_CAST_POINTS);

	for (let i = 0; i < 40; ++i) {
		let proxy = randProxy(points, shapeScale);
		let xf = randTransform(pad);
		let p = vrandint(0, 2) == 0 ? randInBox(box, 0.3 * pad) : triangleCenter(hf, vrandint(0, triangleCount));
		for (let k = 0; k < proxy.count; ++k) {
			points[k] = b3TransformPoint(xf, b3Add(points[k], p));
		}
		vtag("overlap");
		vbool(b3OverlapHeightField(hf, xf, proxy));
		vend();
	}

	for (let i = 0; i < 60; ++i) {
		let input = new b3ShapeCastInput();
		input.proxy = randProxy(points, shapeScale);
		let origin: b3Vec3;
		let target: b3Vec3;
		let kind = vrandint(0, 3);
		if (kind == 0) {
			origin = randInBox(box, pad);
			target = randInBox(box, 0.0);
		} else if (kind == 1) {
			target = triangleCenter(hf, vrandint(0, triangleCount));
			origin = target;
			origin.y = box.upperBound.y + vrandf(shapeScale, pad);
		} else {
			target = triangleCenter(hf, vrandint(0, triangleCount));
			origin = b3Add(target, randVec(-pad, pad));
		}
		for (let k = 0; k < input.proxy.count; ++k) {
			points[k] = b3Add(points[k], origin);
		}
		input.translation = b3MulSV(vrandf(0.5, 2.0), b3Sub(target, origin));
		input.maxFraction = vrandint(0, 3) == 0 ? vrandf(0.2, 1.0) : 1.0;
		input.canEncroach = vrandint(0, 2) == 1;

		vtag("cast");
		pcast(b3ShapeCastHeightField(hf, input));
		vend();
	}

	let planes = new StaticArray<b3PlaneResult>(16);
	for (let i = 0; i < 40; ++i) {
		let mover = b3Capsule();
		let c = vrandint(0, 2) == 0 ? randInBox(box, 0.2 * pad) : triangleCenter(hf, vrandint(0, triangleCount));
		mover.center1 = b3Add(c, randVec(-shapeScale, shapeScale));
		mover.center2 = b3Add(mover.center1, randVec(-shapeScale, shapeScale));
		mover.radius = vrandf(0.05, 2.0 * shapeScale);
		let offset = vrandint(0, 4);
		let capacity = vrandint(1, 12);
		for (let k = 0; k < 16; ++k) {
			planes[k] = b3PlaneResult();
		}
		let count = b3CollideMoverAndHeightField(planes, offset, capacity, hf, mover);
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
