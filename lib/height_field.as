// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/height_field.c

import { B3_ASSERT, B3_ASSERT_ENABLED, FLT_MAX, UINT16_MAX, b3FloatToIntSat, b3FloatToUIntSat, b3Hash64NonZero } from "./core";
import { B3_LINEAR_SLOP, B3_MAX_AABB_MARGIN, B3_MAX_SHAPE_CAST_POINTS } from "./constants";
import {
	B3_PI,
	b3AABB,
	b3AABB_Center,
	b3AABB_Extents,
	b3AABB_Overlaps,
	b3AABB_Transform,
	b3Abs,
	b3Add,
	b3ClampFloat,
	b3ComputeCosSin,
	b3Cross,
	b3Dot,
	b3Lerp,
	b3MakeAABB,
	b3Max,
	b3MaxFloat,
	b3Min,
	b3MinFloat,
	b3Mul,
	b3MulAdd,
	b3MulSV,
	b3Neg,
	b3Normalize,
	b3Plane,
	b3Sub,
	b3Transform,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3MakePlaneFromPoints, b3PlaneSeparation, b3SignedVolume, b3Triangle } from "./math_internal";
import {
	B3_HEIGHT_FIELD_HOLE,
	B3_HEIGHT_FIELD_VERSION,
	b3Capsule,
	b3CastOutput,
	b3DistanceInput,
	b3HeightFieldData,
	b3HeightFieldDef,
	b3MeshQueryFcn,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeCastPairInput,
	b3ShapeProxy,
	b3SimplexCache,
	b3_concaveEdge1,
	b3_concaveEdge2,
	b3_concaveEdge3,
	b3_inverseConcaveEdge1,
	b3_inverseConcaveEdge2,
	b3_inverseConcaveEdge3,
} from "./collision";
import { b3AddV, b3GetXV, b3GetZV, b3IntersectRayTriangle, b3LoadV, b3MaxV, b3MinV, b3MulV, b3SplatV, b3SubV, b3TestBoundsTriangleOverlap, b3_halfV } from "./simd";
import { b3RayCastAABB } from "./aabb";
import { b3SetProxy, b3ShapeCast, b3ShapeDistance } from "./distance";
import { b3SetLocalProxy } from "./shape";

/*
	Convention

	index = row * columnCount + column
	height = minHeight + heightScale * compressedHeights[index];

	column = index % columnCount;
	row = index / columnCount;

	x-axis : columns
	z-axis : rows

	00 --- 01 --- 02 --- 03 X
	|  0   |  1   |  2   |
	04 --- 05 --- 06 --- 07
	|  3   |  4   |  5   |
	08 --- 09 --- 10 --- 11
	|  6   |  7   |  8   |
	12 --- 13 --- 14 --- 15
	Z

	The quads exist before the column and row ends: row < rowCount - 1 and column < columnCount - 1
	quadIndex = row * (columnCount - 1) + column

	Quad origin index from quad index (needs row):
	index = quadIndex + row * columnCount

	Triangle index is related to the quad index
	triangleIndex = 2 * quadIndex + (0/1)
	quadIndex = triangleIndex / 2

	Row and column from quad index:
	row = quadIndex / (columnCount - 1)
	column = quadIndex - row * (columnCount - 1)

	The triangle diagonal is fixed.

	triangle0 = {00, 04, 01} -> {11, 21, 12}
	triangle1 = {04, 05, 01} -> {22, 12, 21}

	11      12
	00 ---- 01
	|     / |
	| 0 / 1 | 1
	| /     |
	04 ---- 05
	21     22

	For adjacency we have

	   NA
	   00 ---- 01 ---- 02
	   |     / |     / |
	NA | 0 / 1 | 2 / 3 |
	   | /     | /     |
	   04 ---- 05 ---- 06
	   |     / |     / |
	NA | 6 / 7 | 8 / 9 |
	   | /     | /     |
	   08 ---- 09 ---- 10

	   0: NA, NA, 1
	   1: 0, 6, 2

	Triangle layouts

	   11  3  12
	   1 ----  3
	   |     /
	 1 | 0 / 2
	   | /
	   2
	   21

			  12
			  2
			/ |
		2 / 1 | 1
		/     |
	   3 ---- 1
	   21  3  22
 */

// The C blob image, so the hash matches C byte for byte
function b3SerializeHeightField(hf: b3HeightFieldData): Uint8Array {
	let bytes = new Uint8Array(hf.byteCount);
	let view = new DataView(bytes.buffer);
	view.setUint64(0, hf.version, true);
	view.setUint64(8, hf.hash, true);
	view.setInt32(16, hf.byteCount, true);
	view.setFloat32(20, hf.aabb.lowerBound.x, true);
	view.setFloat32(24, hf.aabb.lowerBound.y, true);
	view.setFloat32(28, hf.aabb.lowerBound.z, true);
	view.setFloat32(32, hf.aabb.upperBound.x, true);
	view.setFloat32(36, hf.aabb.upperBound.y, true);
	view.setFloat32(40, hf.aabb.upperBound.z, true);
	view.setFloat32(44, hf.minHeight, true);
	view.setFloat32(48, hf.maxHeight, true);
	view.setFloat32(52, hf.heightScale, true);
	view.setFloat32(56, hf.scale.x, true);
	view.setFloat32(60, hf.scale.y, true);
	view.setFloat32(64, hf.scale.z, true);
	view.setInt32(68, hf.columnCount, true);
	view.setInt32(72, hf.rowCount, true);
	view.setInt32(76, hf.heightsOffset, true);
	view.setInt32(80, hf.materialOffset, true);
	view.setInt32(84, hf.flagsOffset, true);
	bytes[88] = hf.clockwise;

	let heights = hf.compressedHeights;
	for (let i = 0; i < heights.length; ++i) {
		view.setUint16(hf.heightsOffset + 2 * i, heights[i], true);
	}

	let materialIndices = hf.materialIndices;
	for (let i = 0; i < materialIndices.length; ++i) {
		bytes[hf.materialOffset + i] = materialIndices[i];
	}

	let flags = hf.flags;
	for (let i = 0; i < flags.length; ++i) {
		bytes[hf.flagsOffset + i] = flags[i];
	}

	return bytes;
}

export function b3CreateHeightField(data: b3HeightFieldDef): b3HeightFieldData {
	let columnCount = data.countX;
	let rowCount = data.countZ;

	let heightCount = columnCount * rowCount;
	if (B3_ASSERT_ENABLED) B3_ASSERT(heightCount >= 4);

	let cellCount = (columnCount - 1) * (rowCount - 1);
	let triangleCount = 2 * cellCount;

	// The constructor allocates the height, material, and flag arrays and sets the offsets and byteCount.
	let hf = new b3HeightFieldData(columnCount, rowCount);

	hf.version = B3_HEIGHT_FIELD_VERSION;
	hf.scale = data.scale;
	hf.clockwise = data.clockwiseWinding ? 1 : 0;

	let compressedHeights = hf.compressedHeights;
	let materialIndices = hf.materialIndices;
	let flags = hf.flags;

	let heights = data.heights;

	if (B3_ASSERT_ENABLED) B3_ASSERT(data.globalMinimumHeight <= data.globalMaximumHeight);
	hf.minHeight = data.globalMinimumHeight;
	hf.maxHeight = data.globalMaximumHeight;

	let height = b3MaxFloat(hf.maxHeight - hf.minHeight, B3_LINEAR_SLOP());
	hf.heightScale = height / <f32>UINT16_MAX;

	let lowerHeightBound = hf.maxHeight;
	let upperHeightBound = hf.minHeight;

	let invHeightScale: f32 = 1.0 / hf.heightScale;
	for (let i = 0; i < heightCount; ++i) {
		let clampedHeight = b3ClampFloat(heights[i], hf.minHeight, hf.maxHeight);
		let scaledHeight = (clampedHeight - hf.minHeight) * invHeightScale;
		compressedHeights[i] = <u16>b3FloatToUIntSat(b3MinFloat(scaledHeight, <f32>UINT16_MAX));

		lowerHeightBound = b3MinFloat(lowerHeightBound, clampedHeight);
		upperHeightBound = b3MaxFloat(upperHeightBound, clampedHeight);
	}

	// Use decompressed heights for accurate convexity metrics.
	let decompressedHeights = new StaticArray<f32>(heightCount);
	for (let i = 0; i < heightCount; ++i) {
		decompressedHeights[i] = hf.minHeight + hf.heightScale * <f32>compressedHeights[i];
	}
	heights = decompressedHeights;

	let defMaterialIndices = data.materialIndices;
	if (defMaterialIndices != null) {
		for (let i = 0; i < cellCount; ++i) {
			materialIndices[i] = defMaterialIndices[i];
		}
	} else {
		for (let i = 0; i < cellCount; ++i) {
			materialIndices[i] = 0;
		}
	}

	hf.aabb.lowerBound = b3Vec3(0.0, hf.scale.y * lowerHeightBound, 0.0);
	hf.aabb.upperBound = b3Vec3(
		hf.scale.x * <f32>(hf.columnCount - 1),
		hf.scale.y * upperHeightBound,
		hf.scale.z * <f32>(hf.rowCount - 1),
	);

	let cos5Deg: f32 = 0.9962;
	let scale = hf.scale;

	let triangleIndex = 0;
	for (let row = 0; row < rowCount - 1; ++row) {
		for (let column = 0; column < columnCount - 1; ++column) {
			// todo compute convexity flags
			// This requires a couple things
			// - determine all 3 adjacent triangles for each triangle
			// - consider clockwise winding
			// - consider borders where there is no adjacent triangle

			let triangleIndex1 = triangleIndex;
			let triangleIndex2 = triangleIndex + 1;
			triangleIndex += 2;

			let cellIndex = row * (columnCount - 1) + column;

			if (materialIndices[cellIndex] == B3_HEIGHT_FIELD_HOLE) {
				continue;
			}

			let flags1 = 0;
			let flags2 = 0;

			let plane1: b3Plane;
			let plane2: b3Plane;

			let index11 = row * columnCount + column;
			let index12 = index11 + 1;
			let index21 = (row + 1) * columnCount + column;
			let index22 = index21 + 1;

			{
				let height11 = heights[index11];
				let height12 = heights[index12];
				let height21 = heights[index21];
				let height22 = heights[index22];

				let x1 = <f32>column;
				let x2 = <f32>(column + 1);
				let z1 = <f32>row;
				let z2 = <f32>(row + 1);

				// triangle 0 : 11, 21, 12
				let vs00 = b3Mul(scale, b3Vec3(x1, height11, z1));
				let vs01 = b3Mul(scale, b3Vec3(x1, height21, z2));
				let vs02 = b3Mul(scale, b3Vec3(x2, height12, z1));
				plane1 = b3MakePlaneFromPoints(vs00, vs01, vs02);

				// triangle 1 : 22, 12, 21
				let vs10 = b3Mul(scale, b3Vec3(x2, height22, z2));
				let vs11 = b3Mul(scale, b3Vec3(x2, height12, z1));
				let vs12 = b3Mul(scale, b3Vec3(x1, height21, z2));
				plane2 = b3MakePlaneFromPoints(vs10, vs11, vs12);

				let separation = b3PlaneSeparation(plane1, vs10);
				let cosAngle = b3Dot(plane1.normal, plane2.normal);
				if (separation > 0.0 || cosAngle > cos5Deg) {
					flags1 |= b3_concaveEdge2;
					flags2 |= b3_concaveEdge2;
				}
				if (separation < 0.0 || cosAngle > cos5Deg) {
					flags1 |= b3_inverseConcaveEdge2;
					flags2 |= b3_inverseConcaveEdge2;
				}
			}

			// top
			let topCellIndex = (row - 1) * (columnCount - 1) + column;
			if (row > 0 && materialIndices[topCellIndex] != B3_HEIGHT_FIELD_HOLE) {
				if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= topCellIndex && topCellIndex < cellCount);

				let r = row - 1;
				let c = column;

				let i11 = r * columnCount + c;
				let i12 = i11 + 1;
				let i21 = (r + 1) * columnCount + c;
				let i22 = i21 + 1;

				if (B3_ASSERT_ENABLED) B3_ASSERT(i21 == index11);
				if (B3_ASSERT_ENABLED) B3_ASSERT(i22 == index12);

				let h12 = heights[i12];
				let h21 = heights[i21];
				let h22 = heights[i22];

				let x1 = <f32>c;
				let x2 = <f32>(c + 1);
				let z1 = <f32>r;
				let z2 = <f32>(r + 1);

				// triangle 1
				let vs0 = b3Mul(scale, b3Vec3(x2, h22, z2));
				let vs1 = b3Mul(scale, b3Vec3(x2, h12, z1));
				let vs2 = b3Mul(scale, b3Vec3(x1, h21, z2));

				let n = b3Normalize(b3Cross(b3Sub(vs1, vs0), b3Sub(vs2, vs0)));

				let separation = b3PlaneSeparation(plane1, vs1);
				let cosAngle = b3Dot(plane1.normal, n);
				if (separation > 0.0 || cosAngle > cos5Deg) {
					flags1 |= b3_concaveEdge3;
				}
				if (separation < 0.0 || cosAngle > cos5Deg) {
					flags1 |= b3_inverseConcaveEdge3;
				}
			}

			let bottomCellIndex = (row + 1) * (columnCount - 1) + column;
			if (row + 1 < rowCount - 1 && materialIndices[bottomCellIndex] != B3_HEIGHT_FIELD_HOLE) {
				if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= bottomCellIndex && bottomCellIndex < cellCount);

				let r = row + 1;
				let c = column;

				let i11 = r * columnCount + c;
				let i12 = i11 + 1;
				let i21 = (r + 1) * columnCount + c;

				if (B3_ASSERT_ENABLED) B3_ASSERT(i11 == index21);
				if (B3_ASSERT_ENABLED) B3_ASSERT(i12 == index22);

				let h11 = heights[i11];
				let h12 = heights[i12];
				let h21 = heights[i21];

				let x1 = <f32>c;
				let x2 = <f32>(c + 1);
				let z1 = <f32>r;
				let z2 = <f32>(r + 1);

				// triangle 0
				let vs0 = b3Mul(scale, b3Vec3(x1, h11, z1));
				let vs1 = b3Mul(scale, b3Vec3(x1, h21, z2));
				let vs2 = b3Mul(scale, b3Vec3(x2, h12, z1));

				let n = b3Normalize(b3Cross(b3Sub(vs1, vs0), b3Sub(vs2, vs0)));

				let separation = b3PlaneSeparation(plane2, vs1);
				let cosAngle = b3Dot(plane2.normal, n);
				if (separation > 0.0 || cosAngle > cos5Deg) {
					flags2 |= b3_concaveEdge3;
				}
				if (separation < 0.0 || cosAngle > cos5Deg) {
					flags2 |= b3_inverseConcaveEdge3;
				}
			}

			let leftCellIndex = row * (columnCount - 1) + column - 1;
			if (column - 1 >= 0 && materialIndices[leftCellIndex] != B3_HEIGHT_FIELD_HOLE) {
				if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= leftCellIndex && leftCellIndex < cellCount);

				let r = row;
				let c = column - 1;

				let i11 = r * columnCount + c;
				let i12 = i11 + 1;
				let i21 = (r + 1) * columnCount + c;
				let i22 = i21 + 1;

				if (B3_ASSERT_ENABLED) B3_ASSERT(i12 == index11);
				if (B3_ASSERT_ENABLED) B3_ASSERT(i22 == index21);

				let h12 = heights[i12];
				let h21 = heights[i21];
				let h22 = heights[i22];

				let x1 = <f32>c;
				let x2 = <f32>(c + 1);
				let z1 = <f32>r;
				let z2 = <f32>(r + 1);

				// triangle 1
				let vs0 = b3Mul(scale, b3Vec3(x2, h22, z2));
				let vs1 = b3Mul(scale, b3Vec3(x2, h12, z1));
				let vs2 = b3Mul(scale, b3Vec3(x1, h21, z2));

				let n = b3Normalize(b3Cross(b3Sub(vs1, vs0), b3Sub(vs2, vs0)));

				let separation = b3PlaneSeparation(plane1, vs2);
				let cosAngle = b3Dot(plane1.normal, n);
				if (separation > 0.0 || cosAngle > cos5Deg) {
					flags1 |= b3_concaveEdge1;
				}
				if (separation < 0.0 || cosAngle > cos5Deg) {
					flags1 |= b3_inverseConcaveEdge1;
				}
			}

			let rightCellIndex = row * (columnCount - 1) + column + 1;
			if (column + 1 < columnCount - 1 && materialIndices[rightCellIndex] != B3_HEIGHT_FIELD_HOLE) {
				if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= rightCellIndex && rightCellIndex < cellCount);

				let r = row;
				let c = column + 1;

				let i11 = r * columnCount + c;
				let i12 = i11 + 1;
				let i21 = (r + 1) * columnCount + c;

				if (B3_ASSERT_ENABLED) B3_ASSERT(i11 == index12);
				if (B3_ASSERT_ENABLED) B3_ASSERT(i21 == index22);

				let h11 = heights[i11];
				let h12 = heights[i12];
				let h21 = heights[i21];

				let x1 = <f32>c;
				let x2 = <f32>(c + 1);
				let z1 = <f32>r;
				let z2 = <f32>(r + 1);

				// triangle 0
				let vs0 = b3Mul(scale, b3Vec3(x1, h11, z1));
				let vs1 = b3Mul(scale, b3Vec3(x1, h21, z2));
				let vs2 = b3Mul(scale, b3Vec3(x2, h12, z1));

				let n = b3Normalize(b3Cross(b3Sub(vs1, vs0), b3Sub(vs2, vs0)));

				let separation = b3PlaneSeparation(plane2, vs2);
				let cosAngle = b3Dot(plane2.normal, n);
				if (separation > 0.0 || cosAngle > cos5Deg) {
					flags2 |= b3_concaveEdge1;
				}
				if (separation < 0.0 || cosAngle > cos5Deg) {
					flags2 |= b3_inverseConcaveEdge1;
				}
			}

			if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= flags1 && flags1 <= 255);
			if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= flags2 && flags2 <= 255);

			flags[triangleIndex1] = <u8>flags1;
			flags[triangleIndex2] = <u8>flags2;
		}
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(triangleIndex == triangleCount);

	hf.hash = 0;
	hf.hash = b3Hash64NonZero(b3SerializeHeightField(hf), hf.byteCount);

	return hf;
}

// Decode the four corner vertices of a height field cell into local space.
// Output order matches the index naming used throughout this file:
// corners[0] = (column, row), corners[1] = (column + 1, row),
// corners[2] = (column, row + 1), corners[3] = (column + 1, row + 1).
function b3GetHeightFieldCellCorners(hf: b3HeightFieldData, row: i32, column: i32): FixedArray<b3Vec3, 4> {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= row && row < hf.rowCount - 1 && 0 <= column && column < hf.columnCount - 1);

	let columnCount = hf.columnCount;
	let index11 = row * columnCount + column;
	let index12 = index11 + 1;
	let index21 = (row + 1) * columnCount + column;
	let index22 = index21 + 1;

	let minHeight = hf.minHeight;
	let heightScale = hf.heightScale;
	let heights = hf.compressedHeights;

	let height11 = minHeight + heightScale * <f32>heights[index11];
	let height12 = minHeight + heightScale * <f32>heights[index12];
	let height21 = minHeight + heightScale * <f32>heights[index21];
	let height22 = minHeight + heightScale * <f32>heights[index22];

	let x1 = <f32>column;
	let x2 = <f32>(column + 1);
	let z1 = <f32>row;
	let z2 = <f32>(row + 1);

	let scale = hf.scale;
	let corners: FixedArray<b3Vec3, 4>;
	corners[0] = b3Mul(scale, b3Vec3(x1, height11, z1));
	corners[1] = b3Mul(scale, b3Vec3(x2, height12, z1));
	corners[2] = b3Mul(scale, b3Vec3(x1, height21, z2));
	corners[3] = b3Mul(scale, b3Vec3(x2, height22, z2));
	return corners;
}

export function b3GetHeightFieldTriangle(heightField: b3HeightFieldData, triangleIndex: i32): b3Triangle {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= triangleIndex);
	if (B3_ASSERT_ENABLED) B3_ASSERT(triangleIndex < 2 * (heightField.columnCount - 1) * (heightField.rowCount - 1));

	let triangle: b3Triangle;
	triangle.flags = <i32>heightField.flags[triangleIndex];

	let columnCount = heightField.columnCount;
	let quadIndex = triangleIndex >> 1;
	let row = quadIndex / (columnCount - 1);
	let column = quadIndex - row * (columnCount - 1);

	let index11 = row * columnCount + column;
	let index12 = index11 + 1;
	let index21 = (row + 1) * columnCount + column;
	let index22 = index21 + 1;

	let cellIndex = row * (columnCount - 1) + column;

	if (B3_ASSERT_ENABLED) B3_ASSERT(quadIndex == cellIndex);
	if (B3_ASSERT_ENABLED) B3_ASSERT(heightField.materialIndices[cellIndex] != B3_HEIGHT_FIELD_HOLE);

	let corners = b3GetHeightFieldCellCorners(heightField, row, column);

	if ((triangleIndex & 1) == 0) {
		triangle.vertices[0] = corners[0];
		triangle.vertices[1] = corners[2];
		triangle.vertices[2] = corners[1];
		triangle.i1 = index11;
		triangle.i2 = index21;
		triangle.i3 = index12;
	} else {
		triangle.vertices[0] = corners[3];
		triangle.vertices[1] = corners[1];
		triangle.vertices[2] = corners[2];
		triangle.i1 = index22;
		triangle.i2 = index12;
		triangle.i3 = index21;
	}

	if (heightField.clockwise != 0) {
		let tv = triangle.vertices[1];
		triangle.vertices[1] = triangle.vertices[2];
		triangle.vertices[2] = tv;
		let ti = triangle.i2;
		triangle.i2 = triangle.i3;
		triangle.i3 = ti;

		// Reversing winding swaps edge1 and edge3; edge2 (the diagonal) is preserved.
		let flags = triangle.flags;
		let edge1Bits = flags & (b3_concaveEdge1 | b3_inverseConcaveEdge1);
		let edge3Bits = flags & (b3_concaveEdge3 | b3_inverseConcaveEdge3);
		flags &= ~(b3_concaveEdge1 | b3_concaveEdge3 | b3_inverseConcaveEdge1 | b3_inverseConcaveEdge3);
		flags |= edge1Bits << 2;
		flags |= edge3Bits >> 2;
		triangle.flags = flags;
	}

	return triangle;
}

export function b3GetHeightFieldMaterial(heightField: b3HeightFieldData, triangleIndex: i32): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= triangleIndex);
	if (B3_ASSERT_ENABLED) B3_ASSERT(triangleIndex < 2 * (heightField.columnCount - 1) * (heightField.rowCount - 1));

	let cellIndex = triangleIndex >> 1;
	return <i32>heightField.materialIndices[cellIndex];
}

export function b3ComputeHeightFieldAABB(shape: b3HeightFieldData, transform: b3Transform): b3AABB {
	return b3AABB_Transform(transform, shape.aabb);
}

// C's stack locals of the queries below. Only the ray cast runs another of them, the shape cast,
// so it keeps its own input and the rest share one set.
const s_rayAsCastInput = new b3ShapeCastInput();
const s_rayAsCastPoint = new StaticArray<b3Vec3>(1);
const s_queryTriangle = new StaticArray<b3Vec3>(3);
const s_queryTriangleProxy = new b3ShapeProxy();
const s_querySegment = new StaticArray<b3Vec3>(2);
const s_querySegmentProxy = new b3ShapeProxy();
const s_queryLocalPoints = new StaticArray<b3Vec3>(B3_MAX_SHAPE_CAST_POINTS);
const s_queryLocalProxy = new b3ShapeProxy();
const s_queryDistanceInput = new b3DistanceInput();
const s_queryCastInput = new b3ShapeCastPairInput();

export function b3RayCastHeightField(heightField: b3HeightFieldData, input: b3RayCastInput): b3CastOutput {
	let shapeCastInput = s_rayAsCastInput;
	s_rayAsCastPoint[0] = input.origin;
	shapeCastInput.proxy = b3SetProxy(shapeCastInput.proxy, s_rayAsCastPoint, 1, 0.0);
	shapeCastInput.translation = input.translation;
	shapeCastInput.maxFraction = input.maxFraction;
	shapeCastInput.canEncroach = false;

	return b3ShapeCastHeightField(heightField, shapeCastInput);
}

// todo advance cast to the grid border immediately if it starts outside the row/column range
// todo terminate the cast immediately if it leaves the row/column range
export function b3ShapeCastHeightField(heightField: b3HeightFieldData, input: b3ShapeCastInput): b3CastOutput {
	let shapeBounds = b3MakeAABB(input.proxy.points, input.proxy.count, input.proxy.radius);
	let shapeTranslation = input.translation;
	let scale = heightField.scale;

	// The shape start is the center of the proxy.
	let shapeStart = b3AABB_Center(shapeBounds);
	let shapeDelta = b3MulSV(input.maxFraction, shapeTranslation);
	let shapeEnd = b3Add(shapeStart, shapeDelta);

	let result = b3CastOutput();

	let shapeExtents = b3AABB_Extents(shapeBounds);
	let margin = b3Vec3(B3_MAX_AABB_MARGIN(), B3_MAX_AABB_MARGIN(), B3_MAX_AABB_MARGIN());
	let combinedBounds = b3AABB(
		b3Sub(b3Sub(heightField.aabb.lowerBound, shapeExtents), margin),
		b3Add(b3Add(heightField.aabb.upperBound, shapeExtents), margin),
	);

	let fractions: FixedArray<f32, 2>;
	let intersects = b3RayCastAABB(combinedBounds, shapeStart, shapeEnd, fractions[0], fractions[1]);
	if (intersects == false) {
		return result;
	}
	let minFraction = fractions[0];
	let maxFraction = fractions[1];

	// These are for walking the grid, not the triangle cast.
	// The triangle cast uses the unclamped ray and fraction.
	let clampedStart = b3MulAdd(shapeStart, minFraction, shapeDelta);
	let clampedDelta = b3MulSV(maxFraction - minFraction, shapeDelta);
	let clampedEnd = b3Add(clampedStart, clampedDelta);

	// Preserve the un-shifted center sweep. clampedStart/clampedEnd get pushed out to the
	// leading box corner below to drive the grid DDA, but the swept-volume AABB used to
	// cull cells must stay centered on the actual shape path.
	let centerStart = clampedStart;
	let centerEnd = clampedEnd;

	// The grid traversal starts from the leading shape bounds corner
	let signX: f32;
	let signZ: f32;
	if (shapeTranslation.x >= 0.0) {
		clampedStart.x += shapeExtents.x;
		signX = 1.0;
	} else {
		clampedStart.x -= shapeExtents.x;
		signX = -1.0;
	}

	if (shapeTranslation.z >= 0.0) {
		clampedStart.z += shapeExtents.z;
		signZ = 1.0;
	} else {
		clampedStart.z -= shapeExtents.z;
		signZ = -1.0;
	}

	// Shift the end as well
	clampedEnd = b3Add(clampedStart, clampedDelta);

	// Row and column range for the shape cast
	let columnStart = b3FloatToIntSat(Mathf.floor(clampedStart.x / scale.x));
	let columnEnd = b3FloatToIntSat(Mathf.floor(clampedEnd.x / scale.x));
	let rowStart = b3FloatToIntSat(Mathf.floor(clampedStart.z / scale.z));
	let rowEnd = b3FloatToIntSat(Mathf.floor(clampedEnd.z / scale.z));

	let absClampedDelta = b3Abs(clampedDelta);

	// Precompute increments for row and column traversal.
	// The ray can be slightly tilted yet remain within a single row or column
	// once rasterized.
	let deltaAlphaX: f32;
	let nextFractionX: f32;
	let deltaColumn: i32;

	if (columnStart < columnEnd) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(absClampedDelta.x > 0.0);

		// Going forward on x columns
		deltaAlphaX = scale.x / absClampedDelta.x;
		nextFractionX = (scale.x * <f32>(columnStart + 1) - clampedStart.x) / absClampedDelta.x;
		deltaColumn = 1;
	} else if (columnEnd < columnStart) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(absClampedDelta.x > 0.0);

		// Going backwards on x columns
		deltaAlphaX = scale.x / absClampedDelta.x;
		nextFractionX = (clampedStart.x - scale.x * <f32>columnStart) / absClampedDelta.x;
		deltaColumn = -1;
	} else {
		// Cast stays in a single column
		deltaAlphaX = 0.0;
		nextFractionX = FLT_MAX;
		deltaColumn = 0;
	}

	let deltaAlphaZ: f32;
	let nextFractionZ: f32;
	let deltaRow: i32;

	if (rowStart < rowEnd) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(absClampedDelta.z > 0.0);

		// Going forward on z rows
		deltaAlphaZ = scale.z / absClampedDelta.z;
		nextFractionZ = (scale.z * <f32>(rowStart + 1) - clampedStart.z) / absClampedDelta.z;
		deltaRow = 1;
	} else if (rowEnd < rowStart) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(absClampedDelta.z > 0.0);

		// Going backwards on z rows
		deltaAlphaZ = scale.z / absClampedDelta.z;
		nextFractionZ = (clampedStart.z - scale.z * <f32>rowStart) / absClampedDelta.z;
		deltaRow = -1;
	} else {
		// Cast stays in a single row
		deltaAlphaZ = 0.0;
		nextFractionZ = FLT_MAX;
		deltaRow = 0;
	}

	// Column and row range for 2D projected initial shape bounds
	let boxColumnHead = columnStart;
	let boxRowHead = rowStart;

	let boxColumnTail = b3FloatToIntSat(Mathf.floor((clampedStart.x - 2.0 * signX * shapeExtents.x) / scale.x));
	let boxRowTail = b3FloatToIntSat(Mathf.floor((clampedStart.z - 2.0 * signZ * shapeExtents.z) / scale.z));

	let bestFraction = input.maxFraction;

	// nextFractionX / nextFractionZ advance in units of the clamped sweep
	// [minFraction, maxFraction], but bestFraction is a fraction of the full input
	// translation. Precompute the affine map from clamped space to input space so
	// the loop termination test compares like with like. Otherwise it can exit
	// early and miss a closer hit in a later cell.
	let gridFractionScale = input.maxFraction * (maxFraction - minFraction);
	let gridFractionOffset = input.maxFraction * minFraction;

	let rowCount = heightField.rowCount;
	let columnCount = heightField.columnCount;
	let materialIndices = heightField.materialIndices;

	let triangleVertices = s_queryTriangle;
	let pairInput = s_queryCastInput;
	pairInput.proxyA = b3SetProxy(s_queryTriangleProxy, triangleVertices, 3, 0.0);
	pairInput.proxyB = input.proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input.translation;
	pairInput.canEncroach = input.canEncroach;

	let castBounds = b3AABB(b3Sub(b3Min(centerStart, centerEnd), shapeExtents), b3Add(b3Max(centerStart, centerEnd), shapeExtents));

	// The ray origin is the center of the shape.
	let rayOrigin = b3LoadV(shapeStart);
	let rayTranslation = b3LoadV(shapeTranslation);

	let clockwise = heightField.clockwise != 0;

	while (true) {
		let column1: i32;
		let column2: i32;
		if (boxColumnTail < boxColumnHead) {
			column1 = boxColumnTail;
			column2 = boxColumnHead;
		} else {
			column1 = boxColumnHead;
			column2 = boxColumnTail;
		}

		let row1: i32;
		let row2: i32;
		if (boxRowTail < boxRowHead) {
			row1 = boxRowTail;
			row2 = boxRowHead;
		} else {
			row1 = boxRowHead;
			row2 = boxRowTail;
		}

		for (let row = row1; row <= row2; ++row) {
			if (row < 0 || rowCount - 1 <= row) {
				continue;
			}

			for (let column = column1; column <= column2; ++column) {
				if (column < 0 || columnCount - 1 <= column) {
					continue;
				}

				let cellIndex = row * (columnCount - 1) + column;

				let materialIndex = materialIndices[cellIndex];
				if (materialIndex == B3_HEIGHT_FIELD_HOLE) {
					continue;
				}

				let corners = b3GetHeightFieldCellCorners(heightField, row, column);
				let point11 = corners[0];
				let point12 = corners[1];
				let point21 = corners[2];
				let point22 = corners[3];

				if (clockwise) {
					let t = point12;
					point12 = point21;
					point21 = t;
				}

				// I know the min/max x and z values, but not the min/max heights.
				let bounds = b3AABB(b3Min(b3Min(point11, point12), b3Min(point21, point22)), b3Max(b3Max(point11, point12), b3Max(point21, point22)));

				if (b3AABB_Overlaps(castBounds, bounds) == false) {
					continue;
				}

				let quadIndex = row * (columnCount - 1) + column;
				let triangleIndex1 = 2 * quadIndex;
				let triangleIndex2 = triangleIndex1 + 1;

				if (input.proxy.count == 1 && input.proxy.radius == 0.0) {
					// Ray cast
					{
						let vertex1 = b3LoadV(point11);
						let vertex2 = b3LoadV(point21);
						let vertex3 = b3LoadV(point12);

						let alpha = b3IntersectRayTriangle(rayOrigin, rayTranslation, vertex1, vertex2, vertex3);
						if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= alpha && alpha <= 1.0);

						if (alpha < bestFraction) {
							let edge1 = b3Sub(point21, point11);
							let edge2 = b3Sub(point12, point11);
							let normal = b3Cross(edge1, edge2);

							result.point = b3MulAdd(shapeStart, alpha, shapeTranslation);
							result.normal = b3Normalize(normal);
							result.fraction = alpha;
							result.triangleIndex = triangleIndex1;
							result.materialIndex = <i32>materialIndex;
							result.hit = true;
							bestFraction = alpha;
						}
					}

					{
						let vertex1 = b3LoadV(point22);
						let vertex2 = b3LoadV(point12);
						let vertex3 = b3LoadV(point21);

						let alpha = b3IntersectRayTriangle(rayOrigin, rayTranslation, vertex1, vertex2, vertex3);
						if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= alpha && alpha <= 1.0);

						if (alpha < bestFraction) {
							let edge1 = b3Sub(point22, point21);
							let edge2 = b3Sub(point12, point21);
							let normal = b3Cross(edge1, edge2);

							result.point = b3MulAdd(shapeStart, alpha, shapeTranslation);
							result.normal = b3Normalize(normal);
							result.fraction = alpha;
							result.triangleIndex = triangleIndex2;
							result.materialIndex = <i32>materialIndex;
							result.hit = true;
							bestFraction = alpha;
						}
					}
				} else {
					// Shape cast
					{
						// shapeStart is the center of the proxy.
						let signedVolume = b3SignedVolume(point11, point21, point12, shapeStart);

						if (signedVolume >= 0.0) {
							// Shift origin to first vertex
							let origin = point11;
							triangleVertices[0] = b3Vec3_zero;
							triangleVertices[1] = b3Sub(point21, origin);
							triangleVertices[2] = b3Sub(point12, origin);
							pairInput.maxFraction = bestFraction;
							pairInput.transform.p = b3Neg(origin);

							let pairOutput = b3ShapeCast(pairInput);

							if (pairOutput.hit) {
								bestFraction = pairOutput.fraction;
								result = pairOutput;
								result.point = b3Add(result.point, origin);
								result.triangleIndex = triangleIndex1;
								result.materialIndex = <i32>materialIndex;
							}
						}
					}

					{
						// shapeStart is the center of the proxy.
						let signedVolume = b3SignedVolume(point21, point22, point12, shapeStart);

						if (signedVolume >= 0.0) {
							// Shift origin to first vertex
							let origin = point21;
							triangleVertices[0] = b3Vec3_zero;
							triangleVertices[1] = b3Sub(point22, origin);
							triangleVertices[2] = b3Sub(point12, origin);
							pairInput.maxFraction = bestFraction;
							pairInput.transform.p = b3Neg(origin);

							let pairOutput = b3ShapeCast(pairInput);

							if (pairOutput.hit) {
								bestFraction = pairOutput.fraction;
								result = pairOutput;
								result.point = b3Add(result.point, origin);
								result.triangleIndex = triangleIndex2;
								result.materialIndex = <i32>materialIndex;
							}
						}
					}
				}
			}
		}

		// These fractions always increase to guarantee the loop eventually exits.
		// Map them from clamped-sweep space into input-translation space before
		// comparing against bestFraction.
		let inputFractionX = nextFractionX == FLT_MAX ? FLT_MAX : gridFractionOffset + nextFractionX * gridFractionScale;
		let inputFractionZ = nextFractionZ == FLT_MAX ? FLT_MAX : gridFractionOffset + nextFractionZ * gridFractionScale;
		if (inputFractionX > bestFraction && inputFractionZ > bestFraction) {
			break;
		}

		// Advance the cast to the next column or row
		if (nextFractionX <= nextFractionZ) {
			if (boxColumnHead == columnEnd) {
				// Hit the end already
				break;
			}

			// Advance to next column
			boxColumnHead += deltaColumn;

			// Build a single column to cast
			boxColumnTail = boxColumnHead;

			if (shapeExtents.z == 0.0) {
				// Single row
				boxRowTail = boxRowHead;
			} else {
				// Rasterize shape row
				let rowIntercept = clampedStart.z + nextFractionX * clampedDelta.z;
				boxRowTail = b3FloatToIntSat(Mathf.floor((rowIntercept - 2.0 * signZ * shapeExtents.z) / scale.z));
			}

			nextFractionX += deltaAlphaX;
		} else {
			if (boxRowHead == rowEnd) {
				// Hit the end already
				break;
			}

			// Advance to next row
			boxRowHead += deltaRow;

			// Build a single row to cast
			boxRowTail = boxRowHead;

			if (shapeExtents.x == 0.0) {
				// Single column
				boxColumnTail = boxColumnHead;
			} else {
				// Rasterize shape column
				let columnIntercept = clampedStart.x + nextFractionZ * clampedDelta.x;
				boxColumnTail = b3FloatToIntSat(Mathf.floor((columnIntercept - 2.0 * signX * shapeExtents.x) / scale.x));
			}

			nextFractionZ += deltaAlphaZ;
		}
	}

	return result;
}

// src/shape.c, until lib/shape.as exists
function b3ComputeProxyAABB(proxy: b3ShapeProxy): b3AABB {
	let points = proxy.points;
	let aabb = b3AABB(points[0], points[0]);

	for (let i = 1; i < proxy.count; ++i) {
		aabb.lowerBound = b3Min(aabb.lowerBound, points[i]);
		aabb.upperBound = b3Max(aabb.upperBound, points[i]);
	}

	let r = b3Vec3(proxy.radius, proxy.radius, proxy.radius);
	aabb.lowerBound = b3Sub(aabb.lowerBound, r);
	aabb.upperBound = b3Add(aabb.upperBound, r);
	return aabb;
}

export function b3OverlapHeightField(shape: b3HeightFieldData, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool {
	let localProxy = b3SetLocalProxy(s_queryLocalProxy, proxy, shapeTransform, s_queryLocalPoints);
	let aabb = b3ComputeProxyAABB(localProxy);

	let scale = shape.scale;
	let minRow = b3FloatToIntSat(Mathf.floor(aabb.lowerBound.z / scale.z));
	let maxRow = b3FloatToIntSat(Mathf.floor(aabb.upperBound.z / scale.z));
	let minCol = b3FloatToIntSat(Mathf.floor(aabb.lowerBound.x / scale.x));
	let maxCol = b3FloatToIntSat(Mathf.floor(aabb.upperBound.x / scale.x));

	let boundsMin = b3LoadV(aabb.lowerBound);
	let boundsMax = b3LoadV(aabb.upperBound);
	let boundsCenter = b3MulV(b3_halfV, b3AddV(boundsMin, boundsMax));
	let boundsExtent = b3SubV(boundsMax, boundsCenter);

	let triangleVertices = s_queryTriangle;
	let input = s_queryDistanceInput;
	input.proxyA = b3SetProxy(s_queryTriangleProxy, triangleVertices, 3, 0.0);
	input.proxyB = localProxy;
	input.transform = b3Transform_identity;
	input.useRadii = true;

	let cache = b3SimplexCache();
	let materialIndices = shape.materialIndices;

	// Outer loop on rows and inner loop on columns so that triangle indices increase monotonically.
	for (let row = minRow; row <= maxRow; ++row) {
		if (row < 0 || shape.rowCount - 1 <= row) {
			continue;
		}

		for (let column = minCol; column <= maxCol; ++column) {
			if (column < 0 || shape.columnCount - 1 <= column) {
				continue;
			}

			let cellIndex = row * (shape.columnCount - 1) + column;
			let material = materialIndices[cellIndex];
			if (material == B3_HEIGHT_FIELD_HOLE) {
				continue;
			}

			let corners = b3GetHeightFieldCellCorners(shape, row, column);
			let point11 = corners[0];
			let point12 = corners[1];
			let point21 = corners[2];
			let point22 = corners[3];

			let v11 = b3LoadV(point11);
			let v12 = b3LoadV(point12);
			let v21 = b3LoadV(point21);
			let v22 = b3LoadV(point22);

			if (b3TestBoundsTriangleOverlap(boundsCenter, boundsExtent, v11, v21, v12)) {
				triangleVertices[0] = point11;
				triangleVertices[1] = point21;
				triangleVertices[2] = point12;

				// reset the cache
				cache.count = 0;

				// get distance between triangle and query shape
				let output = b3ShapeDistance(input, cache, null, 0);

				let tolerance: f32 = 0.1 * B3_LINEAR_SLOP();
				if (output.distance < tolerance) {
					// overlap detected
					return true;
				}
			}

			if (b3TestBoundsTriangleOverlap(boundsCenter, boundsExtent, v21, v22, v12)) {
				triangleVertices[0] = point22;
				triangleVertices[1] = point12;
				triangleVertices[2] = point21;

				// reset the cache
				cache.count = 0;

				// get distance between triangle and query shape
				let output = b3ShapeDistance(input, cache, null, 0);

				let tolerance: f32 = 0.1 * B3_LINEAR_SLOP();
				if (output.distance < tolerance) {
					// overlap detected
					return true;
				}
			}
		}
	}

	return false;
}

export function b3QueryHeightField(heightField: b3HeightFieldData, bounds: b3AABB, fcn: b3MeshQueryFcn): void {
	let scale = heightField.scale;

	let minRow = b3FloatToIntSat(Mathf.floor(bounds.lowerBound.z / scale.z));
	let maxRow = b3FloatToIntSat(Mathf.floor(bounds.upperBound.z / scale.z));
	let minCol = b3FloatToIntSat(Mathf.floor(bounds.lowerBound.x / scale.x));
	let maxCol = b3FloatToIntSat(Mathf.floor(bounds.upperBound.x / scale.x));

	let materialIndices = heightField.materialIndices;

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for (let row = minRow; row <= maxRow; ++row) {
		if (row < 0 || heightField.rowCount - 1 <= row) {
			continue;
		}

		for (let column = minCol; column <= maxCol; ++column) {
			if (column < 0 || heightField.columnCount - 1 <= column) {
				continue;
			}

			let cellIndex = row * (heightField.columnCount - 1) + column;
			let material = materialIndices[cellIndex];
			if (material == B3_HEIGHT_FIELD_HOLE) {
				continue;
			}

			let corners = b3GetHeightFieldCellCorners(heightField, row, column);
			let point11 = corners[0];
			let point12 = corners[1];
			let point21 = corners[2];
			let point22 = corners[3];

			// I know the min/max x and z values, but not the min/max heights.
			// This could be done with no branching in SIMD.
			let cellBound = b3AABB(b3Min(b3Min(point11, point12), b3Min(point21, point22)), b3Max(b3Max(point11, point12), b3Max(point21, point22)));

			if (b3AABB_Overlaps(bounds, cellBound)) {
				let quadIndex = row * (heightField.columnCount - 1) + column;
				let triangleIndex = 2 * quadIndex;

				if (heightField.clockwise != 0) {
					fcn(point11, point12, point21, triangleIndex);
					fcn(point22, point21, point12, triangleIndex + 1);
				} else {
					fcn(point11, point21, point12, triangleIndex);
					fcn(point22, point12, point21, triangleIndex + 1);
				}
			}
		}
	}
}

// C's planes pointer is planes from index planeOffset; capacity counts from there.
export function b3CollideMoverAndHeightField(
	planes: StaticArray<b3PlaneResult>,
	planeOffset: i32,
	capacity: i32,
	shape: b3HeightFieldData,
	mover: b3Capsule,
): i32 {
	let triangleVertices = s_queryTriangle;
	let distanceInput = s_queryDistanceInput;
	distanceInput.proxyA = b3SetProxy(s_queryTriangleProxy, triangleVertices, 3, 0.0);
	s_querySegment[0] = mover.center1;
	s_querySegment[1] = mover.center2;
	distanceInput.proxyB = b3SetProxy(s_querySegmentProxy, s_querySegment, 2, 0.0);
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	let cache = b3SimplexCache();

	let radius = mover.radius;
	let center = b3Lerp(mover.center1, mover.center2, 0.5);
	let center1 = b3LoadV(mover.center1);
	let center2 = b3LoadV(mover.center2);
	let r = b3SplatV(radius);
	let boundsMin = b3SubV(b3MinV(center1, center2), r);
	let boundsMax = b3AddV(b3MaxV(center1, center2), r);
	let boundsCenter = b3MulV(b3_halfV, b3AddV(boundsMin, boundsMax));
	let boundsExtent = b3SubV(boundsMax, boundsCenter);

	let localMinX = b3GetXV(boundsMin);
	let localMinZ = b3GetZV(boundsMin);
	let localMaxX = b3GetXV(boundsMax);
	let localMaxZ = b3GetZV(boundsMax);

	let scale = shape.scale;
	let minRow = b3FloatToIntSat(Mathf.floor(localMinZ / scale.z));
	let maxRow = b3FloatToIntSat(Mathf.floor(localMaxZ / scale.z));
	let minCol = b3FloatToIntSat(Mathf.floor(localMinX / scale.x));
	let maxCol = b3FloatToIntSat(Mathf.floor(localMaxX / scale.x));

	let clockWise = shape.clockwise != 0;
	let materialIndices = shape.materialIndices;

	let planeCount = 0;

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for (let row = minRow; row <= maxRow; ++row) {
		if (row < 0 || shape.rowCount - 1 <= row) {
			continue;
		}

		for (let column = minCol; column <= maxCol; ++column) {
			if (column < 0 || shape.columnCount - 1 <= column) {
				continue;
			}

			let cellIndex = row * (shape.columnCount - 1) + column;
			let material = materialIndices[cellIndex];
			if (material == B3_HEIGHT_FIELD_HOLE) {
				continue;
			}

			let corners = b3GetHeightFieldCellCorners(shape, row, column);
			let point11 = corners[0];
			let point12 = corners[1];
			let point21 = corners[2];
			let point22 = corners[3];

			if (clockWise) {
				let t = point12;
				point12 = point21;
				point21 = t;
			}

			let v11 = b3LoadV(point11);
			let v12 = b3LoadV(point12);
			let v21 = b3LoadV(point21);
			let v22 = b3LoadV(point22);

			let overlap1 = b3TestBoundsTriangleOverlap(boundsCenter, boundsExtent, v11, v21, v12);
			if (overlap1) {
				let signedVolume = b3SignedVolume(point11, point21, point12, center);

				// Front side?
				if (signedVolume >= 0.0) {
					triangleVertices[0] = point11;
					triangleVertices[1] = point21;
					triangleVertices[2] = point12;

					// reset the cache
					cache.count = 0;

					// get distance between triangle and mover
					let distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0);

					if (distanceOutput.distance == 0.0) {
						// deep overlap
					} else if (distanceOutput.distance <= mover.radius) {
						let triangleIndex = 2 * cellIndex;
						ref plane = planes[planeOffset + planeCount];
						plane.plane = b3Plane(distanceOutput.normal, mover.radius - distanceOutput.distance);
						plane.point = distanceOutput.pointA;
						plane.triangleIndex = triangleIndex;
						plane.childIndex = 0;
						plane.materialIndex = <i32>material;
						planeCount += 1;

						if (planeCount == capacity) {
							return planeCount;
						}
					}
				}
			}

			let overlap2 = b3TestBoundsTriangleOverlap(boundsCenter, boundsExtent, v21, v22, v12);
			if (overlap2) {
				let signedVolume = b3SignedVolume(point22, point12, point21, center);

				// Front side?
				if (signedVolume >= 0.0) {
					triangleVertices[0] = point22;
					triangleVertices[1] = point12;
					triangleVertices[2] = point21;

					// reset the cache
					cache.count = 0;

					// get distance between triangle and mover
					let distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0);

					if (distanceOutput.distance == 0.0) {
						// deep overlap
					} else if (distanceOutput.distance <= mover.radius) {
						let triangleIndex = 2 * cellIndex + 1;
						ref plane = planes[planeOffset + planeCount];
						plane.plane = b3Plane(distanceOutput.normal, mover.radius - distanceOutput.distance);
						plane.point = distanceOutput.pointA;
						plane.triangleIndex = triangleIndex;
						plane.childIndex = 0;
						plane.materialIndex = <i32>material;
						planeCount += 1;

						if (planeCount == capacity) {
							return planeCount;
						}
					}
				}
			}
		}
	}

	return planeCount;
}

// The generators share the material layout: every 16th cell a hole when asked
function b3MakeGridMaterials(rowCount: i32, columnCount: i32, makeHoles: bool): StaticArray<u8> {
	let cellCount = (rowCount - 1) * (columnCount - 1);
	let materialIndices = new StaticArray<u8>(cellCount);

	for (let i = 0; i < rowCount - 1; ++i) {
		for (let j = 0; j < columnCount - 1; ++j) {
			let k = i * (columnCount - 1) + j;

			if (makeHoles && k > 0 && k % 16 == 0) {
				materialIndices[k] = <u8>B3_HEIGHT_FIELD_HOLE;
			} else {
				materialIndices[k] = 0;
			}
		}
	}

	return materialIndices;
}

export function b3CreateGrid(rowCount: i32, columnCount: i32, scale: b3Vec3, makeHoles: bool): b3HeightFieldData {
	let heightCount = rowCount * columnCount;
	let heights = new StaticArray<f32>(heightCount);

	let data = new b3HeightFieldDef();
	data.heights = heights;
	data.materialIndices = b3MakeGridMaterials(rowCount, columnCount, makeHoles);
	data.scale = scale;
	data.countX = columnCount;
	data.countZ = rowCount;
	data.globalMinimumHeight = -256.0;
	data.globalMaximumHeight = 256.0;
	data.clockwiseWinding = false;

	return b3CreateHeightField(data);
}

export function b3CreateWave(
	rowCount: i32,
	columnCount: i32,
	scale: b3Vec3,
	rowFrequency: f32,
	columnFrequency: f32,
	makeHoles: bool,
): b3HeightFieldData {
	let heightCount = rowCount * columnCount;
	let heights = new StaticArray<f32>(heightCount);

	let omegaZ: f32 = 2.0 * B3_PI * rowFrequency;
	let omegaX: f32 = 2.0 * B3_PI * columnFrequency;

	for (let i = 0; i < rowCount; ++i) {
		let cs = b3ComputeCosSin(omegaZ * <f32>i);
		let rowHeight = cs.sine;

		for (let j = 0; j < columnCount; ++j) {
			let k = i * columnCount + j;
			cs = b3ComputeCosSin(omegaX * <f32>j);
			let columnHeight = cs.sine;
			heights[k] = rowHeight * columnHeight;
		}
	}

	let data = new b3HeightFieldDef();
	data.heights = heights;
	data.materialIndices = b3MakeGridMaterials(rowCount, columnCount, makeHoles);
	data.scale = scale;
	data.countX = columnCount;
	data.countZ = rowCount;
	data.globalMinimumHeight = -256.0;
	data.globalMaximumHeight = 256.0;
	data.clockwiseWinding = false;

	return b3CreateHeightField(data);
}

export function b3DestroyHeightField(heightField: b3HeightFieldData): void {}
