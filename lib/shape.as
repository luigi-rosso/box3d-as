// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/shape.c

import { B3_ASSERT, B3_ASSERT_ENABLED } from "./core";
import { b3AABB, b3Mul, b3Vec3 } from "./math_functions";
import { b3Triangle } from "./math_internal";
import {
	b3Capsule,
	b3CompoundData,
	b3GetMeshFlags,
	b3GetMeshTriangles,
	b3GetMeshVertices,
	b3HeightFieldData,
	b3HullData,
	b3Mesh,
	b3Sphere,
	b3_concaveEdge1,
	b3_concaveEdge2,
	b3_concaveEdge3,
	b3_inverseConcaveEdge1,
	b3_inverseConcaveEdge2,
	b3_inverseConcaveEdge3,
} from "./collision";
import {
	b3Filter,
	b3QueryFilter,
	b3ShapeType,
	b3SurfaceMaterial,
	b3_capsuleShape,
	b3_hullShape,
	b3_sphereShape,
} from "./types";

export type b3ShapeFlags = u8;
export const b3_enableSensorEvents: b3ShapeFlags = 0x01;
export const b3_enableContactEvents: b3ShapeFlags = 0x02;
export const b3_enableCustomFiltering: b3ShapeFlags = 0x04;
export const b3_enableHitEvents: b3ShapeFlags = 0x08;
export const b3_enablePreSolveEvents: b3ShapeFlags = 0x10;
// 0x20 is free, it was the enlarged AABB flag before the broad-phase marked the tree
export const b3_enableSpeculative: b3ShapeFlags = 0x40;

const b3_emptyMaterials = new StaticArray<b3SurfaceMaterial>(0);

export class b3Shape {
	id: i32 = 0;
	bodyId: i32 = 0;
	prevShapeId: i32 = 0;
	nextShapeId: i32 = 0;
	sensorIndex: i32 = 0;
	proxyKey: i32 = 0;
	type: b3ShapeType = 0;
	density: f32 = 0;
	explosionScale: f32 = 0;
	aabbMargin: f32 = 0;

	aabb: b3AABB = b3AABB();
	localCentroid: b3Vec3 = b3Vec3();

	// C keeps a single material inline and a heap array only for several. Here materials is
	// always the array, length 1 for the inline case, so writes through b3GetShapeMaterials land.
	materialCount: i32 = 0;
	materials: StaticArray<b3SurfaceMaterial> = b3_emptyMaterials;

	filter: b3Filter = b3Filter();
	userData: u64 = 0;
	userShape: u64 = 0;

	nameId: u32 = 0;
	generation: u16 = 0;

	// b3ShapeFlags
	flags: u8 = 0;

	// The C union becomes one field per member; they do not alias.
	capsule: b3Capsule = b3Capsule();
	sphere: b3Sphere = b3Sphere();
	hull: b3HullData? = null;
	mesh: b3Mesh? = null;
	heightField: b3HeightFieldData? = null;
	compound: b3CompoundData? = null;
}

// C copies a b3Shape by value into a temporary; this is that memcpy. Arrays and geometry alias.
export function b3CopyShape(shape: b3Shape): b3Shape {
	let copy = new b3Shape();
	copy.id = shape.id;
	copy.bodyId = shape.bodyId;
	copy.prevShapeId = shape.prevShapeId;
	copy.nextShapeId = shape.nextShapeId;
	copy.sensorIndex = shape.sensorIndex;
	copy.proxyKey = shape.proxyKey;
	copy.type = shape.type;
	copy.density = shape.density;
	copy.explosionScale = shape.explosionScale;
	copy.aabbMargin = shape.aabbMargin;
	copy.aabb = shape.aabb;
	copy.localCentroid = shape.localCentroid;
	copy.materialCount = shape.materialCount;
	copy.materials = shape.materials;
	copy.filter = shape.filter;
	copy.userData = shape.userData;
	copy.userShape = shape.userShape;
	copy.nameId = shape.nameId;
	copy.generation = shape.generation;
	copy.flags = shape.flags;
	copy.capsule = shape.capsule;
	copy.sphere = shape.sphere;
	copy.hull = shape.hull;
	copy.mesh = shape.mesh;
	copy.heightField = shape.heightField;
	copy.compound = shape.compound;
	return copy;
}

// A single material shape keeps its material inline. Multi material meshes and compounds own a heap
// array. Reach the materials the same way for both: a single material shape presents its inline
// material as a one element array.
@inline export function b3GetShapeMaterials(shape: b3Shape): StaticArray<b3SurfaceMaterial> {
	return shape.materials;
}

// Height field

@inline export function b3GetHeightFieldTriangleCount(heightField: b3HeightFieldData): i32 {
	let cellCount = (heightField.rowCount - 1) * (heightField.columnCount - 1);
	return 2 * cellCount;
}

// Mesh

// Making this inline made the tree benchmarks faster.
export function b3GetMeshTriangle(mesh: b3Mesh, triangleIndex: i32): b3Triangle {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= triangleIndex && triangleIndex < mesh.data.triangleCount);

	let triangles = b3GetMeshTriangles(mesh.data);
	let flags = b3GetMeshFlags(mesh.data);
	let vertices = b3GetMeshVertices(mesh.data);

	let result: b3Triangle;
	let triangle = triangles[triangleIndex];
	let triangleFlags: u8 = flags[triangleIndex];

	let scale = mesh.scale;

	result.vertices[0] = b3Mul(scale, vertices[triangle.index1]);
	result.i1 = triangle.index1;

	if (scale.x * scale.y * scale.z < 0.0) {
		result.vertices[1] = b3Mul(scale, vertices[triangle.index3]);
		result.vertices[2] = b3Mul(scale, vertices[triangle.index2]);

		result.i2 = triangle.index3;
		result.i3 = triangle.index2;

		// mesh is inverted, so concave edges are now convex
		result.flags = 0;
		result.flags |= (triangleFlags & b3_inverseConcaveEdge1) != 0 ? b3_concaveEdge1 : 0;
		result.flags |= (triangleFlags & b3_inverseConcaveEdge2) != 0 ? b3_concaveEdge2 : 0;
		result.flags |= (triangleFlags & b3_inverseConcaveEdge3) != 0 ? b3_concaveEdge3 : 0;
	} else {
		result.vertices[1] = b3Mul(scale, vertices[triangle.index2]);
		result.vertices[2] = b3Mul(scale, vertices[triangle.index3]);

		result.i2 = triangle.index2;
		result.i3 = triangle.index3;
		result.flags = triangleFlags;
	}

	return result;
}

@inline export function b3ShouldShapesCollide(filterA: b3Filter, filterB: b3Filter): bool {
	if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
		return filterA.groupIndex > 0;
	}

	return (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
}

@inline export function b3ShouldQueryCollide(shapeFilter: b3Filter, queryFilter: b3QueryFilter): bool {
	return (shapeFilter.categoryBits & queryFilter.maskBits) != 0 && (shapeFilter.maskBits & queryFilter.categoryBits) != 0;
}

@inline export function b3IsConvex(type: b3ShapeType): bool {
	return type == b3_sphereShape || type == b3_capsuleShape || type == b3_hullShape;
}

// functions from shape.c follow
