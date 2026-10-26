// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/shape.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, B3_SECRET_COOKIE, FLT_EPSILON, b3GetLengthUnitsPerMeter, b3InParallel, b3OnMain } from "./core";
import {
	B3_AABB_MARGIN_FRACTION,
	B3_LINEAR_SLOP,
	B3_MAX_AABB_MARGIN,
	B3_MAX_SHAPES,
	B3_MAX_SHAPE_CAST_POINTS,
	B3_MAX_WORKERS,
	B3_SPECULATIVE_DISTANCE,
} from "./constants";
import {
	B3_PI,
	b3AABB,
	b3AABB_Center,
	b3AABB_Inflate,
	b3AABB_Transform,
	b3Abs,
	b3Add,
	b3ClampInt,
	b3Cross,
	b3Distance,
	b3DistanceSquared,
	b3Dot,
	b3GetLengthAndNormalize,
	b3InvMulTransforms,
	b3InvRotateVector,
	b3InvTransformPoint,
	b3InvertTransform,
	b3IsValidFloat,
	b3IsValidPosition,
	b3IsValidVec3,
	b3Length,
	b3Lerp,
	b3MakeMatrixFromQuat,
	b3Max,
	b3MaxFloat,
	b3Min,
	b3MinFloat,
	b3MinInt,
	b3Mul,
	b3MulAdd,
	b3MulMV,
	b3MulSV,
	b3MulSub,
	b3MulTransforms,
	b3Normalize,
	b3OffsetPos,
	b3Pos,
	b3Pos_zero,
	b3RotateVector,
	b3SafeScale,
	b3Sub,
	b3ToRelativeTransform,
	b3Transform,
	b3TransformPoint,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_one,
	b3Vec3_zero,
	b3WorldTransform,
} from "./math_functions";
import { b3ShapeExtent, b3Triangle } from "./math_internal";
import {
	B3_COMPOUND_VERSION,
	B3_HEIGHT_FIELD_VERSION,
	B3_HULL_VERSION,
	B3_MESH_VERSION,
	b3Capsule,
	b3CastOutput,
	b3ChildShape,
	b3CompoundData,
	b3DistanceInput,
	b3GetHullEdges,
	b3GetHullFaces,
	b3GetHullPlanes,
	b3GetHullPoints,
	b3GetMeshFlags,
	b3GetMeshMaterialIndices,
	b3GetMeshTriangles,
	b3GetMeshVertices,
	b3HeightFieldData,
	b3HullData,
	b3MassData,
	b3Mesh,
	b3MeshData,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeProxy,
	b3SimplexCache,
	b3Sphere,
	b3Sweep,
	b3TOIInput,
	b3TOIOutput,
	b3WorldCastOutput,
	b3_concaveEdge1,
	b3_concaveEdge2,
	b3_concaveEdge3,
	b3_inverseConcaveEdge1,
	b3_inverseConcaveEdge2,
	b3_inverseConcaveEdge3,
} from "./collision";
import {
	b3BodyId,
	b3BodyType,
	b3ContactData,
	b3ContactId,
	b3Filter,
	b3QueryFilter,
	b3SensorEndTouchEvent,
	b3ShapeDef,
	b3ShapeId,
	b3ShapeType,
	b3SurfaceMaterial,
	b3WorldId,
	b3_bodyTypeCount,
	b3_capsuleShape,
	b3_compoundShape,
	b3_dynamicBody,
	b3_heightShape,
	b3_hullShape,
	b3_meshShape,
	b3_nullShapeId,
	b3_sphereShape,
	b3_staticBody,
	B3_DEFAULT_MASK_BITS,
} from "./types";
import {
	b3AddHullToDatabase,
	b3AddOwnedHullToDatabase,
	b3GetUnlockedWorld,
	b3GetWorld,
	b3RemoveHullFromDatabase,
	b3World,
	b3_awakeSet,
	b3_disabledSet,
	b3_firstSleepingSet,
} from "./physics_world";
import { b3AddName, b3FindNameWithDefault } from "./name_cache";
import {
	b3GetBodyFullId,
	b3GetBodySim,
	b3GetBodyState,
	b3GetBodyTransform,
	b3GetBodyTransformQuick,
	b3MakeBodyId,
	b3SyncBodyFlags,
	b3UpdateBodyMassData,
	b3WakeBodyWithLock,
	b3_dirtyMass,
} from "./body";
import {
	B3_PROXY_TYPE,
	b3BroadPhase,
	b3BroadPhase_CreateProxy,
	b3BroadPhase_DestroyProxy,
	b3BroadPhase_MoveProxy,
} from "./broad_phase";
import { b3DestroyContact, b3_contactTouchingFlag } from "./contact";
import { b3Sensor, b3Visitor } from "./sensor";
import { b3FarthestPointOnAABB } from "./aabb";
import { b3AllocId, b3FreeId } from "./id_pool";
import { b3Array_RemoveSwap } from "./container";
import { b3CopyProxy, b3GetSweepTransform, b3MakeOwningProxy, b3SetProxy, b3ShapeDistance, b3TimeOfImpactAt } from "./distance";
import { b3ComputeCapsuleAABB, b3ComputeCapsuleMass, b3ComputeSweptCapsuleAABB, b3CollideMoverAndCapsule, b3OverlapCapsule, b3RayCastCapsule, b3ShapeCastCapsule } from "./capsule";
import { b3ComputeSphereAABB, b3ComputeSphereMass, b3ComputeSweptSphereAABB, b3CollideMoverAndSphere, b3OverlapSphere, b3RayCastSphere, b3ShapeCastSphere } from "./sphere";
import {
	b3CloneAndTransformHull,
	b3CollideMoverAndHull,
	b3ComputeHullAABB,
	b3ComputeHullExtent,
	b3ComputeHullMass,
	b3ComputeHullProjectedArea,
	b3ComputeSweptHullAABB,
	b3OverlapHull,
	b3RayCastHull,
	b3ShapeCastHull,
} from "./hull";
import { B3_TREE_STACK_SIZE, b3QueryNodesVisitor } from "./dynamic_tree";
import {
	B3_MESH_STACK_SIZE,
	b3CollideMoverAndMesh,
	b3ComputeMeshAABB,
	b3IsValidMesh,
	b3OverlapMesh,
	b3QueryMesh,
	b3QueryMeshVisitor,
	b3RayCastMesh,
	b3ShapeCastMesh,
} from "./mesh";
import {
	b3CollideMoverAndHeightField,
	b3ComputeHeightFieldAABB,
	b3GetHeightFieldMaterial,
	b3OverlapHeightField,
	b3QueryHeightField,
	b3QueryHeightFieldVisitor,
	b3RayCastHeightField,
	b3ShapeCastHeightField,
} from "./height_field";
import {
	b3CollideMoverAndCompound,
	b3ComputeCompoundAABB,
	b3GetCompoundChild,
	b3GetCompoundChildInto,
	b3GetCompoundMaterials,
	b3MakeCompoundChildSweep,
	b3OverlapCompound,
	b3QueryCompound,
	b3RayCastCompound,
	b3ShapeCastCompound,
} from "./compound";

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

	// A compound's children as shapes for parallel parts, built for compoundChildData, b3PrepareCompoundContacts
	compoundChildData: b3CompoundData? = null;
	compoundChildren: StaticArray<b3ChildShape>? = null;
	compoundChildShapes: StaticArray<b3Shape>? = null;
	compoundChildMaterials: StaticArray<StaticArray<b3SurfaceMaterial>>? = null;
}

// C copies a b3Shape by value into a temporary; this is that memcpy. Arrays and geometry alias.
export function b3CopyShape(shape: b3Shape): b3Shape {
	let copy = new b3Shape();
	b3CopyShapeInto(copy, shape);
	return copy;
}

export function b3CopyShapeInto(copy: b3Shape, shape: b3Shape): void {
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

function b3GetShape(world: b3World, shapeId: b3ShapeId): b3Shape {
	let id = shapeId.index1 - 1;
	let shape = world.shapes[id];
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.id == id && shape.generation == shapeId.generation);
	return shape;
}

function b3ComputeShapeMargin(shape: b3Shape): f32 {
	let margin: f32 = 0.0;

	switch (shape.type) {
		case b3_sphereShape: {
			margin = shape.sphere.radius;
			break;
		}

		case b3_capsuleShape: {
			margin = 0.5 * b3Distance(shape.capsule.center2, shape.capsule.center1) + shape.capsule.radius;
			break;
		}

		case b3_hullShape: {
			let hull = shape.hull!;
			let points = b3GetHullPoints(hull);
			let maxExtentSqr: f32 = 0.0;
			let count = hull.vertexCount;
			for (let i = 0; i < count; ++i) {
				let distSqr = b3DistanceSquared(points[i], hull.center);
				maxExtentSqr = b3MaxFloat(maxExtentSqr, distSqr);
			}
			margin = Mathf.sqrt(maxExtentSqr);
			break;
		}

		case b3_meshShape:
		case b3_heightShape:
		case b3_compoundShape: {
			// Static-only shapes: broadphase uses speculative distance for static
			// proxies, so the per-shape margin is never consumed in practice.
			// Return the cap so any incidental use is generous.
			return B3_MAX_AABB_MARGIN();
		}

		default:
			return B3_MAX_AABB_MARGIN();
	}

	return b3MinFloat(B3_MAX_AABB_MARGIN(), B3_AABB_MARGIN_FRACTION * margin);
}

// C writes the fat AABB through a pointer; this returns it for the caller to store.
function b3UpdateShapeAABBs(shape: b3Shape, transform: b3WorldTransform, proxyType: b3BodyType): b3AABB {
	// Compute a bounding box with a speculative margin
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();
	let aabbMargin = shape.aabbMargin;

	let aabb = b3ComputeFatShapeAABB(shape, transform, speculativeDistance);
	shape.aabb = aabb;

	// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
	let margin = proxyType == b3_staticBody ? speculativeDistance : aabbMargin;
	return b3AABB_Inflate(aabb, margin);
}

// C passes the geometry as a void pointer plus the shape type.
class b3ShapeGeometry {
	capsule: b3Capsule = b3Capsule();
	sphere: b3Sphere = b3Sphere();
	hull: b3HullData? = null;
	mesh: b3MeshData? = null;
	heightField: b3HeightFieldData? = null;
	compound: b3CompoundData? = null;
}

function b3CreateShapeInternal(
	world: b3World,
	bodyId: i32,
	bodyTransform: b3WorldTransform,
	def: b3ShapeDef,
	geometry: b3ShapeGeometry,
	shapeType: b3ShapeType,
	shapeTransform: b3Transform,
	scale: b3Vec3,
	haveShapeTransform: bool,
): b3Shape? {
	let shapeId = b3AllocId(world.shapeIdPool);

	if (shapeId == world.shapes.length) {
		world.shapes.push(new b3Shape());
		world.fatAABBs.push(b3AABB());
	} else {
		if (B3_ASSERT_ENABLED) B3_ASSERT(world.shapes[shapeId].id == B3_NULL_INDEX);
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(world.fatAABBs.length == world.shapes.length);

	let shape = world.shapes[shapeId];

	switch (shapeType) {
		case b3_capsuleShape:
			shape.capsule = geometry.capsule;
			break;

		case b3_compoundShape:
			// Compounds must be a static and not a sensor
			if (B3_ASSERT_ENABLED) B3_ASSERT(world.bodies[bodyId].type == b3_staticBody);
			if (B3_ASSERT_ENABLED) B3_ASSERT(def.isSensor == false);
			shape.compound = geometry.compound;
			break;

		case b3_sphereShape:
			shape.sphere = geometry.sphere;
			break;

		case b3_hullShape:
			if (haveShapeTransform) {
				// The transform and non-uniform scale are baked into fresh data, then shared.
				let baked = b3CloneAndTransformHull(geometry.hull, shapeTransform, scale);
				if (baked == null) {
					// This can fail to produce a valid hull in extreme cases
					b3FreeId(world.shapeIdPool, shapeId);
					shape.id = B3_NULL_INDEX;
					return null;
				}

				shape.hull = b3AddOwnedHullToDatabase(world, baked);
			} else {
				shape.hull = b3AddHullToDatabase(world, geometry.hull!);
			}
			break;

		case b3_meshShape: {
			shape.mesh = new b3Mesh(geometry.mesh!, b3SafeScale(scale));
			break;
		}

		case b3_heightShape:
			shape.heightField = geometry.heightField;
			break;

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	shape.id = shapeId;
	shape.bodyId = world.bodies[bodyId].id;
	shape.type = shapeType;
	shape.density = def.density;
	shape.explosionScale = def.explosionScale;
	shape.filter = def.filter;
	shape.userData = def.userData;
	shape.userShape = 0;
	shape.flags = 0;
	shape.flags |= def.enableSensorEvents ? b3_enableSensorEvents : 0;
	shape.flags |= def.enableContactEvents ? b3_enableContactEvents : 0;
	shape.flags |= def.enableCustomFiltering ? b3_enableCustomFiltering : 0;
	shape.flags |= def.enableHitEvents ? b3_enableHitEvents : 0;
	shape.flags |= def.enablePreSolveEvents ? b3_enablePreSolveEvents : 0;
	shape.flags |= def.enableSpeculativeContact ? b3_enableSpeculative : 0;
	shape.proxyKey = B3_NULL_INDEX;
	shape.localCentroid = b3GetShapeCentroid(shape);
	shape.aabbMargin = b3ComputeShapeMargin(shape);
	shape.aabb = b3AABB(b3Vec3_zero, b3Vec3_zero);
	world.fatAABBs[shapeId] = b3AABB(b3Vec3_zero, b3Vec3_zero);
	shape.nameId = b3AddName(world.names, def.name);
	shape.generation += 1;

	if (shape.type == b3_compoundShape) {
		world.compoundShapeCount += 1;

		// Own a copy of the compound materials so every shape frees its array the same way. Compounds
		// are few, so the copy is cheap and avoids aliasing the geometry blob.
		let compound = shape.compound!;
		let materialCount = compound.materialCount;
		shape.materialCount = materialCount;
		// C's zero byte b3Alloc returns NULL and falls back to the inline material
		shape.materials = new StaticArray<b3SurfaceMaterial>(materialCount > 0 ? materialCount : 1);
		let compoundMaterials = b3GetCompoundMaterials(compound);
		for (let i = 0; i < materialCount; ++i) {
			shape.materials[i] = compoundMaterials![i];
		}
	} else if (def.materialCount > 1 && def.materials != null) {
		// Per triangle materials need a heap array.
		let defMaterials = def.materials!;
		shape.materialCount = def.materialCount;
		shape.materials = new StaticArray<b3SurfaceMaterial>(def.materialCount);
		for (let i = 0; i < def.materialCount; ++i) {
			shape.materials[i] = defMaterials[i];
		}
	} else {
		// The common case is one material, stored inline with no allocation.
		let defMaterials = def.materials;
		shape.materials = new StaticArray<b3SurfaceMaterial>(1);
		shape.materials[0] = def.materialCount == 1 && defMaterials != null ? defMaterials[0] : def.baseMaterial;
		shape.materialCount = 1;
	}

	if (world.bodies[bodyId].setIndex != b3_disabledSet) {
		let proxyType = world.bodies[bodyId].type;
		let forcePairCreation = def.invokeContactCreation && shape.type != b3_compoundShape;
		b3CreateShapeProxy(world, shape, proxyType, bodyTransform, forcePairCreation);
	}

	// Add to shape doubly linked list
	let headShapeId = world.bodies[bodyId].headShapeId;
	if (headShapeId != B3_NULL_INDEX) {
		let headShape = world.shapes[headShapeId];
		headShape.prevShapeId = shapeId;
	}

	shape.prevShapeId = B3_NULL_INDEX;
	shape.nextShapeId = headShapeId;
	world.bodies[bodyId].headShapeId = shapeId;
	world.bodies[bodyId].shapeCount += 1;

	if (def.isSensor) {
		shape.sensorIndex = world.sensors.length;
		let sensor = new b3Sensor();
		world.sensors.push(sensor);
		sensor.shapeId = shapeId;
	} else {
		shape.sensorIndex = B3_NULL_INDEX;
	}

	return shape;
}

function b3CreateShape(
	bodyId: b3BodyId,
	def: b3ShapeDef,
	geometry: b3ShapeGeometry,
	shapeType: b3ShapeType,
	transform: b3Transform,
	scale: b3Vec3,
	haveTransform: bool,
): b3ShapeId {
	if (B3_ASSERT_ENABLED) {
		B3_ASSERT(def.internalValue == B3_SECRET_COOKIE);
		B3_ASSERT(b3IsValidFloat(def.density) && def.density >= 0.0);
		B3_ASSERT(b3IsValidFloat(def.baseMaterial.friction) && def.baseMaterial.friction >= 0.0);
		B3_ASSERT(b3IsValidFloat(def.baseMaterial.restitution) && def.baseMaterial.restitution >= 0.0);
	}

	let world = b3GetUnlockedWorld(bodyId.world0);
	if (world == null) {
		return b3ShapeId();
	}

	if (world.shapes.length == B3_MAX_SHAPES && world.shapeIdPool.freeArray.length == 0) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(false);
		return b3_nullShapeId;
	}

	let body = b3GetBodyFullId(world, bodyId);
	if (world.bodies[body].type != b3_staticBody && (shapeType == b3_compoundShape || shapeType == b3_heightShape)) {
		// Compound and height shapes must be on static bodies.
		return b3_nullShapeId;
	}

	world.locked = true;

	let bodyTransform = b3GetBodyTransformQuick(world, body);

	let shape = b3CreateShapeInternal(world, body, bodyTransform, def, geometry, shapeType, transform, scale, haveTransform);

	if (shape == null) {
		world.locked = false;
		return b3_nullShapeId;
	}

	if (def.updateBodyMass == true) {
		b3UpdateBodyMassData(world, body);
	} else if ((world.bodies[body].flags & b3_dirtyMass) == 0) {
		world.bodies[body].flags |= b3_dirtyMass;
		b3SyncBodyFlags(world, body);
	}

	let id = b3MakeShapeIdFrom(shape.id + 1, bodyId.world0, shape.generation);

	world.locked = false;

	return id;
}

export function b3CreateSphereShape(bodyId: b3BodyId, def: b3ShapeDef, sphere: b3Sphere): b3ShapeId {
	let geometry = new b3ShapeGeometry();
	geometry.sphere = sphere;
	return b3CreateShape(bodyId, def, geometry, b3_sphereShape, b3Transform_identity, b3Vec3_one, false);
}

export function b3CreateCapsuleShape(bodyId: b3BodyId, def: b3ShapeDef, capsule: b3Capsule): b3ShapeId {
	let lengthSqr = b3DistanceSquared(capsule.center1, capsule.center2);
	let linearSlop = B3_LINEAR_SLOP();
	let shapeId: b3ShapeId;
	let geometry = new b3ShapeGeometry();
	if (lengthSqr <= linearSlop * linearSlop) {
		geometry.sphere = b3Sphere(b3Lerp(capsule.center1, capsule.center2, 0.5), capsule.radius);
		shapeId = b3CreateShape(bodyId, def, geometry, b3_sphereShape, b3Transform_identity, b3Vec3_one, false);
	} else {
		geometry.capsule = capsule;
		shapeId = b3CreateShape(bodyId, def, geometry, b3_capsuleShape, b3Transform_identity, b3Vec3_one, false);
	}
	return shapeId;
}

export function b3CreateHullShape(bodyId: b3BodyId, def: b3ShapeDef, hull: b3HullData): b3ShapeId {
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull.version == B3_HULL_VERSION);
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull.hash != 0);

	if (hull.version != B3_HULL_VERSION) {
		return b3_nullShapeId;
	}

	let geometry = new b3ShapeGeometry();
	geometry.hull = hull;
	return b3CreateShape(bodyId, def, geometry, b3_hullShape, b3Transform_identity, b3Vec3_one, false);
}

export function b3CreateTransformedHullShape(
	bodyId: b3BodyId,
	def: b3ShapeDef,
	hull: b3HullData,
	transform: b3Transform,
	scale: b3Vec3,
): b3ShapeId {
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull.version == B3_HULL_VERSION);
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull.hash != 0);

	if (hull.version != B3_HULL_VERSION) {
		return b3_nullShapeId;
	}

	let geometry = new b3ShapeGeometry();
	geometry.hull = hull;
	return b3CreateShape(bodyId, def, geometry, b3_hullShape, transform, scale, true);
}

export function b3CreateMeshShape(bodyId: b3BodyId, def: b3ShapeDef, mesh: b3MeshData, scale: b3Vec3): b3ShapeId {
	if (B3_ASSERT_ENABLED) B3_ASSERT(mesh.version == B3_MESH_VERSION);
	if (B3_ASSERT_ENABLED) B3_ASSERT(mesh.hash != 0);

	if (mesh.version != B3_MESH_VERSION) {
		return b3_nullShapeId;
	}

	let geometry = new b3ShapeGeometry();
	geometry.mesh = mesh;
	return b3CreateShape(bodyId, def, geometry, b3_meshShape, b3Transform_identity, scale, true);
}

export function b3CreateHeightFieldShape(bodyId: b3BodyId, def: b3ShapeDef, heightField: b3HeightFieldData): b3ShapeId {
	if (B3_ASSERT_ENABLED) B3_ASSERT(heightField.version == B3_HEIGHT_FIELD_VERSION);
	if (B3_ASSERT_ENABLED) B3_ASSERT(heightField.hash != 0);

	if (heightField.version != B3_HEIGHT_FIELD_VERSION) {
		return b3_nullShapeId;
	}

	let geometry = new b3ShapeGeometry();
	geometry.heightField = heightField;
	return b3CreateShape(bodyId, def, geometry, b3_heightShape, b3Transform_identity, b3Vec3_one, false);
}

export function b3CreateBakedCompoundShape(bodyId: b3BodyId, def: b3ShapeDef, compound: b3CompoundData): b3ShapeId {
	if (B3_ASSERT_ENABLED) B3_ASSERT(compound.version == B3_COMPOUND_VERSION);

	if (compound.version != B3_COMPOUND_VERSION) {
		return b3_nullShapeId;
	}

	let geometry = new b3ShapeGeometry();
	geometry.compound = compound;
	return b3CreateShape(bodyId, def, geometry, b3_compoundShape, b3Transform_identity, b3Vec3_one, false);
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
function b3DestroyShapeInternal(world: b3World, shape: b3Shape, bodyId: i32, wakeBodies: bool): void {
	let shapeId = shape.id;

	if (shape.type == b3_compoundShape) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(world.compoundShapeCount > 0);
		world.compoundShapeCount -= 1;
	}

	// Remove the shape from the body's doubly linked list.
	if (shape.prevShapeId != B3_NULL_INDEX) {
		let prevShape = world.shapes[shape.prevShapeId];
		prevShape.nextShapeId = shape.nextShapeId;
	}

	if (shape.nextShapeId != B3_NULL_INDEX) {
		let nextShape = world.shapes[shape.nextShapeId];
		nextShape.prevShapeId = shape.prevShapeId;
	}

	if (shapeId == world.bodies[bodyId].headShapeId) {
		world.bodies[bodyId].headShapeId = shape.nextShapeId;
	}

	world.bodies[bodyId].shapeCount -= 1;

	// Remove from broad-phase.
	b3DestroyShapeProxy(shape, world.broadPhase);

	// Destroy any contacts associated with the shape.
	let contactKey = world.bodies[bodyId].headContactKey;
	while (contactKey != B3_NULL_INDEX) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let contact = world.contacts[contactId];
		contactKey = contact.edges[edgeIndex].nextKey;

		if (contact.shapeIdA == shapeId || contact.shapeIdB == shapeId) {
			b3DestroyContact(world, contact, wakeBodies);
		}
	}

	if (shape.sensorIndex != B3_NULL_INDEX) {
		let sensor = world.sensors[shape.sensorIndex];
		for (let i = 0; i < sensor.overlaps2.length; ++i) {
			let ref = sensor.overlaps2[i];
			let event = b3SensorEndTouchEvent();
			event.sensorShapeId = b3MakeShapeIdFrom(shapeId + 1, world.worldId, shape.generation);
			event.visitorShapeId = b3MakeShapeIdFrom(ref.shapeId + 1, world.worldId, ref.generation);

			world.sensorEndEvents[world.endEventArrayIndex].push(event);
		}

		// Destroy sensor
		sensor.hits = new Array<b3Visitor>();
		sensor.overlaps1 = new Array<b3Visitor>();
		sensor.overlaps2 = new Array<b3Visitor>();

		let movedIndex = b3Array_RemoveSwap(world.sensors, shape.sensorIndex);
		if (movedIndex != B3_NULL_INDEX) {
			// Fixup moved sensor
			let movedSensor = world.sensors[shape.sensorIndex];
			let otherSensorShape = world.shapes[movedSensor.shapeId];
			otherSensorShape.sensorIndex = shape.sensorIndex;
		}
	}

	// Destroy every shape member from b3Alloc
	b3DestroyShapeAllocations(world, shape);

	// Return shape to free list.
	b3FreeId(world.shapeIdPool, shapeId);
	shape.id = B3_NULL_INDEX;
}

export function b3DestroyShape(shapeId: b3ShapeId, updateBodyMass: bool): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;

	let shape = b3GetShape(world, shapeId);

	// need to wake bodies because this might be a static body
	let wakeBodies = true;

	let body = shape.bodyId;
	b3DestroyShapeInternal(world, shape, body, wakeBodies);

	if (updateBodyMass == true) {
		b3UpdateBodyMassData(world, body);
	}

	world.locked = false;
}

export function b3ComputeShapeAABB(shape: b3Shape, transform: b3Transform): b3AABB {
	switch (shape.type) {
		case b3_capsuleShape:
			return b3ComputeCapsuleAABB(shape.capsule, transform);

		case b3_compoundShape:
			return b3ComputeCompoundAABB(shape.compound!, transform);

		case b3_heightShape:
			return b3ComputeHeightFieldAABB(shape.heightField!, transform);

		case b3_hullShape:
			return b3ComputeHullAABB(shape.hull!, transform);

		case b3_meshShape: {
			let mesh = shape.mesh!;
			return b3ComputeMeshAABB(mesh.data, transform, mesh.scale);
		}

		case b3_sphereShape:
			return b3ComputeSphereAABB(shape.sphere, transform);

		default: {
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			let empty = b3AABB(transform.p, transform.p);
			return empty;
		}
	}
}

export function b3ComputeFatShapeAABB(shape: b3Shape, transform: b3WorldTransform, extra: f32): b3AABB {
	let r = b3Vec3(extra, extra, extra);
	let aabb = b3ComputeShapeAABB(shape, transform);
	aabb.lowerBound = b3Sub(aabb.lowerBound, r);
	aabb.upperBound = b3Add(aabb.upperBound, r);
	return aabb;
}

export function b3ComputeSweptShapeAABB(shape: b3Shape, sweep: b3Sweep, time: f32): b3AABB {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0.0 <= time && time <= 1.0);
	let xf1 = b3Transform(b3Sub(sweep.c1, b3RotateVector(sweep.q1, sweep.localCenter)), sweep.q1);
	let xf2 = b3GetSweepTransform(sweep, time);

	switch (shape.type) {
		case b3_capsuleShape:
			return b3ComputeSweptCapsuleAABB(shape.capsule, xf1, xf2);

		case b3_hullShape:
			return b3ComputeSweptHullAABB(shape.hull!, xf1, xf2);

		case b3_sphereShape:
			return b3ComputeSweptSphereAABB(shape.sphere, xf1, xf2);

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			return b3AABB(xf1.p, xf1.p);
	}
}

export function b3GetShapeCentroid(shape: b3Shape): b3Vec3 {
	switch (shape.type) {
		case b3_capsuleShape:
			return b3Lerp(shape.capsule.center1, shape.capsule.center2, 0.5);
		case b3_compoundShape: {
			let aabb = b3ComputeCompoundAABB(shape.compound!, b3Transform_identity);
			return b3AABB_Center(aabb);
		}
		case b3_sphereShape:
			return shape.sphere.center;
		case b3_hullShape:
			return shape.hull!.center;
		case b3_meshShape: {
			let mesh = shape.mesh!;
			let aabb = b3ComputeMeshAABB(mesh.data, b3Transform_identity, mesh.scale);
			return b3AABB_Center(aabb);
		}
		case b3_heightShape: {
			let aabb = b3ComputeHeightFieldAABB(shape.heightField!, b3Transform_identity);
			return b3AABB_Center(aabb);
		}
		default:
			return b3Vec3_zero;
	}
}

export function b3GetShapeArea(shape: b3Shape): f32 {
	// todo_erin fix these
	switch (shape.type) {
		case b3_capsuleShape:
			return 2.0 * b3Length(b3Sub(shape.capsule.center1, shape.capsule.center2)) + 2.0 * B3_PI * shape.capsule.radius;

		case b3_hullShape:
			return shape.hull!.surfaceArea;

		case b3_sphereShape:
			return 2.0 * B3_PI * shape.sphere.radius;

		default:
			return 0.0;
	}
}

// This projects the shape surface area onto a plane
export function b3GetShapeProjectedArea(shape: b3Shape, planeNormal: b3Vec3): f32 {
	switch (shape.type) {
		case b3_capsuleShape: {
			let radius = shape.capsule.radius;
			let axis = b3Sub(shape.capsule.center2, shape.capsule.center1);
			let projectedLength = b3Length(b3Cross(axis, planeNormal));
			let cylinderArea: f32 = 2.0 * radius * projectedLength;
			let sphereArea: f32 = B3_PI * radius * radius;
			return sphereArea + cylinderArea;
		}

		case b3_hullShape:
			return b3ComputeHullProjectedArea(shape.hull!, planeNormal);

		case b3_sphereShape:
			return B3_PI * shape.sphere.radius * shape.sphere.radius;

		default:
			return 0.0;
	}
}

export function b3ComputeShapeMass(shape: b3Shape): b3MassData {
	switch (shape.type) {
		case b3_capsuleShape:
			return b3ComputeCapsuleMass(shape.capsule, shape.density);

		case b3_hullShape:
			return b3ComputeHullMass(shape.hull!, shape.density);

		case b3_sphereShape:
			return b3ComputeSphereMass(shape.sphere, shape.density);

		default:
			return b3MassData();
	}
}

export function b3ComputeShapeExtent(shape: b3Shape, localCenter: b3Vec3): b3ShapeExtent {
	let extent = b3ShapeExtent();

	switch (shape.type) {
		case b3_capsuleShape: {
			let radius = shape.capsule.radius;
			extent.minExtent = radius;
			let c1 = b3Sub(shape.capsule.center1, localCenter);
			let c2 = b3Sub(shape.capsule.center2, localCenter);
			let r = b3Vec3(radius, radius, radius);
			extent.maxExtent = b3Add(b3Max(b3Abs(c1), b3Abs(c2)), r);
			break;
		}

		case b3_compoundShape: {
			// This is shouldn't be needed but here for completeness
			let aabb = b3ComputeCompoundAABB(shape.compound!, b3Transform_identity);
			let r1 = b3Length(b3Sub(aabb.lowerBound, localCenter));
			let r2 = b3Length(b3Sub(aabb.upperBound, localCenter));
			extent.minExtent = b3MinFloat(r1, r2);
			let p = b3FarthestPointOnAABB(aabb, localCenter);
			extent.maxExtent = b3Abs(b3Sub(p, localCenter));
			break;
		}

		case b3_sphereShape: {
			let radius = shape.sphere.radius;
			extent.minExtent = radius;
			let h = b3Abs(b3Sub(shape.sphere.center, localCenter));
			let r = b3Vec3(radius, radius, radius);
			extent.maxExtent = b3Add(h, r);
			break;
		}

		case b3_hullShape:
			extent = b3ComputeHullExtent(shape.hull!, localCenter);
			break;

		case b3_meshShape: {
			// This is needed for kinematic mesh sleeping
			let mesh = shape.mesh!;
			let aabb = b3ComputeMeshAABB(mesh.data, b3Transform_identity, mesh.scale);
			let r1 = b3Length(b3Sub(aabb.lowerBound, localCenter));
			let r2 = b3Length(b3Sub(aabb.upperBound, localCenter));
			extent.minExtent = b3MinFloat(r1, r2);
			let p = b3FarthestPointOnAABB(aabb, localCenter);
			extent.maxExtent = b3Abs(b3Sub(p, localCenter));
			break;
		}

		default:
			break;
	}

	return extent;
}

export function b3ComputeShapeMinExtent(shape: b3Shape, localCenter: b3Vec3): f32 {
	switch (shape.type) {
		case b3_capsuleShape:
			return shape.capsule.radius;

		case b3_sphereShape:
			return shape.sphere.radius;

		case b3_hullShape:
			return shape.hull!.innerRadius;

		default:
			return b3ComputeShapeExtent(shape, localCenter).minExtent;
	}
}

export function b3RayCastShape(shape: b3Shape, transform: b3Transform, input: b3RayCastInput): b3CastOutput {
	let localInput = input;
	localInput.origin = b3InvTransformPoint(transform, input.origin);
	localInput.translation = b3InvRotateVector(transform.q, input.translation);

	let output = b3CastOutput();
	switch (shape.type) {
		case b3_capsuleShape:
			output = b3RayCastCapsule(shape.capsule, localInput);
			break;
		case b3_compoundShape:
			output = b3RayCastCompound(shape.compound!, localInput);
			break;
		case b3_sphereShape:
			output = b3RayCastSphere(shape.sphere, localInput);
			break;
		case b3_hullShape:
			output = b3RayCastHull(shape.hull!, localInput);
			break;
		case b3_meshShape:
			output = b3RayCastMesh(shape.mesh!, localInput);
			break;
		case b3_heightShape:
			output = b3RayCastHeightField(shape.heightField!, localInput);
			break;
		default:
			return output;
	}

	output.point = b3TransformPoint(transform, output.point);
	output.normal = b3RotateVector(transform.q, output.normal);
	return output;
}

// C's stack locals of b3ShapeCastShape, which never nests
const s_localCastInput = new b3ShapeCastInput();
const s_localCastPoints = new StaticArray<b3Vec3>(B3_MAX_SHAPE_CAST_POINTS);

export function b3ShapeCastShape(shape: b3Shape, transform: b3Transform, input: b3ShapeCastInput): b3CastOutput {
	let localInput = s_localCastInput;
	let count = b3MinInt(input.proxy.count, B3_MAX_SHAPE_CAST_POINTS);
	let localPoints = s_localCastPoints;
	for (let i = 0; i < count; ++i) {
		localPoints[i] = b3InvTransformPoint(transform, input.proxy.points[i]);
	}

	localInput.proxy = b3SetProxy(localInput.proxy, localPoints, count, input.proxy.radius);
	localInput.translation = b3InvRotateVector(transform.q, input.translation);
	localInput.maxFraction = input.maxFraction;
	localInput.canEncroach = input.canEncroach;

	let output = b3CastOutput();
	switch (shape.type) {
		case b3_capsuleShape:
			output = b3ShapeCastCapsule(shape.capsule, localInput);
			break;

		case b3_compoundShape:
			output = b3ShapeCastCompound(shape.compound!, localInput);
			break;

		case b3_heightShape:
			output = b3ShapeCastHeightField(shape.heightField!, localInput);
			break;

		case b3_hullShape:
			output = b3ShapeCastHull(shape.hull!, localInput);
			break;

		case b3_meshShape:
			output = b3ShapeCastMesh(shape.mesh!, localInput);
			break;

		case b3_sphereShape:
			output = b3ShapeCastSphere(shape.sphere, localInput);
			break;
		default:
			return output;
	}

	output.point = b3TransformPoint(transform, output.point);
	output.normal = b3RotateVector(transform.q, output.normal);
	return output;
}

export function b3OverlapShape(shape: b3Shape, transform: b3Transform, proxy: b3ShapeProxy): bool {
	let type = shape.type;
	switch (type) {
		case b3_capsuleShape:
			return b3OverlapCapsule(shape.capsule, transform, proxy);

		case b3_compoundShape:
			return b3OverlapCompound(shape.compound!, transform, proxy);

		case b3_heightShape:
			return b3OverlapHeightField(shape.heightField!, transform, proxy);

		case b3_hullShape:
			return b3OverlapHull(shape.hull!, transform, proxy);

		case b3_meshShape:
			return b3OverlapMesh(shape.mesh!, transform, proxy);

		case b3_sphereShape:
			return b3OverlapSphere(shape.sphere, transform, proxy);

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			return false;
	}
}

export function b3CollideMover(
	planes: StaticArray<b3PlaneResult>,
	planeCapacity: i32,
	shape: b3Shape,
	transform: b3Transform,
	mover: b3Capsule,
): i32 {
	if (planeCapacity == 0) {
		return 0;
	}

	let localMover: b3Capsule;
	localMover.center1 = b3InvTransformPoint(transform, mover.center1);
	localMover.center2 = b3InvTransformPoint(transform, mover.center2);
	localMover.radius = mover.radius;

	let planeCount = 0;
	switch (shape.type) {
		case b3_capsuleShape:
			planeCount = b3CollideMoverAndCapsule(planes[0], shape.capsule, localMover);
			break;

		case b3_compoundShape:
			planeCount = b3CollideMoverAndCompound(planes, planeCapacity, shape.compound!, localMover);
			break;

		case b3_sphereShape:
			planeCount = b3CollideMoverAndSphere(planes[0], shape.sphere, localMover);
			break;

		case b3_hullShape:
			planeCount = b3CollideMoverAndHull(planes[0], shape.hull!, localMover);
			break;

		case b3_meshShape:
			planeCount = b3CollideMoverAndMesh(planes, 0, planeCapacity, shape.mesh!, localMover);
			break;

		case b3_heightShape:
			planeCount = b3CollideMoverAndHeightField(planes, 0, planeCapacity, shape.heightField!, localMover);
			break;

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	for (let i = 0; i < planeCount; ++i) {
		planes[i].plane.normal = b3RotateVector(transform.q, planes[i].plane.normal);
		planes[i].point = b3TransformPoint(transform, planes[i].point);
		planes[i].materialIndex = b3ClampInt(planes[i].materialIndex, 0, shape.materialCount - 1);
	}

	return planeCount;
}

export function b3CreateShapeProxy(
	world: b3World,
	shape: b3Shape,
	type: b3BodyType,
	transform: b3WorldTransform,
	forcePairCreation: bool,
): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.proxyKey == B3_NULL_INDEX);

	let fatAABB = b3UpdateShapeAABBs(shape, transform, type);
	world.fatAABBs[shape.id] = fatAABB;

	// Create proxies in the broad-phase.
	shape.proxyKey = b3BroadPhase_CreateProxy(world.broadPhase, type, fatAABB, shape.filter.categoryBits, shape.id, forcePairCreation);
	if (B3_ASSERT_ENABLED) B3_ASSERT(B3_PROXY_TYPE(shape.proxyKey) < b3_bodyTypeCount);
}

export function b3DestroyShapeProxy(shape: b3Shape, bp: b3BroadPhase): void {
	if (shape.proxyKey != B3_NULL_INDEX) {
		b3BroadPhase_DestroyProxy(bp, shape.proxyKey);
		shape.proxyKey = B3_NULL_INDEX;
	}
}

function b3DestroyShapeAllocationForShapeChange(world: b3World, shape: b3Shape): void {
	let type = shape.type;
	switch (type) {
		case b3_hullShape:
			b3RemoveHullFromDatabase(world, shape.hull!);
			shape.hull = null;
			break;

		default:
			break;
	}

	// The debug shape destroy callback is skipped with debug draw; userShape is never set.
	if (shape.userShape != 0) {
		shape.userShape = 0;
	}
}

export function b3DestroyShapeAllocations(world: b3World, shape: b3Shape): void {
	b3DestroyShapeAllocationForShapeChange(world, shape);

	// C's materials != NULL: a compound or a shape created with several materials.
	if (shape.type == b3_compoundShape || shape.materialCount > 1) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(shape.materialCount > 0);
		shape.materials = b3_emptyMaterials;
		shape.materialCount = 0;
	}

	// Name is stored inline. Sensor data is destroyed elsewhere
}

// b3MakeShapeProxy into a caller owned proxy; points holds a capsule's or a sphere's centers
export function b3SetShapeProxy(proxy: b3ShapeProxy, shape: b3Shape, points: StaticArray<b3Vec3>): b3ShapeProxy {
	switch (shape.type) {
		case b3_capsuleShape:
			points[0] = shape.capsule.center1;
			points[1] = shape.capsule.center2;
			return b3SetProxy(proxy, points, 2, shape.capsule.radius);

		case b3_sphereShape:
			points[0] = shape.sphere.center;
			return b3SetProxy(proxy, points, 1, shape.sphere.radius);

		case b3_hullShape: {
			let hull = shape.hull!;
			return b3SetProxy(proxy, b3GetHullPoints(hull), hull.vertexCount, 0.0);
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			return b3SetProxy(proxy, b3_noProxyPoints, 0, 0.0);
	}
}

const b3_noProxyPoints = new StaticArray<b3Vec3>(0);

// b3SetShapeProxy into a proxy that owns its points, for parallel parts
function b3CopyShapeProxy(proxy: b3ShapeProxy, shape: b3Shape): b3ShapeProxy {
	let points = proxy.points;
	switch (shape.type) {
		case b3_capsuleShape:
			points[0] = shape.capsule.center1;
			points[1] = shape.capsule.center2;
			proxy.count = 2;
			proxy.radius = shape.capsule.radius;
			return proxy;

		case b3_sphereShape:
			points[0] = shape.sphere.center;
			proxy.count = 1;
			proxy.radius = shape.sphere.radius;
			return proxy;

		case b3_hullShape: {
			let hull = shape.hull!;
			return b3CopyProxy(proxy, b3GetHullPoints(hull), hull.vertexCount, 0.0);
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			proxy.count = 0;
			proxy.radius = 0.0;
			return proxy;
	}
}

export function b3MakeShapeProxy(shape: b3Shape): b3ShapeProxy {
	switch (shape.type) {
		case b3_capsuleShape: {
			let points = new StaticArray<b3Vec3>(2);
			points[0] = shape.capsule.center1;
			points[1] = shape.capsule.center2;
			return new b3ShapeProxy(points, 2, shape.capsule.radius);
		}

		case b3_sphereShape: {
			let points = new StaticArray<b3Vec3>(1);
			points[0] = shape.sphere.center;
			return new b3ShapeProxy(points, 1, shape.sphere.radius);
		}

		case b3_hullShape: {
			let hull = shape.hull!;
			let points = b3GetHullPoints(hull);
			return new b3ShapeProxy(points, hull.vertexCount, 0.0);
		}

		default: {
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			return new b3ShapeProxy();
		}
	}
}

export function b3MakeLocalProxy(proxy: b3ShapeProxy, transform: b3Transform, buffer: StaticArray<b3Vec3>): b3ShapeProxy {
	return b3SetLocalProxy(new b3ShapeProxy(), proxy, transform, buffer);
}

// b3MakeLocalProxy into a caller owned proxy
export function b3SetLocalProxy(
	local: b3ShapeProxy,
	proxy: b3ShapeProxy,
	transform: b3Transform,
	buffer: StaticArray<b3Vec3>,
): b3ShapeProxy {
	let invTransform = b3InvertTransform(transform);
	let R = b3MakeMatrixFromQuat(invTransform.q);

	let count = b3MinInt(proxy.count, B3_MAX_SHAPE_CAST_POINTS);
	for (let i = 0; i < count; ++i) {
		buffer[i] = b3Add(b3MulMV(R, proxy.points[i]), invTransform.p);
	}

	return b3SetProxy(local, buffer, count, proxy.radius);
}

export function b3ComputeProxyAABB(proxy: b3ShapeProxy): b3AABB {
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

export function b3Shape_GetBody(shapeId: b3ShapeId): b3BodyId {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return b3MakeBodyId(world, shape.bodyId);
}

export function b3Shape_GetWorld(shapeId: b3ShapeId): b3WorldId {
	let world = b3GetWorld(shapeId.world0);
	let id = b3WorldId();
	id.index1 = <u16>(shapeId.world0 + 1);
	id.generation = world.generation;
	return id;
}

export function b3Shape_SetUserData(shapeId: b3ShapeId, userData: u64): void {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	shape.userData = userData;
}

export function b3Shape_GetUserData(shapeId: b3ShapeId): u64 {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return shape.userData;
}

export function b3Shape_SetName(shapeId: b3ShapeId, name: string): void {
	let world = b3GetWorld(shapeId.world0);

	let shape = b3GetShape(world, shapeId);
	shape.nameId = b3AddName(world.names, name);
}

export function b3Shape_GetName(shapeId: b3ShapeId): string {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return b3FindNameWithDefault(world.names, shape.nameId, "");
}

export function b3Shape_IsSensor(shapeId: b3ShapeId): bool {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return shape.sensorIndex != B3_NULL_INDEX;
}

// todo no tests
export function b3Shape_RayCast(shapeId: b3ShapeId, origin: b3Pos, translation: b3Vec3): b3WorldCastOutput {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidPosition(origin));
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(translation));

	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);

	// Re-center on the origin so the cast runs in float precision far from the world origin
	let transform = b3ToRelativeTransform(b3GetBodyTransform(world, shape.bodyId), origin);

	// The ray starts at the origin, so its origin in the re-centered frame is zero
	let input = b3RayCastInput(b3Vec3_zero, translation, 1.0);

	// Lift the re-centered float result back to a world position
	let local = b3RayCastShape(shape, transform, input);
	let output: b3WorldCastOutput;
	output.normal = local.normal;
	output.point = b3OffsetPos(origin, local.point);
	output.fraction = local.fraction;
	output.iterations = local.iterations;
	output.triangleIndex = local.triangleIndex;
	output.childIndex = local.childIndex;
	output.materialIndex = local.materialIndex;
	output.hit = local.hit;

	return output;
}

export function b3Shape_SetDensity(shapeId: b3ShapeId, density: f32, updateBodyMass: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(density) && density >= 0.0);

	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b3GetShape(world, shapeId);
	if (density == shape.density) {
		// early return to avoid expensive function
		return;
	}

	shape.density = density;

	if (updateBodyMass == true) {
		b3UpdateBodyMassData(world, shape.bodyId);
	}
}

export function b3Shape_GetDensity(shapeId: b3ShapeId): f32 {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return shape.density;
}

export function b3Shape_SetFriction(shapeId: b3ShapeId, friction: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(friction) && friction >= 0.0);
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type != b3_compoundShape);
	b3GetShapeMaterials(shape)[0].friction = friction;
}

export function b3Shape_GetFriction(shapeId: b3ShapeId): f32 {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return b3GetShapeMaterials(shape)[0].friction;
}

export function b3Shape_SetRestitution(shapeId: b3ShapeId, restitution: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(restitution) && restitution >= 0.0);
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type != b3_compoundShape);
	b3GetShapeMaterials(shape)[0].restitution = restitution;
}

export function b3Shape_GetRestitution(shapeId: b3ShapeId): f32 {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return b3GetShapeMaterials(shape)[0].restitution;
}

export function b3Shape_SetSurfaceMaterial(shapeId: b3ShapeId, surfaceMaterial: b3SurfaceMaterial): void {
	if (B3_ASSERT_ENABLED) {
		B3_ASSERT(b3IsValidFloat(surfaceMaterial.friction) && surfaceMaterial.friction >= 0.0);
		B3_ASSERT(b3IsValidFloat(surfaceMaterial.restitution) && surfaceMaterial.restitution >= 0.0);
		B3_ASSERT(b3IsValidFloat(surfaceMaterial.rollingResistance) && surfaceMaterial.rollingResistance >= 0.0);
		B3_ASSERT(b3IsValidVec3(surfaceMaterial.tangentVelocity));
	}

	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type != b3_compoundShape);
	let materials = b3GetShapeMaterials(shape);
	materials[0] = surfaceMaterial;
}

export function b3Shape_GetSurfaceMaterial(shapeId: b3ShapeId): b3SurfaceMaterial {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return b3GetShapeMaterials(shape)[0];
}

export function b3Shape_GetMeshMaterialCount(shapeId: b3ShapeId): i32 {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return shape.materialCount;
}

export function b3Shape_SetMeshMaterial(shapeId: b3ShapeId, surfaceMaterial: b3SurfaceMaterial, index: i32): void {
	if (B3_ASSERT_ENABLED) {
		B3_ASSERT(b3IsValidFloat(surfaceMaterial.friction) && surfaceMaterial.friction >= 0.0);
		B3_ASSERT(b3IsValidFloat(surfaceMaterial.restitution) && surfaceMaterial.restitution >= 0.0);
		B3_ASSERT(b3IsValidFloat(surfaceMaterial.rollingResistance) && surfaceMaterial.rollingResistance >= 0.0);
		B3_ASSERT(b3IsValidVec3(surfaceMaterial.tangentVelocity));
	}

	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);

	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < shape.materialCount);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type != b3_compoundShape);

	let materials = b3GetShapeMaterials(shape);
	materials[index] = surfaceMaterial;
}

export function b3Shape_GetMeshSurfaceMaterial(shapeId: b3ShapeId, index: i32): b3SurfaceMaterial {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < shape.materialCount);
	return b3GetShapeMaterials(shape)[index];
}

export function b3Shape_GetFilter(shapeId: b3ShapeId): b3Filter {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return shape.filter;
}

function b3ResetProxy(world: b3World, shape: b3Shape, wakeBodies: bool, destroyProxy: bool): void {
	let body = shape.bodyId;

	let shapeId = shape.id;

	// destroy all contacts associated with this shape
	let contactKey = world.bodies[body].headContactKey;
	while (contactKey != B3_NULL_INDEX) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let contact = world.contacts[contactId];
		contactKey = contact.edges[edgeIndex].nextKey;

		if (contact.shapeIdA == shapeId || contact.shapeIdB == shapeId) {
			b3DestroyContact(world, contact, wakeBodies);
		}
	}

	let transform = b3GetBodyTransformQuick(world, body);
	if (shape.proxyKey != B3_NULL_INDEX) {
		let proxyType = B3_PROXY_TYPE(shape.proxyKey);
		let fatAABB = b3UpdateShapeAABBs(shape, transform, proxyType);
		world.fatAABBs[shapeId] = fatAABB;

		if (destroyProxy) {
			b3BroadPhase_DestroyProxy(world.broadPhase, shape.proxyKey);

			let forcePairCreation = true;
			shape.proxyKey = b3BroadPhase_CreateProxy(
				world.broadPhase,
				proxyType,
				fatAABB,
				shape.filter.categoryBits,
				shapeId,
				forcePairCreation,
			);
		} else {
			b3BroadPhase_MoveProxy(world.broadPhase, shape.proxyKey, fatAABB);
		}
	} else {
		let proxyType = world.bodies[body].type;
		world.fatAABBs[shapeId] = b3UpdateShapeAABBs(shape, transform, proxyType);
	}
}

export function b3Shape_SetFilter(shapeId: b3ShapeId, filter: b3Filter, invokeContacts: bool): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b3GetShape(world, shapeId);
	if (
		filter.maskBits == shape.filter.maskBits &&
		filter.categoryBits == shape.filter.categoryBits &&
		filter.groupIndex == shape.filter.groupIndex
	) {
		return;
	}

	shape.filter = filter;

	if (invokeContacts) {
		world.locked = true;
		let wakeBodies = true;

		// If the category bits change, I need to destroy the proxy because it affects the tree sorting.
		// C compares after the assignment above, so this is always true.
		let destroyProxy = filter.categoryBits == shape.filter.categoryBits;

		// need to wake bodies because a filter change may destroy contacts
		b3ResetProxy(world, shape, wakeBodies, destroyProxy);
		world.locked = false;
	}

	// note: this does not immediately update sensor overlaps. Instead sensor
	// overlaps are updated the next time step
}

export function b3Shape_EnableSensorEvents(shapeId: b3ShapeId, flag: bool): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b3GetShape(world, shapeId);
	shape.flags = flag ? shape.flags | b3_enableSensorEvents : shape.flags & ~b3_enableSensorEvents;
}

export function b3Shape_AreSensorEventsEnabled(shapeId: b3ShapeId): bool {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return (shape.flags & b3_enableSensorEvents) != 0;
}

export function b3Shape_EnableContactEvents(shapeId: b3ShapeId, flag: bool): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b3GetShape(world, shapeId);
	shape.flags = flag ? shape.flags | b3_enableContactEvents : shape.flags & ~b3_enableContactEvents;
}

export function b3Shape_AreContactEventsEnabled(shapeId: b3ShapeId): bool {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return (shape.flags & b3_enableContactEvents) != 0;
}

export function b3Shape_EnablePreSolveEvents(shapeId: b3ShapeId, flag: bool): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b3GetShape(world, shapeId);
	shape.flags = flag ? shape.flags | b3_enablePreSolveEvents : shape.flags & ~b3_enablePreSolveEvents;
}

export function b3Shape_ArePreSolveEventsEnabled(shapeId: b3ShapeId): bool {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return (shape.flags & b3_enablePreSolveEvents) != 0;
}

export function b3Shape_EnableHitEvents(shapeId: b3ShapeId, flag: bool): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b3GetShape(world, shapeId);
	shape.flags = flag ? shape.flags | b3_enableHitEvents : shape.flags & ~b3_enableHitEvents;
}

export function b3Shape_AreHitEventsEnabled(shapeId: b3ShapeId): bool {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return (shape.flags & b3_enableHitEvents) != 0;
}

export function b3Shape_GetType(shapeId: b3ShapeId): b3ShapeType {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	return shape.type;
}

export function b3Shape_GetSphere(shapeId: b3ShapeId): b3Sphere {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type == b3_sphereShape);
	return shape.sphere;
}

export function b3Shape_GetCapsule(shapeId: b3ShapeId): b3Capsule {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type == b3_capsuleShape);
	return shape.capsule;
}

export function b3Shape_GetHull(shapeId: b3ShapeId): b3HullData? {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type == b3_hullShape);
	return shape.hull;
}

// C returns the embedded b3Mesh by value; this returns a copy.
export function b3Shape_GetMesh(shapeId: b3ShapeId): b3Mesh {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type == b3_meshShape);
	let mesh = shape.mesh!;
	return new b3Mesh(mesh.data, mesh.scale);
}

export function b3Shape_GetHeightField(shapeId: b3ShapeId): b3HeightFieldData? {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type == b3_heightShape);
	return shape.heightField;
}

export function b3Shape_GetCompound(shapeId: b3ShapeId): b3CompoundData? {
	let world = b3GetWorld(shapeId.world0);
	let shape = b3GetShape(world, shapeId);
	if (B3_ASSERT_ENABLED) B3_ASSERT(shape.type == b3_compoundShape);
	return shape.compound;
}

export function b3Shape_SetSphere(shapeId: b3ShapeId, sphere: b3Sphere): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;

	let shape = b3GetShape(world, shapeId);

	b3DestroyShapeAllocationForShapeChange(world, shape);

	shape.sphere = sphere;
	shape.type = b3_sphereShape;
	shape.aabbMargin = b3ComputeShapeMargin(shape);

	// need to wake bodies so they can react to the shape change
	let wakeBodies = true;
	let destroyProxy = true;
	b3ResetProxy(world, shape, wakeBodies, destroyProxy);

	world.locked = false;
}

export function b3Shape_SetCapsule(shapeId: b3ShapeId, capsule: b3Capsule): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;

	let shape = b3GetShape(world, shapeId);

	b3DestroyShapeAllocationForShapeChange(world, shape);

	shape.capsule = capsule;
	shape.type = b3_capsuleShape;
	shape.aabbMargin = b3ComputeShapeMargin(shape);

	// need to wake bodies so they can react to the shape change
	let wakeBodies = true;
	let destroyProxy = true;
	b3ResetProxy(world, shape, wakeBodies, destroyProxy);

	world.locked = false;
}

export function b3Shape_SetHull(shapeId: b3ShapeId, hull: b3HullData): void {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;

	let shape = b3GetShape(world, shapeId);

	// Acquire the new hull before releasing the old so the input may safely alias
	// the shape's current shared data.
	let data = b3AddHullToDatabase(world, hull);

	// Same shared hull, avoid destroying contacts and recreating the proxy
	if (shape.type == b3_hullShape && data == shape.hull) {
		b3RemoveHullFromDatabase(world, data);
		world.locked = false;
		return;
	}

	b3DestroyShapeAllocationForShapeChange(world, shape);

	shape.hull = data;
	shape.type = b3_hullShape;
	shape.aabbMargin = b3ComputeShapeMargin(shape);

	// need to wake bodies so they can react to the shape change
	let wakeBodies = true;
	let destroyProxy = true;
	b3ResetProxy(world, shape, wakeBodies, destroyProxy);

	world.locked = false;
}

export function b3Shape_SetMesh(shapeId: b3ShapeId, meshData: b3MeshData, scale: b3Vec3): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidVec3(scale));
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidMesh(meshData));

	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	world.locked = true;

	let shape = b3GetShape(world, shapeId);

	b3DestroyShapeAllocationForShapeChange(world, shape);

	shape.mesh = new b3Mesh(meshData, b3SafeScale(scale));
	shape.type = b3_meshShape;
	shape.aabbMargin = b3ComputeShapeMargin(shape);

	// need to wake bodies so they can react to the shape change
	let wakeBodies = true;
	let destroyProxy = true;
	b3ResetProxy(world, shape, wakeBodies, destroyProxy);

	world.locked = false;
}

export function b3Shape_GetContactCapacity(shapeId: b3ShapeId): i32 {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return 0;
	}

	let shape = b3GetShape(world, shapeId);
	if (shape.sensorIndex != B3_NULL_INDEX) {
		return 0;
	}

	// Conservative and fast
	return world.bodies[shape.bodyId].contactCount;
}

export function b3Shape_GetContactData(shapeId: b3ShapeId, contactData: StaticArray<b3ContactData>, capacity: i32): i32 {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return 0;
	}

	let shape = b3GetShape(world, shapeId);
	if (shape.sensorIndex != B3_NULL_INDEX) {
		return 0;
	}

	let contactKey = world.bodies[shape.bodyId].headContactKey;
	let index = 0;
	while (contactKey != B3_NULL_INDEX && index < capacity) {
		let contactId = contactKey >> 1;
		let edgeIndex = contactKey & 1;

		let contact = world.contacts[contactId];

		// Does contact involve this shape and is it touching?
		if (
			(contact.shapeIdA == shapeId.index1 - 1 || contact.shapeIdB == shapeId.index1 - 1) &&
			(contact.flags & b3_contactTouchingFlag) != 0
		) {
			let shapeA = world.shapes[contact.shapeIdA];
			let shapeB = world.shapes[contact.shapeIdB];

			let data = new b3ContactData();
			let contactIdOut = b3ContactId();
			contactIdOut.index1 = contact.contactId + 1;
			contactIdOut.world0 = shapeId.world0;
			contactIdOut.padding = 0;
			contactIdOut.generation = contact.generation;
			data.contactId = contactIdOut;
			data.shapeIdA = b3MakeShapeIdFrom(shapeA.id + 1, shapeId.world0, shapeA.generation);
			data.shapeIdB = b3MakeShapeIdFrom(shapeB.id + 1, shapeId.world0, shapeB.generation);
			data.manifolds = contact.manifolds;
			data.manifoldCount = contact.manifoldCount;
			contactData[index] = data;
			index += 1;
		}

		contactKey = contact.edges[edgeIndex].nextKey;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(index <= capacity);

	return index;
}

export function b3Shape_GetSensorCapacity(shapeId: b3ShapeId): i32 {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return 0;
	}

	let shape = b3GetShape(world, shapeId);
	if (shape.sensorIndex == B3_NULL_INDEX) {
		return 0;
	}

	let sensor = world.sensors[shape.sensorIndex];
	return sensor.overlaps2.length;
}

export function b3Shape_GetSensorData(shapeId: b3ShapeId, visitorIds: StaticArray<b3ShapeId>, capacity: i32): i32 {
	let world = b3GetUnlockedWorld(shapeId.world0);
	if (world == null) {
		return 0;
	}

	let shape = b3GetShape(world, shapeId);
	if (shape.sensorIndex == B3_NULL_INDEX) {
		return 0;
	}

	let sensor = world.sensors[shape.sensorIndex];

	let count = b3MinInt(sensor.overlaps2.length, capacity);
	let refs = sensor.overlaps2;
	for (let i = 0; i < count; ++i) {
		let visitorId = b3MakeShapeIdFrom(refs[i].shapeId + 1, shapeId.world0, refs[i].generation);

		visitorIds[i] = visitorId;
	}

	return count;
}

export function b3Shape_GetAABB(shapeId: b3ShapeId): b3AABB {
	let world = b3GetWorld(shapeId.world0);
	if (world == null) {
		return b3AABB();
	}

	let shape = b3GetShape(world, shapeId);
	return shape.aabb;
}

export function b3Shape_ComputeMassData(shapeId: b3ShapeId): b3MassData {
	let world = b3GetWorld(shapeId.world0);
	if (world == null) {
		return b3MassData();
	}

	let shape = b3GetShape(world, shapeId);
	return b3ComputeShapeMass(shape);
}

export function b3Shape_GetClosestPoint(shapeId: b3ShapeId, target: b3Vec3): b3Vec3 {
	let world = b3GetWorld(shapeId.world0);
	if (world == null) {
		return b3Vec3_zero;
	}

	let shape = b3GetShape(world, shapeId);
	// Low level closest point query is a documented float carve-out far from the origin
	let transform = b3ToRelativeTransform(b3GetBodyTransformQuick(world, shape.bodyId), b3Pos_zero);

	let targetPoint = new StaticArray<b3Vec3>(1);
	targetPoint[0] = target;

	let input = new b3DistanceInput();
	input.proxyA = b3MakeShapeProxy(shape);
	input.proxyB = new b3ShapeProxy(targetPoint, 1, 0.0);
	input.transform = b3InvMulTransforms(transform, b3Transform_identity);
	input.useRadii = true;

	let cache = b3SimplexCache();
	let output = b3ShapeDistance(input, cache, null, 0);

	// Witness point comes back in frame A, lift it back to the query frame
	return b3TransformPoint(transform, output.pointA);
}

// https://en.wikipedia.org/wiki/Density_of_air
// https://www.engineeringtoolbox.com/wind-load-d_1775.html
// force = 0.5 * air_density * velocity^2 * area
// https://en.wikipedia.org/wiki/Lift_(force)
export function b3Shape_ApplyWind(shapeId: b3ShapeId, wind: b3Vec3, drag: f32, lift: f32, maxSpeed: f32, wake: bool): void {
	let world = b3GetWorld(shapeId.world0);
	if (world == null) {
		return;
	}

	let shape = b3GetShape(world, shapeId);

	let shapeType = shape.type;
	if (shapeType != b3_sphereShape && shapeType != b3_capsuleShape && shapeType != b3_hullShape) {
		return;
	}

	let body = shape.bodyId;

	if (world.bodies[body].type != b3_dynamicBody) {
		return;
	}

	if (world.bodies[body].setIndex == b3_disabledSet) {
		return;
	}

	if (world.bodies[body].setIndex >= b3_firstSleepingSet && wake == false) {
		return;
	}

	// C keeps this sim pointer across the wake below, so after a wake it points into the destroyed
	// sleeping set and the force written at the end is lost. The copy reproduces that.
	let sim = b3GetBodySim(world, body);
	let simWasAwake = world.bodies[body].setIndex == b3_awakeSet;

	if (world.bodies[body].setIndex != b3_awakeSet) {
		// Must wake for state to exist
		b3WakeBodyWithLock(world, body);
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(world.bodies[body].setIndex == b3_awakeSet);

	let state = world.solverSets[b3_awakeSet].bodyStates[b3GetBodyState(world, body)];
	// Only the rotation is used below, so the demoted world transform is exact
	let transform = b3ToRelativeTransform(sim.transform, b3Pos_zero);

	let lengthUnits = b3GetLengthUnitsPerMeter();
	let volumeUnits: f32 = lengthUnits * lengthUnits * lengthUnits;

	let airDensity: f32 = 1.225 / volumeUnits;

	let force = b3Vec3();
	let torque = b3Vec3();
	let speedOut: FixedArray<f32, 1>;

	switch (shape.type) {
		case b3_sphereShape: {
			let radius = shape.sphere.radius;
			let centroid = shape.localCentroid;
			let lever = b3RotateVector(transform.q, b3Sub(centroid, sim.localCenter));
			let shapeVelocity = b3Add(state.linearVelocity, b3Cross(state.angularVelocity, lever));
			let relativeVelocity = b3MulSub(wind, drag, shapeVelocity);
			let direction = b3GetLengthAndNormalize(speedOut[0], relativeVelocity);
			let speed = b3MinFloat(speedOut[0], maxSpeed);
			let projectedArea: f32 = B3_PI * radius * radius;
			force = b3MulSV(0.5 * airDensity * projectedArea * speed * speed, direction);
			torque = b3Cross(lever, force);
			break;
		}

		case b3_capsuleShape: {
			let centroid = shape.localCentroid;
			let lever = b3RotateVector(transform.q, b3Sub(centroid, sim.localCenter));
			let shapeVelocity = b3Add(state.linearVelocity, b3Cross(state.angularVelocity, lever));
			let relativeVelocity = b3MulSub(wind, drag, shapeVelocity);
			let direction = b3GetLengthAndNormalize(speedOut[0], relativeVelocity);
			let speed = b3MinFloat(speedOut[0], maxSpeed);

			let d = b3Sub(shape.capsule.center2, shape.capsule.center1);
			d = b3RotateVector(transform.q, d);

			let radius = shape.capsule.radius;
			let projectedArea: f32 = B3_PI * radius * radius + 2.0 * radius * b3Length(b3Cross(d, direction));

			// Normal that opposes the wind
			let e = b3Normalize(d);
			let normal = b3Sub(b3MulSV(b3Dot(direction, e), e), direction);

			// portion of wind that is perpendicular to surface
			let liftDirection = b3Cross(b3Cross(normal, direction), direction);

			let forceMagnitude: f32 = 0.5 * airDensity * projectedArea * speed * speed;
			force = b3MulSV(forceMagnitude, b3MulAdd(direction, lift, liftDirection));

			let edgeLever = b3MulAdd(lever, radius, normal);
			torque = b3Cross(edgeLever, force);
			break;
		}

		case b3_hullShape: {
			let matrix = b3MakeMatrixFromQuat(transform.q);

			let hull = shape.hull!;
			let faceCount = hull.faceCount;
			let points = b3GetHullPoints(hull);
			let faces = b3GetHullFaces(hull);
			let edges = b3GetHullEdges(hull);
			let planes = b3GetHullPlanes(hull);

			let linearVelocity = state.linearVelocity;
			let angularVelocity = state.angularVelocity;
			let localCenterOfMass = sim.localCenter;

			for (let i = 0; i < faceCount; ++i) {
				// C walks edge pointers; these are the edge indices
				let edge1 = <i32>faces[i].edge;
				let edge2 = <i32>edges[edge1].next;
				let edge3 = <i32>edges[edge2].next;

				if (B3_ASSERT_ENABLED) B3_ASSERT(edge1 != edge3);
				if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[edge1].origin < hull.vertexCount);
				if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[edge2].origin < hull.vertexCount);

				let localPoint1 = points[edges[edge1].origin];
				let localPoint2 = points[edges[edge2].origin];
				let v1 = b3MulMV(matrix, localPoint1);
				let v2 = b3MulMV(matrix, localPoint2);
				let normal = b3MulMV(matrix, planes[i].normal);

				do {
					if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[edge3].origin < hull.vertexCount);
					let localPoint3 = points[edges[edge3].origin];
					let v3 = b3MulMV(matrix, localPoint3);

					// Triangle center
					let localCenter = b3MulSV(0.333333, b3Add(localPoint1, b3Add(localPoint2, localPoint3)));

					// Lever arm from center of mass to triangle center in world space
					let lever = b3MulMV(matrix, b3Sub(localCenter, localCenterOfMass));

					// Velocity of the triangle center in world space
					let centerVelocity = b3Add(linearVelocity, b3Cross(angularVelocity, lever));

					let relativeVelocity = b3MulSub(wind, drag, centerVelocity);
					let direction = b3GetLengthAndNormalize(speedOut[0], relativeVelocity);
					let speed = speedOut[0];

					// Check for back-side
					if (b3Dot(normal, direction) < -FLT_EPSILON) {
						let projectedArea: f32 = -0.5 * b3Dot(b3Cross(b3Sub(v2, v1), b3Sub(v3, v1)), direction);

						let liftDirection = b3Cross(b3Cross(normal, direction), direction);

						speed = b3MinFloat(speed, maxSpeed);

						let forceMagnitude: f32 = 0.5 * airDensity * projectedArea * speed * speed;
						let deltaForce = b3MulSV(forceMagnitude, b3MulAdd(direction, lift, liftDirection));
						let deltaTorque = b3Cross(lever, deltaForce);

						force = b3Add(force, deltaForce);
						torque = b3Add(torque, deltaTorque);
					}

					edge2 = edge3;
					edge3 = <i32>edges[edge3].next;
					v2 = v3;
					localPoint2 = localPoint3;
				} while (edge1 != edge3);
			}
			break;
		}

		default:
			break;
	}

	sim.force = b3Add(sim.force, force);
	sim.torque = b3Add(sim.torque, torque);

	if (simWasAwake) {
		let sims = world.solverSets[b3_awakeSet].bodySims;
		let localIndex = world.bodies[body].localIndex;
		sims[localIndex].force = sim.force;
		sims[localIndex].torque = sim.torque;
	}
}

class b3MeshImpactContext {
	toiInput: b3TOIInput = new b3TOIInput();
	toiOutput: b3TOIOutput = b3TOIOutput();
	// Centroid of shape in body B local space
	localCentroidB: b3Vec3 = b3Vec3();
	// Centroid of shape at beginning and end of sweep in mesh local space. Used for early out.
	meshLocalCentroidB1: b3Vec3 = b3Vec3();
	meshLocalCentroidB2: b3Vec3 = b3Vec3();
	fallbackRadius: f32 = 0;
	isSensor: bool = false;
	// C's stack triangle, only read inside b3TimeOfImpact
	triangle: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(3);
}

// C copies b3TOIInput by value; the proxies are copied too so later field writes stay local.
function b3CopyTOIInput(copy: b3TOIInput, input: b3TOIInput): b3TOIInput {
	b3SetProxy(copy.proxyA, input.proxyA.points, input.proxyA.count, input.proxyA.radius);
	b3SetProxy(copy.proxyB, input.proxyB.points, input.proxyB.count, input.proxyB.radius);
	copy.sweepA = input.sweepA;
	copy.sweepB = input.sweepB;
	copy.maxFraction = input.maxFraction;
	return copy;
}

// A parallel part's context keeps its proxies, which own their points
function b3ResetMeshImpactContext<M>(context: b3MeshImpactContext): b3MeshImpactContext {
	let input = context.toiInput;
	if (isBoolean<M>()) {
		input.proxyA.count = 0;
		input.proxyA.radius = 0.0;
		input.proxyB.count = 0;
		input.proxyB.radius = 0.0;
	} else {
		b3SetProxy(input.proxyA, b3_noProxyPoints, 0, 0.0);
		b3SetProxy(input.proxyB, b3_noProxyPoints, 0, 0.0);
	}
	input.sweepA = b3Sweep();
	input.sweepB = b3Sweep();
	input.maxFraction = 0.0;
	context.toiOutput = b3TOIOutput();
	context.localCentroidB = b3Vec3();
	context.meshLocalCentroidB1 = b3Vec3();
	context.meshLocalCentroidB2 = b3Vec3();
	context.fallbackRadius = 0.0;
	context.isSensor = false;
	return context;
}

// C's stack locals of the time of impact functions below. b3ShapeTimeOfImpact never nests, and its
// mesh and compound mesh child paths never run at once, so one set serves all of them.
const s_meshImpactContext = new b3MeshImpactContext();
const s_fallbackInput = new b3TOIInput();
const s_fallbackCentroid = new StaticArray<b3Vec3>(1);
const s_impactPointsA = new StaticArray<b3Vec3>(2);
const s_impactPointsB = new StaticArray<b3Vec3>(2);
const s_impactInput = new b3TOIInput();
const s_impactChild = new b3ChildShape();
const s_impactChildMaterial = new Uint16Array(1);

// The same locals per worker for parallel parts, with proxies that own their points or already point at the context's
// triangle, and the mesh query's stack and visitor
class b3ImpactScratch {
	meshContext: b3MeshImpactContext = new b3MeshImpactContext();
	fallbackInput: b3TOIInput = new b3TOIInput();
	fallbackCentroid: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(1);
	impactInput: b3TOIInput = new b3TOIInput();
	meshStack: StaticArray<i32> = new StaticArray<i32>(B3_MESH_STACK_SIZE);
	visitor: b3MeshImpactVisitor;
	compoundContext: b3CompoundImpactContext = new b3CompoundImpactContext();
	compoundStack: StaticArray<i32> = new StaticArray<i32>(B3_TREE_STACK_SIZE);
	compoundVisitor: b3CompoundImpactVisitor;

	constructor(workerIndex: i32) {
		let meshInput = this.meshContext.toiInput;
		meshInput.proxyA.points = this.meshContext.triangle;
		meshInput.proxyB = b3MakeOwningProxy(B3_MAX_SHAPE_CAST_POINTS);
		this.fallbackInput.proxyA.points = this.meshContext.triangle;
		this.fallbackInput.proxyB.points = this.fallbackCentroid;
		this.impactInput.proxyA = b3MakeOwningProxy(B3_MAX_SHAPE_CAST_POINTS);
		this.impactInput.proxyB = b3MakeOwningProxy(B3_MAX_SHAPE_CAST_POINTS);
		this.visitor = new b3MeshImpactVisitor(this.meshContext, workerIndex);
		this.compoundContext.toiInput.proxyA = b3MakeOwningProxy(B3_MAX_SHAPE_CAST_POINTS);
		this.compoundContext.toiInput.proxyB = b3MakeOwningProxy(B3_MAX_SHAPE_CAST_POINTS);
		this.compoundVisitor = new b3CompoundImpactVisitor(this.compoundContext, workerIndex);
	}
}

// The compound query's callback for a parallel part, over the children the main lane built, b3PrepareCompoundContacts
class b3CompoundImpactVisitor {
	context: b3CompoundImpactContext;
	workerIndex: i32;
	world: b3World? = null;
	shapeId: i32 = 0;

	constructor(context: b3CompoundImpactContext, workerIndex: i32) {
		this.context = context;
		this.workerIndex = workerIndex;
	}

	visit(proxyId: i32, userData: u64): bool {
		let shape = this.world!.shapes[this.shapeId];
		return b3CompoundTimeOfImpactFcn<b3InParallel>(shape.compoundChildren![<i32>userData], this.context, this.workerIndex);
	}
}

// Points each worker's compound query at world for parallel parts
export function b3PrepareParallelImpacts(world: b3World, workerCount: i32): void {
	for (let i = 0; i < workerCount; ++i) {
		s_impactScratch[i].compoundVisitor.world = world;
	}
}

class b3MeshImpactVisitor {
	context: b3MeshImpactContext;
	workerIndex: i32;

	constructor(context: b3MeshImpactContext, workerIndex: i32) {
		this.context = context;
		this.workerIndex = workerIndex;
	}

	visit(a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32): bool {
		return b3MeshTimeOfImpactFcn<b3InParallel>(a, b, c, triangleIndex, this.context, this.workerIndex);
	}
}

function b3MakeImpactScratch(): StaticArray<b3ImpactScratch> {
	let scratch = new StaticArray<b3ImpactScratch>(B3_MAX_WORKERS);
	for (let i = 0; i < B3_MAX_WORKERS; ++i) {
		scratch[i] = new b3ImpactScratch(i);
	}
	return scratch;
}

const s_impactScratch = b3MakeImpactScratch();

function b3MeshTimeOfImpactCallback(a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32): bool {
	return b3MeshTimeOfImpactFcn<b3OnMain>(a, b, c, triangleIndex, s_meshImpactContext, 0);
}

function b3MeshTimeOfImpactFcn<M>(
	a: b3Vec3,
	b: b3Vec3,
	c: b3Vec3,
	_triangleIndex: i32,
	toiContext: b3MeshImpactContext,
	workerIndex: i32,
): bool {
	// Early out for parallel movement
	let c1 = toiContext.meshLocalCentroidB1;
	let c2 = toiContext.meshLocalCentroidB2;

	let n = b3Normalize(b3Cross(b3Sub(b, a), b3Sub(c, a)));
	let offset1 = b3Dot(n, b3Sub(c1, a));
	let offset2 = b3Dot(n, b3Sub(c2, a));

	if (offset1 < 0.0) {
		// Started behind or finished in front
		return true;
	}

	if (toiContext.isSensor == false && offset1 - offset2 < toiContext.fallbackRadius && offset2 > toiContext.fallbackRadius) {
		// Finished in front
		return true;
	}

	let triangle = toiContext.triangle;
	triangle[0] = a;
	triangle[1] = b;
	triangle[2] = c;
	if (!isBoolean<M>()) {
		toiContext.toiInput.proxyA.points = triangle;
	}
	toiContext.toiInput.proxyA.count = 3;

	let output = b3TimeOfImpactAt<M>(toiContext.toiInput, workerIndex);

	// It is possible for a hit at fraction == 0

	if (0.0 < output.fraction && output.fraction < toiContext.toiInput.maxFraction) {
		toiContext.toiOutput = output;
		toiContext.toiInput.maxFraction = output.fraction;
	} else if (0.0 == output.fraction) {
		// fallback to TOI of a small circle around the fast shape centroid
		let fallbackInput = s_fallbackInput;
		if (isBoolean<M>()) {
			// Its proxies already point at the context's triangle and its own centroid
			let scratch = s_impactScratch[workerIndex];
			fallbackInput = scratch.fallbackInput;
			let input = toiContext.toiInput;
			fallbackInput.proxyA.count = input.proxyA.count;
			fallbackInput.proxyA.radius = input.proxyA.radius;
			fallbackInput.sweepA = input.sweepA;
			fallbackInput.sweepB = input.sweepB;
			fallbackInput.maxFraction = input.maxFraction;
			scratch.fallbackCentroid[0] = toiContext.localCentroidB;
			fallbackInput.proxyB.count = 1;
			fallbackInput.proxyB.radius = toiContext.fallbackRadius + B3_LINEAR_SLOP();
		} else {
			b3CopyTOIInput(fallbackInput, toiContext.toiInput);
			let centroid = s_fallbackCentroid;
			centroid[0] = toiContext.localCentroidB;
			b3SetProxy(fallbackInput.proxyB, centroid, 1, toiContext.fallbackRadius + B3_LINEAR_SLOP());
		}
		output = b3TimeOfImpactAt<M>(fallbackInput, workerIndex);

		if (0.0 < output.fraction && output.fraction < toiContext.toiInput.maxFraction) {
			toiContext.toiOutput = output;
			toiContext.toiInput.maxFraction = output.fraction;
			toiContext.toiOutput.usedFallback = true;
		}
	}

	// Continue the query
	return true;
}

class b3CompoundImpactContext {
	toiInput: b3TOIInput = new b3TOIInput();
	toiOutput: b3TOIOutput = b3TOIOutput();
	compoundTransform: b3Transform = b3Transform();

	// Bounds local to compound
	localSweepBoundsB: b3AABB = b3AABB();

	// Centroid of shape in body B local space
	localCentroidB: b3Vec3 = b3Vec3();
	fallbackRadius: f32 = 0;
}

const s_compoundImpactContext = new b3CompoundImpactContext();

// Implements b3CompoundQueryFcn
function b3CompoundTimeOfImpactCallback(compound: b3CompoundData, childIndex: i32): bool {
	let child = s_impactChild;
	b3GetCompoundChildInto(compound, childIndex, child, s_impactChildMaterial);
	return b3CompoundTimeOfImpactFcn<b3OnMain>(child, s_compoundImpactContext, 0);
}

// A parallel part's context has proxies that own their points
function b3CompoundTimeOfImpactFcn<M>(child: b3ChildShape, toiContext: b3CompoundImpactContext, workerIndex: i32): bool {
	let output = b3TOIOutput();
	toiContext.toiInput.sweepA = b3MakeCompoundChildSweep(toiContext.compoundTransform, child.transform);

	switch (child.type) {
		case b3_capsuleShape: {
			let points = toiContext.toiInput.proxyA.points;
			if (!isBoolean<M>()) {
				points = s_impactPointsA;
				toiContext.toiInput.proxyA.points = points;
			}
			points[0] = child.capsule.center1;
			points[1] = child.capsule.center2;
			toiContext.toiInput.proxyA.count = 2;
			toiContext.toiInput.proxyA.radius = child.capsule.radius;
			output = b3TimeOfImpactAt<M>(toiContext.toiInput, workerIndex);
			break;
		}

		case b3_hullShape: {
			let hull = child.hull!;
			if (isBoolean<M>()) {
				b3CopyProxy(toiContext.toiInput.proxyA, b3GetHullPoints(hull), hull.vertexCount, 0.0);
			} else {
				toiContext.toiInput.proxyA.points = b3GetHullPoints(hull);
				toiContext.toiInput.proxyA.count = hull.vertexCount;
				toiContext.toiInput.proxyA.radius = 0.0;
			}
			output = b3TimeOfImpactAt<M>(toiContext.toiInput, workerIndex);
			break;
		}

		case b3_meshShape: {
			let scratch = s_impactScratch[workerIndex];
			let meshContext = s_meshImpactContext;
			if (isBoolean<M>()) {
				// The mesh context's proxy A already points at its triangle
				meshContext = b3ResetMeshImpactContext<M>(scratch.meshContext);
				let input = toiContext.toiInput;
				let meshInput = meshContext.toiInput;
				meshInput.proxyA.count = input.proxyA.count;
				meshInput.proxyA.radius = input.proxyA.radius;
				b3CopyProxy(meshInput.proxyB, input.proxyB.points, input.proxyB.count, input.proxyB.radius);
				meshInput.sweepA = input.sweepA;
				meshInput.sweepB = input.sweepB;
				meshInput.maxFraction = input.maxFraction;
			} else {
				b3ResetMeshImpactContext<M>(meshContext);
				b3CopyTOIInput(meshContext.toiInput, toiContext.toiInput);
			}
			meshContext.isSensor = false;
			meshContext.localCentroidB = toiContext.localCentroidB;
			meshContext.fallbackRadius = toiContext.fallbackRadius;

			let meshWorldTransform = b3MulTransforms(toiContext.compoundTransform, child.transform);

			let sweepB = toiContext.toiInput.sweepB;
			let xfB1 = b3Transform(b3Sub(sweepB.c1, b3RotateVector(sweepB.q1, sweepB.localCenter)), sweepB.q1);

			let xfB2 = b3Transform(b3Sub(sweepB.c2, b3RotateVector(sweepB.q2, sweepB.localCenter)), sweepB.q2);

			meshContext.meshLocalCentroidB1 = b3InvTransformPoint(meshWorldTransform, b3TransformPoint(xfB1, meshContext.localCentroidB));
			meshContext.meshLocalCentroidB2 = b3InvTransformPoint(meshWorldTransform, b3TransformPoint(xfB2, meshContext.localCentroidB));

			// Bounds local to mesh
			let localBounds = b3AABB_Transform(b3InvertTransform(child.transform), toiContext.localSweepBoundsB);

			if (isBoolean<M>()) {
				b3QueryMeshVisitor<b3MeshImpactVisitor>(child.mesh!, localBounds, scratch.visitor, scratch.meshStack);
			} else {
				b3QueryMesh(child.mesh!, localBounds, b3MeshTimeOfImpactCallback);
			}

			output = meshContext.toiOutput;
			break;
		}

		case b3_sphereShape: {
			let points = toiContext.toiInput.proxyA.points;
			if (!isBoolean<M>()) {
				points = s_impactPointsA;
				toiContext.toiInput.proxyA.points = points;
			}
			points[0] = child.sphere.center;
			toiContext.toiInput.proxyA.count = 1;
			toiContext.toiInput.proxyA.radius = child.sphere.radius;
			output = b3TimeOfImpactAt<M>(toiContext.toiInput, workerIndex);
			break;
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	if (0.0 < output.fraction && output.fraction < toiContext.toiInput.maxFraction) {
		toiContext.toiOutput = output;
		toiContext.toiInput.maxFraction = output.fraction;
	}

	// Clear this to be safe
	if (isBoolean<M>()) {
		toiContext.toiInput.proxyA.count = 0;
		toiContext.toiInput.proxyA.radius = 0.0;
	} else {
		b3SetProxy(toiContext.toiInput.proxyA, b3_noProxyPoints, 0, 0.0);
	}

	// Continue the query
	return true;
}

export function b3ShapeTimeOfImpact(shapeA: b3Shape, shapeB: b3Shape, sweepA: b3Sweep, sweepB: b3Sweep, maxFraction: f32): b3TOIOutput {
	return b3ShapeTimeOfImpactAt<b3OnMain>(shapeA, shapeB, sweepA, sweepB, maxFraction, 0);
}

// b3ShapeTimeOfImpact for worker workerIndex
export function b3ShapeTimeOfImpactAt<M>(
	shapeA: b3Shape,
	shapeB: b3Shape,
	sweepA: b3Sweep,
	sweepB: b3Sweep,
	maxFraction: f32,
	workerIndex: i32,
): b3TOIOutput {
	let isSensor = shapeA.sensorIndex != B3_NULL_INDEX;
	let scratch = s_impactScratch[workerIndex];

	let typeA = shapeA.type;
	if (typeA == b3_compoundShape) {
		// todo implement b3CompoundTimeOfImpact
		let context = s_compoundImpactContext;
		if (isBoolean<M>()) {
			context = scratch.compoundContext;
			context.toiInput.proxyA.count = 0;
			context.toiInput.proxyA.radius = 0.0;
			b3CopyShapeProxy(context.toiInput.proxyB, shapeB);
		} else {
			b3SetProxy(context.toiInput.proxyA, b3_noProxyPoints, 0, 0.0);
			b3SetShapeProxy(context.toiInput.proxyB, shapeB, s_impactPointsB);
		}
		context.toiInput.sweepA = b3Sweep();
		context.toiOutput = b3TOIOutput();
		context.toiInput.sweepB = sweepB;
		context.toiInput.maxFraction = maxFraction;

		context.compoundTransform = b3Transform(sweepA.c1, sweepA.q1);

		let localCentroidB = b3GetShapeCentroid(shapeB);
		context.localCentroidB = localCentroidB;

		let minExtent = b3ComputeShapeMinExtent(shapeB, context.localCentroidB);
		context.fallbackRadius = b3MaxFloat(0.75 * minExtent, B3_SPECULATIVE_DISTANCE());

		// Swept bounds of shapeB
		let bounds = b3ComputeSweptShapeAABB(shapeB, sweepB, maxFraction);

		// Bounds local to mesh
		let localBounds = b3AABB_Transform(b3InvertTransform(context.compoundTransform), bounds);
		context.localSweepBoundsB = localBounds;

		if (isBoolean<M>()) {
			// The main lane built the compound's children, b3PrepareCompoundContacts
			scratch.compoundVisitor.shapeId = shapeA.id;
			let compound = shapeA.compound!;
			b3QueryNodesVisitor<b3CompoundImpactVisitor>(compound.nodes, localBounds, B3_DEFAULT_MASK_BITS, false, scratch.compoundVisitor, scratch.compoundStack);
		} else {
			b3QueryCompound(shapeA.compound!, localBounds, b3CompoundTimeOfImpactCallback);
		}

		return context.toiOutput;
	}

	if (typeA == b3_heightShape || typeA == b3_meshShape) {
		// todo implement b3MeshTimeOfImpact and b3HeightFieldTimeOfImpact
		// Note: assuming mesh is static

		let context = s_meshImpactContext;
		if (isBoolean<M>()) {
			context = b3ResetMeshImpactContext<M>(scratch.meshContext);
			b3CopyShapeProxy(context.toiInput.proxyB, shapeB);
		} else {
			b3ResetMeshImpactContext<M>(context);
			b3SetShapeProxy(context.toiInput.proxyB, shapeB, s_impactPointsB);
		}
		context.toiInput.sweepA = sweepA;
		context.toiInput.proxyA.count = 3;
		context.toiInput.sweepB = sweepB;
		context.toiInput.maxFraction = maxFraction;
		context.isSensor = isSensor;

		let localCentroidB = b3GetShapeCentroid(shapeB);
		context.localCentroidB = localCentroidB;

		// Assume mesh is static
		let xfA = b3Transform(b3Sub(sweepA.c1, b3RotateVector(sweepA.q1, sweepA.localCenter)), sweepA.q1);

		let xfB1 = b3Transform(b3Sub(sweepB.c1, b3RotateVector(sweepB.q1, sweepB.localCenter)), sweepB.q1);

		let xfB2 = b3Transform(b3Sub(sweepB.c2, b3RotateVector(sweepB.q2, sweepB.localCenter)), sweepB.q2);

		context.meshLocalCentroidB1 = b3InvTransformPoint(xfA, b3TransformPoint(xfB1, localCentroidB));
		context.meshLocalCentroidB2 = b3InvTransformPoint(xfA, b3TransformPoint(xfB2, localCentroidB));

		let minExtent = b3ComputeShapeMinExtent(shapeB, context.localCentroidB);
		context.fallbackRadius = b3MaxFloat(0.5 * minExtent, B3_LINEAR_SLOP());

		// Swept bounds of shapeB
		// todo pass in xfA to get local bounds directly
		let bounds = b3ComputeSweptShapeAABB(shapeB, sweepB, maxFraction);

		// Bounds local to mesh
		let localBounds = b3AABB_Transform(b3InvertTransform(xfA), bounds);

		if (isBoolean<M>()) {
			if (typeA == b3_meshShape) {
				b3QueryMeshVisitor<b3MeshImpactVisitor>(shapeA.mesh!, localBounds, scratch.visitor, scratch.meshStack);
			} else if (typeA == b3_heightShape) {
				b3QueryHeightFieldVisitor<b3MeshImpactVisitor>(shapeA.heightField!, localBounds, scratch.visitor);
			}
		} else if (typeA == b3_meshShape) {
			b3QueryMesh(shapeA.mesh!, localBounds, b3MeshTimeOfImpactCallback);
		} else if (typeA == b3_heightShape) {
			b3QueryHeightField(shapeA.heightField!, localBounds, b3MeshTimeOfImpactCallback);
		}

		return context.toiOutput;
	}

	if (B3_ASSERT_ENABLED)
		B3_ASSERT(shapeB.type != b3_compoundShape && shapeB.type != b3_meshShape && shapeB.type != b3_heightShape);

	let input = s_impactInput;
	if (isBoolean<M>()) {
		input = scratch.impactInput;
		b3CopyShapeProxy(input.proxyA, shapeA);
		b3CopyShapeProxy(input.proxyB, shapeB);
	} else {
		b3SetShapeProxy(input.proxyA, shapeA, s_impactPointsA);
		b3SetShapeProxy(input.proxyB, shapeB, s_impactPointsB);
	}
	input.sweepA = sweepA;
	input.sweepB = sweepB;
	input.maxFraction = maxFraction;

	let output = b3TimeOfImpactAt<M>(input, workerIndex);

	return output;
}

// Resolve the user material id for a hit point on the given shape. Mesh/heightfield shapes
// use the manifold-point triangleIndex to pick a per-triangle material. Compound shapes use
// the contact's childIndex to find the participating child, then for a mesh child apply the
// child's materialIndices indirection on top of the per-triangle index. Convex shapes fall
// back to materials[0]. childIndex is unused for non-compound shapes.
export function b3GetShapeUserMaterialId(shape: b3Shape, childIndex: i32, triangleIndex: i32): u64 {
	if (shape.materialCount == 0) {
		return 0;
	}

	let materialIndex = 0;
	if (shape.type == b3_meshShape) {
		let meshData = shape.mesh!.data;
		if (meshData.materialOffset != 0) {
			let indices = b3GetMeshMaterialIndices(meshData);
			materialIndex = indices[triangleIndex];
		}
	} else if (shape.type == b3_heightShape) {
		materialIndex = b3GetHeightFieldMaterial(shape.heightField!, triangleIndex);
	} else if (shape.type == b3_compoundShape) {
		let child = b3GetCompoundChild(shape.compound!, childIndex);
		let childIndices = child.materialIndices!;
		if (child.type == b3_meshShape) {
			let meshData = child.mesh!.data;
			let meshMaterialIndex = meshData.materialOffset != 0 ? <i32>b3GetMeshMaterialIndices(meshData)[triangleIndex] : 0;
			if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= meshMaterialIndex && meshMaterialIndex < child.materialCount);
			materialIndex = childIndices[meshMaterialIndex];
		} else {
			materialIndex = childIndices[0];
		}
	}

	materialIndex = b3ClampInt(materialIndex, 0, shape.materialCount - 1);
	return b3GetShapeMaterials(shape)[materialIndex].userMaterialId;
}

function b3MakeShapeIdFrom(index1: i32, world0: u16, generation: u16): b3ShapeId {
	let id = b3ShapeId();
	id.index1 = index1;
	id.world0 = world0;
	id.generation = generation;
	return id;
}
