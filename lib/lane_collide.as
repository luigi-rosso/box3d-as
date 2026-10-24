// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The narrow phase on lanes. b3CollideTask runs unchanged on a lane's replica world: the shapes come from a
// snapshot blob made once until the shapes change, and each step serializes the contacts' state, their body sims
// and shape bounds, then copies the updated state back. A worker keeps the shapes it decoded from step to step.

import { Blob } from "rive/shared";

import { b3GetLengthUnitsPerMeter, b3SetLengthUnitsPerMeter } from "./core";
import { b3AABB, b3Quat, b3Transform, b3Vec3 } from "./math_functions";
import {
	b3Capsule,
	b3CompoundCapsule,
	b3CompoundData,
	b3CompoundSphere,
	b3HeightFieldData,
	b3HullData,
	b3HullFace,
	b3HullHalfEdge,
	b3HullInstance,
	b3HullVertex,
	b3Manifold,
	b3Mesh,
	b3MeshData,
	b3MeshInstance,
	b3MeshNode,
	b3MeshTriangle,
	b3Sphere,
	b3TreeNode,
} from "./collision";
import { b3Plane } from "./math_functions";
import { b3SurfaceMaterial } from "./types";
import { b3SetBit, b3SetBitCountAndClear } from "./bitset";
import { b3GetIdCapacity } from "./id_pool";
import { b3ArrayData, b3Body, b3BodySim, b3IsStaticSimIndex } from "./body";
import {
	b3Contact,
	b3ContactCache,
	b3ContactSpec,
	b3TriangleCache,
	b3_emptyManifolds,
	b3_simDisjoint,
	b3_simMeshContact,
	b3_simStartedTouching,
	b3_simStoppedTouching,
	b3_simTouchingFlag,
} from "./contact";
import { b3Shape } from "./shape";
import { b3SolverSet } from "./solver_set";
import { b3StepContext } from "./solver";
import {
	b3CollideTask,
	b3DefaultFrictionCallback,
	b3DefaultRestitutionCallback,
	b3World,
	b3_awakeSet,
	b3_firstSleepingSet,
	b3_staticSet,
} from "./physics_world";
import { B3_CONTACT_MANIFOLD_COUNT_BUCKETS } from "./constants";
import {
	B3_MAX_LANES,
	b3GrowI32,
	b3GrowU8,
	b3LaneNow,
	b3LaneWriter,
	b3MakeLaneTaskContext,
	b3ReadF32,
	b3ReadI32,
	b3ReadU32,
	b3SplitStart,
} from "./lane_common";

const SNAPSHOT_MAGIC: u32 = 0x53533342;

const G_HULL: i32 = 1;
const G_MESH: i32 = 2;
const G_HEIGHT: i32 = 3;
const G_COMPOUND: i32 = 4;

// Section head words: counts, then the capacities and settings the replica sizes itself by
const S_CONTACTS: i32 = 0;
const S_SHAPES: i32 = 4;
const S_BODIES: i32 = 8;
const S_CONTACT_CAPACITY: i32 = 12;
const S_SHAPE_CAPACITY: i32 = 16;
const S_BODY_CAPACITY: i32 = 20;
const S_RECYCLE_DISTANCE: i32 = 24;
const S_SIMD_WIDTH: i32 = 28;
const S_LENGTH_UNITS: i32 = 32;
const S_AWAKE_CAPACITY: i32 = 36;
const S_STATIC_CAPACITY: i32 = 40;
const S_SLEEPING_COUNT: i32 = 44;
const S_BYTES: i32 = 48;

// Body record: where the sim lives, then the sim
const BODY_AWAKE: i32 = 0;
const BODY_STATIC: i32 = 1;
const BODY_SLEEPING: i32 = 2;
const BODY_WORDS: i32 = 3;

// Shape record: id, then its bounds and its fat bounds
const SHAPE_BYTES: i32 = 4 + 2 * <i32>sizeof<b3AABB>();

// Contact record: the ids and sims the task reads, before the state it updates
const C_IDS_WORDS: i32 = 8;

// Output head: byte length, then the collide counters the task context holds
const O_SAT_CALLS: i32 = 4;
const O_SAT_HITS: i32 = 8;
const O_RECYCLED: i32 = 12;
const O_MANIFOLD_COUNTS: i32 = 16;
const O_BYTES: i32 = O_MANIFOLD_COUNTS + 4 * B3_CONTACT_MANIFOLD_COUNT_BUCKETS;

// Contact state: what b3CollideTask reads and writes on a contact. Written and read the same way both ways.
unsafe function b3StateBytes(contact: b3Contact): i32 {
	return 4 * 3 + 2 * <i32>sizeof<b3Quat>() + <i32>sizeof<b3Transform>() + 4 * 6 + <i32>sizeof<b3ContactCache>() +
		<i32>sizeof<b3AABB>() + contact.meshContact.triangleCache.length * <i32>sizeof<b3TriangleCache>() +
		contact.manifoldCount * <i32>sizeof<b3Manifold>();
}

unsafe function b3WriteState(at: usize, contact: b3Contact): usize {
	let triangleCount = contact.meshContact.triangleCache.length;
	store<u32>(at, contact.flags);
	store<i32>(at + 4, contact.manifoldCount);
	store<i32>(at + 8, triangleCount);
	at += 12;
	store<b3Quat>(at, contact.cachedRotationA);
	at += sizeof<b3Quat>();
	store<b3Quat>(at, contact.cachedRotationB);
	at += sizeof<b3Quat>();
	store<b3Transform>(at, contact.cachedRelativePose);
	at += sizeof<b3Transform>();
	store<f32>(at, contact.friction);
	store<f32>(at + 4, contact.restitution);
	store<f32>(at + 8, contact.rollingResistance);
	store<b3Vec3>(at + 12, contact.tangentVelocity);
	at += 24;
	store<b3ContactCache>(at, contact.convexContact.cache);
	at += sizeof<b3ContactCache>();
	store<b3AABB>(at, contact.meshContact.queryBounds);
	at += sizeof<b3AABB>();
	let triangleBytes = <usize>triangleCount * sizeof<b3TriangleCache>();
	if (triangleBytes > 0) {
		memory.copy(at, b3ArrayData(contact.meshContact.triangleCache), triangleBytes);
	}
	at += triangleBytes;
	let manifoldBytes = <usize>contact.manifoldCount * sizeof<b3Manifold>();
	if (manifoldBytes > 0) {
		memory.copy(at, changetype<usize>(contact.manifolds), manifoldBytes);
	}
	return at + manifoldBytes;
}

// Manifold arrays are sized to their count, as b3AllocateManifolds sizes them, so the lengths match the serial run
unsafe function b3ReadState(at: usize, contact: b3Contact): usize {
	contact.flags = load<u32>(at);
	let manifoldCount = load<i32>(at + 4);
	let triangleCount = load<i32>(at + 8);
	at += 12;
	contact.cachedRotationA = load<b3Quat>(at);
	at += sizeof<b3Quat>();
	contact.cachedRotationB = load<b3Quat>(at);
	at += sizeof<b3Quat>();
	contact.cachedRelativePose = load<b3Transform>(at);
	at += sizeof<b3Transform>();
	contact.friction = load<f32>(at);
	contact.restitution = load<f32>(at + 4);
	contact.rollingResistance = load<f32>(at + 8);
	contact.tangentVelocity = load<b3Vec3>(at + 12);
	at += 24;
	contact.convexContact.cache = load<b3ContactCache>(at);
	at += sizeof<b3ContactCache>();
	contact.meshContact.queryBounds = load<b3AABB>(at);
	at += sizeof<b3AABB>();
	let triangles = contact.meshContact.triangleCache;
	triangles.length = triangleCount;
	let triangleBytes = <usize>triangleCount * sizeof<b3TriangleCache>();
	if (triangleBytes > 0) {
		memory.copy(b3ArrayData(triangles), at, triangleBytes);
	}
	at += triangleBytes;
	if (manifoldCount == 0) {
		contact.manifolds = b3_emptyManifolds;
	} else if (contact.manifolds.length != manifoldCount) {
		contact.manifolds = new StaticArray<b3Manifold>(manifoldCount);
	}
	contact.manifoldCount = manifoldCount;
	let manifoldBytes = <usize>manifoldCount * sizeof<b3Manifold>();
	if (manifoldBytes > 0) {
		memory.copy(changetype<usize>(contact.manifolds), at, manifoldBytes);
	}
	return at + manifoldBytes;
}

// Shape snapshot

// Sections of struct arrays copy as raw bytes behind their length. Generic over the array, since a struct cannot
// be a generic argument (notes/findings.md), so the element size comes from the caller.
@inline unsafe function b3WriteSection<A>(w: b3LaneWriter, a: A, elementSize: usize): void {
	w.i32(a.length);
	w.bytes(changetype<usize>(a), a.length * <i32>elementSize);
}

@inline unsafe function b3ReadSection<A>(base: usize, at: i32, out: A, elementSize: usize): i32 {
	let bytes = out.length * <i32>elementSize;
	memory.copy(changetype<usize>(out), base + <usize>at, <usize>bytes);
	return at + bytes;
}

// The shapes and their geometry, made once on the main lane and kept until the shapes change
export class b3LaneSnapshot {
	writer: b3LaneWriter = new b3LaneWriter();
	blob: Blob? = null;
	checksum: u64 = 0;
	builds: i32 = 0;
	geometry: Map<usize, i32> = new Map<usize, i32>();
	geometryObjects: Array<usize> = new Array<usize>();
	geometryKinds: Array<i32> = new Array<i32>();
}

// What can change a shape the narrow phase reads, short of its geometry, which never changes once made
function b3ShapesChecksum(world: b3World): u64 {
	let h: u64 = 1469598103934665603;
	let shapes = world.shapes;
	h = (h ^ <u64>shapes.length) * 1099511628211;
	for (let i = 0; i < shapes.length; ++i) {
		let shape = shapes[i];
		h = (h ^ <u64>shape.id) * 1099511628211;
		h = (h ^ <u64>shape.generation) * 1099511628211;
		h = (h ^ (<u64>shape.type << 8) ^ <u64>shape.flags) * 1099511628211;
		let materials = shape.materials;
		for (let k = 0; k < shape.materialCount; ++k) {
			let m = materials[k];
			h = (h ^ <u64>reinterpret<u32>(m.friction)) * 1099511628211;
			h = (h ^ <u64>reinterpret<u32>(m.restitution)) * 1099511628211;
			h = (h ^ <u64>reinterpret<u32>(m.rollingResistance)) * 1099511628211;
			h = (h ^ m.userMaterialId) * 1099511628211;
			h = (h ^ <u64>reinterpret<u32>(m.tangentVelocity.x) ^ (<u64>reinterpret<u32>(m.tangentVelocity.y) << 32)) * 1099511628211;
			h = (h ^ <u64>reinterpret<u32>(m.tangentVelocity.z)) * 1099511628211;
		}
	}
	return h;
}

function b3GeometryIndex(s: b3LaneSnapshot, object: usize, kind: i32): i32 {
	if (s.geometry.has(object)) {
		return s.geometry.get(object);
	}
	let index = s.geometryObjects.length;
	s.geometry.set(object, index);
	s.geometryObjects.push(object);
	s.geometryKinds.push(kind);
	return index;
}

unsafe function b3WriteHull(w: b3LaneWriter, hull: b3HullData): void {
	w.bytes(changetype<usize>(hull), <i32>offsetof<b3HullData>("vertices"));
	b3WriteSection(w, hull.vertices, sizeof<b3HullVertex>());
	b3WriteSection(w, hull.points, sizeof<b3Vec3>());
	b3WriteSection(w, hull.edges, sizeof<b3HullHalfEdge>());
	b3WriteSection(w, hull.planes, sizeof<b3Plane>());
	b3WriteSection(w, hull.faces, sizeof<b3HullFace>());
	b3WriteSection(w, hull.soaVertices, sizeof<f32>());
	b3WriteSection(w, hull.soaNormals, sizeof<f32>());
	b3WriteSection(w, hull.edgeCosines, sizeof<f32>());
}

unsafe function b3WriteMeshData(w: b3LaneWriter, mesh: b3MeshData): void {
	w.bytes(changetype<usize>(mesh), <i32>offsetof<b3MeshData>("nodes"));
	b3WriteSection(w, mesh.nodes, sizeof<b3MeshNode>());
	b3WriteSection(w, mesh.vertices, sizeof<b3Vec3>());
	b3WriteSection(w, mesh.triangles, sizeof<b3MeshTriangle>());
	b3WriteSection(w, mesh.materialIndices, sizeof<u8>());
	b3WriteSection(w, mesh.flags, sizeof<u8>());
}

unsafe function b3WriteHeightField(w: b3LaneWriter, field: b3HeightFieldData): void {
	w.bytes(changetype<usize>(field), <i32>offsetof<b3HeightFieldData>("compressedHeights"));
	b3WriteSection(w, field.compressedHeights, sizeof<u16>());
	b3WriteSection(w, field.materialIndices, sizeof<u8>());
	b3WriteSection(w, field.flags, sizeof<u8>());
}

unsafe function b3WriteCompound(s: b3LaneSnapshot, w: b3LaneWriter, compound: b3CompoundData): void {
	w.bytes(changetype<usize>(compound), <i32>offsetof<b3CompoundData>("nodes"));
	b3WriteSection(w, compound.nodes, sizeof<b3TreeNode>());
	b3WriteSection(w, compound.materials, sizeof<b3SurfaceMaterial>());
	b3WriteSection(w, compound.capsules, sizeof<b3CompoundCapsule>());
	b3WriteSection(w, compound.hullInstances, sizeof<b3HullInstance>());
	w.i32(compound.hulls.length);
	for (let i = 0; i < compound.hulls.length; ++i) {
		w.i32(b3GeometryIndex(s, changetype<usize>(compound.hulls[i]), G_HULL));
	}
	b3WriteSection(w, compound.meshInstances, sizeof<b3MeshInstance>());
	let indices = compound.meshMaterialIndices;
	w.i32(indices.length);
	w.bytes(indices.dataStart, indices.length * 2);
	w.i32(compound.meshes.length);
	for (let i = 0; i < compound.meshes.length; ++i) {
		w.i32(b3GeometryIndex(s, changetype<usize>(compound.meshes[i]), G_MESH));
	}
	b3WriteSection(w, compound.spheres, sizeof<b3CompoundSphere>());
}

// Layout: magic, shape slots, geometry count, the offset of each shape (0 for an empty slot) and of each
// geometry, then the records
unsafe function b3BuildSnapshot(s: b3LaneSnapshot, world: b3World): void {
	s.geometry.clear();
	s.geometryObjects.length = 0;
	s.geometryKinds.length = 0;
	let shapes = world.shapes;
	let shapeCount = shapes.length;
	let shapeOffsets = new Int32Array(shapeCount);

	let w = s.writer;
	w.reset();
	// The head and tables come first; the geometry count is known only after the shapes
	let shapeBody = new b3LaneWriter();
	for (let i = 0; i < shapeCount; ++i) {
		let shape = shapes[i];
		if (shape.id != i) {
			continue;
		}
		shapeOffsets[i] = shapeBody.length;
		shapeBody.i32(shape.id);
		shapeBody.i32(shape.bodyId);
		shapeBody.i32(shape.sensorIndex);
		shapeBody.i32(shape.type);
		shapeBody.u32(shape.flags);
		shapeBody.u32(shape.generation);
		let at = shapeBody.reserve(<i32>sizeof<b3Vec3>() + <i32>sizeof<b3Capsule>() + <i32>sizeof<b3Sphere>());
		store<b3Vec3>(at, shape.localCentroid);
		store<b3Capsule>(at + sizeof<b3Vec3>(), shape.capsule);
		store<b3Sphere>(at + sizeof<b3Vec3>() + sizeof<b3Capsule>(), shape.sphere);
		let kind = 0;
		let geometry = -1;
		let scale = b3Vec3(1.0, 1.0, 1.0);
		if (shape.hull != null) {
			kind = G_HULL;
			geometry = b3GeometryIndex(s, changetype<usize>(shape.hull!), G_HULL);
		} else if (shape.mesh != null) {
			kind = G_MESH;
			geometry = b3GeometryIndex(s, changetype<usize>(shape.mesh!.data), G_MESH);
			scale = shape.mesh!.scale;
		} else if (shape.heightField != null) {
			kind = G_HEIGHT;
			geometry = b3GeometryIndex(s, changetype<usize>(shape.heightField!), G_HEIGHT);
		} else if (shape.compound != null) {
			kind = G_COMPOUND;
			geometry = b3GeometryIndex(s, changetype<usize>(shape.compound!), G_COMPOUND);
		}
		shapeBody.i32(kind);
		shapeBody.i32(geometry);
		shapeBody.f32(scale.x);
		shapeBody.f32(scale.y);
		shapeBody.f32(scale.z);
		shapeBody.i32(shape.materialCount);
		b3WriteSection(shapeBody, shape.materials, sizeof<b3SurfaceMaterial>());
	}

	// Geometry records; a compound adds the hulls and meshes it holds, which the loop then reaches
	let geometryBody = new b3LaneWriter();
	let geometryOffsets = new Array<i32>();
	for (let g = 0; g < s.geometryObjects.length; ++g) {
		geometryOffsets.push(geometryBody.length);
		let kind = s.geometryKinds[g];
		let object = s.geometryObjects[g];
		geometryBody.i32(kind);
		if (kind == G_HULL) {
			b3WriteHull(geometryBody, changetype<b3HullData>(object));
		} else if (kind == G_MESH) {
			b3WriteMeshData(geometryBody, changetype<b3MeshData>(object));
		} else if (kind == G_HEIGHT) {
			b3WriteHeightField(geometryBody, changetype<b3HeightFieldData>(object));
		} else {
			b3WriteCompound(s, geometryBody, changetype<b3CompoundData>(object));
		}
	}

	let geometryCount = geometryOffsets.length;
	let head = 16 + 4 * shapeCount + 4 * geometryCount;
	w.u32(SNAPSHOT_MAGIC);
	w.i32(shapeCount);
	w.i32(geometryCount);
	w.i32(head);
	for (let i = 0; i < shapeCount; ++i) {
		w.i32(shapes[i].id == i ? head + shapeOffsets[i] : 0);
	}
	let geometryBase = head + shapeBody.length;
	for (let g = 0; g < geometryCount; ++g) {
		w.i32(geometryBase + geometryOffsets[g]);
	}
	w.bytes(shapeBody.buffer.dataStart, shapeBody.length);
	w.bytes(geometryBody.buffer.dataStart, geometryBody.length);

	let old = s.blob;
	if (old != null) {
		old.release();
	}
	s.blob = Blob.fromArray(w.buffer, 0, w.length);
	s.builds += 1;
}

// A lane's decoded shapes, made from the snapshot on first use. A worker keeps them while the snapshot stays.
export class b3LaneShapeCache {
	snapshotId: u32 = 0;
	bytes: Uint8Array = new Uint8Array(0);
	shapes: Array<b3Shape> = new Array<b3Shape>();
	decoded: Uint8Array = new Uint8Array(0);
	// Per geometry record, its slot in the array of its kind once decoded, else -1
	slots: Int32Array = new Int32Array(0);
	hulls: Array<b3HullData> = new Array<b3HullData>();
	meshes: Array<b3MeshData> = new Array<b3MeshData>();
	fields: Array<b3HeightFieldData> = new Array<b3HeightFieldData>();
	compounds: Array<b3CompoundData> = new Array<b3CompoundData>();
	none: b3Shape = new b3Shape();
}

function b3UseSnapshot(cache: b3LaneShapeCache, snapshot: Blob): void {
	if (cache.snapshotId == snapshot.id && cache.bytes.length > 0) {
		return;
	}
	cache.snapshotId = snapshot.id;
	let length = <i32>snapshot.byteLength;
	cache.bytes = new Uint8Array(length);
	snapshot.readInto(0, cache.bytes);
	let base = cache.bytes.dataStart;
	let shapeCount = b3ReadI32(base, 4);
	let geometryCount = b3ReadI32(base, 8);
	cache.decoded = new Uint8Array(shapeCount);
	cache.slots = new Int32Array(geometryCount);
	cache.slots.fill(-1);
	cache.hulls.length = 0;
	cache.meshes.length = 0;
	cache.fields.length = 0;
	cache.compounds.length = 0;
	let shapes = cache.shapes;
	shapes.length = 0;
	for (let i = 0; i < shapeCount; ++i) {
		shapes.push(new b3Shape());
	}
}

// The byte offset of geometry record g, past its kind word
@inline unsafe function b3GeometryAt(cache: b3LaneShapeCache, g: i32): i32 {
	let base = cache.bytes.dataStart;
	let shapeCount = b3ReadI32(base, 4);
	return b3ReadI32(base, 16 + 4 * shapeCount + 4 * g) + 4;
}

unsafe function b3DecodeHull(cache: b3LaneShapeCache, g: i32): b3HullData {
	if (cache.slots[g] >= 0) {
		return cache.hulls[cache.slots[g]];
	}
	let base = cache.bytes.dataStart;
	let at = b3GeometryAt(cache, g);
	let hull = new b3HullData();
	memory.copy(changetype<usize>(hull), base + <usize>at, offsetof<b3HullData>("vertices"));
	at += <i32>offsetof<b3HullData>("vertices");
	hull.vertices = new StaticArray<b3HullVertex>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, hull.vertices, sizeof<b3HullVertex>());
	hull.points = new StaticArray<b3Vec3>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, hull.points, sizeof<b3Vec3>());
	hull.edges = new StaticArray<b3HullHalfEdge>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, hull.edges, sizeof<b3HullHalfEdge>());
	hull.planes = new StaticArray<b3Plane>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, hull.planes, sizeof<b3Plane>());
	hull.faces = new StaticArray<b3HullFace>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, hull.faces, sizeof<b3HullFace>());
	hull.soaVertices = new StaticArray<f32>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, hull.soaVertices, sizeof<f32>());
	hull.soaNormals = new StaticArray<f32>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, hull.soaNormals, sizeof<f32>());
	hull.edgeCosines = new StaticArray<f32>(b3ReadI32(base, at));
	b3ReadSection(base, at + 4, hull.edgeCosines, sizeof<f32>());
	cache.slots[g] = cache.hulls.length;
	cache.hulls.push(hull);
	return hull;
}

unsafe function b3DecodeMesh(cache: b3LaneShapeCache, g: i32): b3MeshData {
	if (cache.slots[g] >= 0) {
		return cache.meshes[cache.slots[g]];
	}
	let base = cache.bytes.dataStart;
	let at = b3GeometryAt(cache, g);
	let mesh = new b3MeshData();
	memory.copy(changetype<usize>(mesh), base + <usize>at, offsetof<b3MeshData>("nodes"));
	at += <i32>offsetof<b3MeshData>("nodes");
	mesh.nodes = new StaticArray<b3MeshNode>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, mesh.nodes, sizeof<b3MeshNode>());
	mesh.vertices = new StaticArray<b3Vec3>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, mesh.vertices, sizeof<b3Vec3>());
	mesh.triangles = new StaticArray<b3MeshTriangle>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, mesh.triangles, sizeof<b3MeshTriangle>());
	mesh.materialIndices = new StaticArray<u8>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, mesh.materialIndices, sizeof<u8>());
	mesh.flags = new StaticArray<u8>(b3ReadI32(base, at));
	b3ReadSection(base, at + 4, mesh.flags, sizeof<u8>());
	cache.slots[g] = cache.meshes.length;
	cache.meshes.push(mesh);
	return mesh;
}

unsafe function b3DecodeHeightField(cache: b3LaneShapeCache, g: i32): b3HeightFieldData {
	if (cache.slots[g] >= 0) {
		return cache.fields[cache.slots[g]];
	}
	let base = cache.bytes.dataStart;
	let at = b3GeometryAt(cache, g);
	let field = new b3HeightFieldData();
	memory.copy(changetype<usize>(field), base + <usize>at, offsetof<b3HeightFieldData>("compressedHeights"));
	at += <i32>offsetof<b3HeightFieldData>("compressedHeights");
	field.compressedHeights = new StaticArray<u16>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, field.compressedHeights, sizeof<u16>());
	field.materialIndices = new StaticArray<u8>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, field.materialIndices, sizeof<u8>());
	field.flags = new StaticArray<u8>(b3ReadI32(base, at));
	b3ReadSection(base, at + 4, field.flags, sizeof<u8>());
	cache.slots[g] = cache.fields.length;
	cache.fields.push(field);
	return field;
}

unsafe function b3DecodeCompound(cache: b3LaneShapeCache, g: i32): b3CompoundData {
	if (cache.slots[g] >= 0) {
		return cache.compounds[cache.slots[g]];
	}
	let base = cache.bytes.dataStart;
	let at = b3GeometryAt(cache, g);
	let compound = new b3CompoundData();
	memory.copy(changetype<usize>(compound), base + <usize>at, offsetof<b3CompoundData>("nodes"));
	at += <i32>offsetof<b3CompoundData>("nodes");
	compound.nodes = new StaticArray<b3TreeNode>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, compound.nodes, sizeof<b3TreeNode>());
	compound.materials = new StaticArray<b3SurfaceMaterial>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, compound.materials, sizeof<b3SurfaceMaterial>());
	compound.capsules = new StaticArray<b3CompoundCapsule>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, compound.capsules, sizeof<b3CompoundCapsule>());
	compound.hullInstances = new StaticArray<b3HullInstance>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, compound.hullInstances, sizeof<b3HullInstance>());
	let hullCount = b3ReadI32(base, at);
	at += 4;
	let hulls = new StaticArray<b3HullData>(hullCount);
	for (let i = 0; i < hullCount; ++i) {
		hulls[i] = b3DecodeHull(cache, b3ReadI32(base, at));
		at += 4;
	}
	compound.hulls = hulls;
	compound.meshInstances = new StaticArray<b3MeshInstance>(b3ReadI32(base, at));
	at = b3ReadSection(base, at + 4, compound.meshInstances, sizeof<b3MeshInstance>());
	let indexCount = b3ReadI32(base, at);
	at += 4;
	let indices = new Uint16Array(indexCount);
	memory.copy(indices.dataStart, base + <usize>at, <usize>(2 * indexCount));
	at += 2 * indexCount;
	compound.meshMaterialIndices = indices;
	let meshCount = b3ReadI32(base, at);
	at += 4;
	let meshes = new StaticArray<b3MeshData>(meshCount);
	for (let i = 0; i < meshCount; ++i) {
		meshes[i] = b3DecodeMesh(cache, b3ReadI32(base, at));
		at += 4;
	}
	compound.meshes = meshes;
	compound.spheres = new StaticArray<b3CompoundSphere>(b3ReadI32(base, at));
	b3ReadSection(base, at + 4, compound.spheres, sizeof<b3CompoundSphere>());
	cache.slots[g] = cache.compounds.length;
	cache.compounds.push(compound);
	return compound;
}

unsafe function b3DecodeShape(cache: b3LaneShapeCache, shapeId: i32): b3Shape {
	let shape = cache.shapes[shapeId];
	if (cache.decoded[shapeId] != 0) {
		return shape;
	}
	let base = cache.bytes.dataStart;
	let at = b3ReadI32(base, 16 + 4 * shapeId);
	shape.id = b3ReadI32(base, at);
	shape.bodyId = b3ReadI32(base, at + 4);
	shape.sensorIndex = b3ReadI32(base, at + 8);
	shape.type = b3ReadI32(base, at + 12);
	shape.flags = <u8>b3ReadU32(base, at + 16);
	shape.generation = <u16>b3ReadU32(base, at + 20);
	at += 24;
	shape.localCentroid = load<b3Vec3>(base + <usize>at);
	shape.capsule = load<b3Capsule>(base + <usize>at + sizeof<b3Vec3>());
	shape.sphere = load<b3Sphere>(base + <usize>at + sizeof<b3Vec3>() + sizeof<b3Capsule>());
	at += <i32>(sizeof<b3Vec3>() + sizeof<b3Capsule>() + sizeof<b3Sphere>());
	let kind = b3ReadI32(base, at);
	let geometry = b3ReadI32(base, at + 4);
	let scale = b3Vec3(b3ReadF32(base, at + 8), b3ReadF32(base, at + 12), b3ReadF32(base, at + 16));
	shape.materialCount = b3ReadI32(base, at + 20);
	at += 24;
	let materials = new StaticArray<b3SurfaceMaterial>(b3ReadI32(base, at));
	b3ReadSection(base, at + 4, materials, sizeof<b3SurfaceMaterial>());
	shape.materials = materials;
	if (kind == G_HULL) {
		shape.hull = b3DecodeHull(cache, geometry);
	} else if (kind == G_MESH) {
		shape.mesh = new b3Mesh(b3DecodeMesh(cache, geometry), scale);
	} else if (kind == G_HEIGHT) {
		shape.heightField = b3DecodeHeightField(cache, geometry);
	} else if (kind == G_COMPOUND) {
		shape.compound = b3DecodeCompound(cache, geometry);
	}
	cache.decoded[shapeId] = 1;
	return shape;
}

// A lane's world for b3CollideTask: the contacts, shapes, bounds and sims a section names, at the indices the
// main lane's world has them
export class b3LaneCollideReplica {
	world: b3World = new b3World();
	context: b3StepContext;
	cache: b3LaneShapeCache = new b3LaneShapeCache();
	dummyContact: b3Contact = new b3Contact();
	input: Uint8Array = new Uint8Array(0);
	output: Uint8Array = new Uint8Array(0);
	contactIds: StaticArray<i32> = new StaticArray<i32>(0);

	constructor() {
		this.context = new b3StepContext(this.world);
		let world = this.world;
		for (let i = 0; i <= b3_firstSleepingSet; ++i) {
			world.solverSets.push(new b3SolverSet());
		}
		world.taskContexts.push(b3MakeLaneTaskContext());
		world.frictionCallback = b3DefaultFrictionCallback;
		world.restitutionCallback = b3DefaultRestitutionCallback;
		// b3CollideTask writes a mesh contact's manifold count into its graph color; here that lands in one spare spec
		world.constraintGraph.colors[0].contacts.push(b3ContactSpec());
	}
}

@inline function b3EnsureSims(sims: Array<b3BodySim>, count: i32): void {
	if (sims.length < count) {
		sims.length = count;
	}
}

// Runs b3CollideTask over a section on the replica and answers the contacts' updated state as a blob, whose size
// only the task knows: a mesh contact's manifolds and triangle cache may grow
export unsafe function b3LaneCollidePartition(r: b3LaneCollideReplica, snapshot: Blob, input: Blob, partition: i32): Blob {
	let table = new Uint32Array(2 + 2 * B3_MAX_LANES);
	input.readInto(0, table);
	let lanes = <i32>table[1];
	let offset = table[2 + partition];
	let length = <i32>table[2 + lanes + partition];
	r.input = b3GrowU8(r.input, length);
	input.readInto(offset, r.input, 0, length);
	b3UseSnapshot(r.cache, snapshot);

	let base = r.input.dataStart;
	let world = r.world;
	let contactCount = b3ReadI32(base, S_CONTACTS);
	let shapeCount = b3ReadI32(base, S_SHAPES);
	let bodyCount = b3ReadI32(base, S_BODIES);
	world.contactRecycleDistance = b3ReadF32(base, S_RECYCLE_DISTANCE);
	world.simdWidth = b3ReadI32(base, S_SIMD_WIDTH);
	b3SetLengthUnitsPerMeter(b3ReadF32(base, S_LENGTH_UNITS));
	let taskContext = world.taskContexts[0];
	b3SetBitCountAndClear(taskContext.contactStateBitSet, <u32>b3ReadI32(base, S_CONTACT_CAPACITY));
	taskContext.satCallCount = 0;
	taskContext.satCacheHitCount = 0;
	taskContext.recycledContactCount = 0;
	for (let j = 0; j < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++j) {
		taskContext.manifoldCounts[j] = 0;
	}

	// Shapes and their bounds
	let shapeCapacity = b3ReadI32(base, S_SHAPE_CAPACITY);
	let shapes = world.shapes;
	while (shapes.length < shapeCapacity) {
		shapes.push(r.cache.none);
	}
	if (world.fatAABBs.length < shapeCapacity) {
		world.fatAABBs.length = shapeCapacity;
	}
	let fatAABBs = b3ArrayData(world.fatAABBs);
	let at = S_BYTES;
	for (let k = 0; k < shapeCount; ++k) {
		let shapeId = b3ReadI32(base, at);
		let shape = b3DecodeShape(r.cache, shapeId);
		shape.aabb = load<b3AABB>(base + <usize>at + 4);
		shapes[shapeId] = shape;
		memory.copy(fatAABBs + <usize>shapeId * sizeof<b3AABB>(), base + <usize>at + 4 + sizeof<b3AABB>(), sizeof<b3AABB>());
		at += SHAPE_BYTES;
	}

	// Sims where the encoded indices find them
	let awakeSims = world.solverSets[b3_awakeSet].bodySims;
	let staticSims = world.solverSets[b3_staticSet].bodySims;
	let sleepingSims = world.solverSets[b3_firstSleepingSet].bodySims;
	b3EnsureSims(awakeSims, b3ReadI32(base, S_AWAKE_CAPACITY));
	b3EnsureSims(staticSims, b3ReadI32(base, S_STATIC_CAPACITY));
	b3EnsureSims(sleepingSims, b3ReadI32(base, S_SLEEPING_COUNT));
	let bodyCapacity = b3ReadI32(base, S_BODY_CAPACITY);
	if (world.bodies.length < bodyCapacity) {
		world.bodies.length = bodyCapacity;
	}
	let simSize = sizeof<b3BodySim>();
	for (let k = 0; k < bodyCount; ++k) {
		let kind = b3ReadI32(base, at);
		let index = b3ReadI32(base, at + 4);
		let bodyId = b3ReadI32(base, at + 8);
		at += BODY_WORDS * 4;
		let sims = kind == BODY_AWAKE ? awakeSims : kind == BODY_STATIC ? staticSims : sleepingSims;
		memory.copy(b3ArrayData(sims) + <usize>index * simSize, base + <usize>at, simSize);
		at += <i32>simSize;
		if (kind == BODY_SLEEPING) {
			world.bodies[bodyId].setIndex = b3_firstSleepingSet;
			world.bodies[bodyId].localIndex = index;
		}
	}

	// Contacts
	let contacts = world.contacts;
	let contactCapacity = b3ReadI32(base, S_CONTACT_CAPACITY);
	while (contacts.length < contactCapacity) {
		contacts.push(r.dummyContact);
	}
	if (r.contactIds.length < contactCount) {
		r.contactIds = new StaticArray<i32>(contactCount + (contactCount >> 2) + 16);
	}
	let ids = r.contactIds;
	for (let k = 0; k < contactCount; ++k) {
		let contactId = b3ReadI32(base, at);
		let contact = contacts[contactId];
		if (contact == r.dummyContact) {
			contact = new b3Contact();
			contacts[contactId] = contact;
		}
		ids[k] = contactId;
		contact.contactId = contactId;
		contact.shapeIdA = b3ReadI32(base, at + 4);
		contact.shapeIdB = b3ReadI32(base, at + 8);
		contact.childIndex = b3ReadI32(base, at + 12);
		contact.edges[0].bodyId = b3ReadI32(base, at + 16);
		contact.edges[1].bodyId = b3ReadI32(base, at + 20);
		contact.encodedBodySimA = b3ReadI32(base, at + 24);
		contact.encodedBodySimB = b3ReadI32(base, at + 28);
		contact.colorIndex = 0;
		contact.localIndex = 0;
		at = <i32>(b3ReadState(base + <usize>(at + C_IDS_WORDS * 4), contact) - base);
	}

	r.context.awakeContactIndices = ids;
	if (contactCount > 0) {
		b3CollideTask(0, contactCount, 0, r.context);
	}

	// Output: the head, then each contact's state in section order
	let bytes = O_BYTES;
	for (let k = 0; k < contactCount; ++k) {
		bytes += b3StateBytes(contacts[ids[k]]);
	}
	r.output = b3GrowU8(r.output, bytes);
	let out = r.output.dataStart;
	store<i32>(out, bytes);
	store<i32>(out + O_SAT_CALLS, taskContext.satCallCount);
	store<i32>(out + O_SAT_HITS, taskContext.satCacheHitCount);
	store<i32>(out + O_RECYCLED, taskContext.recycledContactCount);
	for (let j = 0; j < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++j) {
		store<i32>(out + <usize>(O_MANIFOLD_COUNTS + 4 * j), taskContext.manifoldCounts[j]);
	}
	let write = out + <usize>O_BYTES;
	for (let k = 0; k < contactCount; ++k) {
		write = b3WriteState(write, contacts[ids[k]]);
	}
	return Blob.fromArray(r.output, 0, bytes);
}

// Main lane side

export class b3LaneCollideMain {
	snapshot: b3LaneSnapshot = new b3LaneSnapshot();
	writer: b3LaneWriter = new b3LaneWriter();
	staging: Uint8Array = new Uint8Array(0);
	shapeStamps: Int32Array = new Int32Array(0);
	bodyStamps: Int32Array = new Int32Array(0);
	stamp: i32 = 0;
	ranges: Int32Array = new Int32Array(B3_MAX_LANES + 1);
	pack: f64 = 0;
	apply: f64 = 0;
	partition0: f64 = 0;
	wait: f64 = 0;
	bytesIn: f64 = 0;
	bytesOut: f64 = 0;
	steps: i32 = 0;
	fallbacks: i32 = 0;
}

// Whether the lanes can run this step's collide tasks: no user callbacks, which cannot cross
export function b3LaneCollideSupported(world: b3World): bool {
	return world.workerCount == 1 && world.preSolveFcn == null && world.frictionCallback == b3DefaultFrictionCallback &&
		world.restitutionCallback == b3DefaultRestitutionCallback;
}

unsafe function b3PackBody(m: b3LaneCollideMain, world: b3World, encoded: i32, bodyId: i32): void {
	let stamps = m.bodyStamps;
	if (stamps[bodyId] == m.stamp) {
		return;
	}
	stamps[bodyId] = m.stamp;
	let w = m.writer;
	let simSize = <i32>sizeof<b3BodySim>();
	let at = w.reserve(BODY_WORDS * 4 + simSize);
	let kind: i32;
	let index: i32;
	let sims: Array<b3BodySim>;
	if (encoded >= 0) {
		kind = BODY_AWAKE;
		index = encoded;
		sims = world.solverSets[b3_awakeSet].bodySims;
	} else if (b3IsStaticSimIndex(encoded)) {
		kind = BODY_STATIC;
		index = -encoded - 2;
		sims = world.solverSets[b3_staticSet].bodySims;
	} else {
		kind = BODY_SLEEPING;
		let body = world.bodies[bodyId];
		index = body.localIndex;
		sims = world.solverSets[body.setIndex].bodySims;
	}
	store<i32>(at, kind);
	store<i32>(at + 4, index);
	store<i32>(at + 8, bodyId);
	memory.copy(at + <usize>(BODY_WORDS * 4), b3ArrayData(sims) + <usize>index * <usize>simSize, <usize>simSize);
}

unsafe function b3PackShape(m: b3LaneCollideMain, world: b3World, shapeId: i32): void {
	let stamps = m.shapeStamps;
	if (stamps[shapeId] == m.stamp) {
		return;
	}
	stamps[shapeId] = m.stamp;
	let at = m.writer.reserve(SHAPE_BYTES);
	store<i32>(at, shapeId);
	store<b3AABB>(at + 4, world.shapes[shapeId].aabb);
	store<b3AABB>(at + 4 + sizeof<b3AABB>(), world.fatAABBs[shapeId]);
}

// Serializes contacts [first, last) of the step's awake contacts as one section
unsafe function b3PackCollideSection(m: b3LaneCollideMain, context: b3StepContext, first: i32, last: i32): void {
	let world = context.world;
	let w = m.writer;
	let ids = context.awakeContactIndices;
	let contacts = world.contacts;
	let sectionStart = w.length;
	m.stamp += 1;

	let head = w.reserve(S_BYTES);
	let maxSleeping = 0;
	for (let s = b3_firstSleepingSet; s < world.solverSets.length; ++s) {
		maxSleeping = max(maxSleeping, world.solverSets[s].bodySims.length);
	}
	store<i32>(head + S_CONTACTS, last - first);
	store<i32>(head + S_CONTACT_CAPACITY, contacts.length);
	store<i32>(head + S_SHAPE_CAPACITY, world.shapes.length);
	store<i32>(head + S_BODY_CAPACITY, world.bodies.length);
	store<f32>(head + S_RECYCLE_DISTANCE, world.contactRecycleDistance);
	store<i32>(head + S_SIMD_WIDTH, world.simdWidth);
	store<f32>(head + S_LENGTH_UNITS, b3GetLengthUnitsPerMeter());
	store<i32>(head + S_AWAKE_CAPACITY, world.solverSets[b3_awakeSet].bodySims.length);
	store<i32>(head + S_STATIC_CAPACITY, world.solverSets[b3_staticSet].bodySims.length);
	store<i32>(head + S_SLEEPING_COUNT, maxSleeping);

	let shapesStart = w.length;
	for (let i = first; i < last; ++i) {
		let contact = contacts[ids[i]];
		b3PackShape(m, world, contact.shapeIdA);
		b3PackShape(m, world, contact.shapeIdB);
	}
	let shapeCount = (w.length - shapesStart) / SHAPE_BYTES;

	let bodiesStart = w.length;
	let bodyCount = 0;
	let bodyRecord = BODY_WORDS * 4 + <i32>sizeof<b3BodySim>();
	for (let i = first; i < last; ++i) {
		let contact = contacts[ids[i]];
		b3PackBody(m, world, contact.encodedBodySimA, contact.edges[0].bodyId);
		b3PackBody(m, world, contact.encodedBodySimB, contact.edges[1].bodyId);
	}
	bodyCount = (w.length - bodiesStart) / bodyRecord;

	for (let i = first; i < last; ++i) {
		let contact = contacts[ids[i]];
		let stateBytes = b3StateBytes(contact);
		let at = w.reserve(C_IDS_WORDS * 4 + stateBytes);
		store<i32>(at, contact.contactId);
		store<i32>(at + 4, contact.shapeIdA);
		store<i32>(at + 8, contact.shapeIdB);
		store<i32>(at + 12, contact.childIndex);
		store<i32>(at + 16, contact.edges[0].bodyId);
		store<i32>(at + 20, contact.edges[1].bodyId);
		store<i32>(at + 24, contact.encodedBodySimA);
		store<i32>(at + 28, contact.encodedBodySimB);
		b3WriteState(at + <usize>(C_IDS_WORDS * 4), contact);
	}
	w.patch(sectionStart + S_SHAPES, <u32>shapeCount);
	w.patch(sectionStart + S_BODIES, <u32>bodyCount);
}

// Packs the other lanes' shares of the step's contacts into one blob, behind a table of section offsets.
// Partition 0 stays on the main lane. Answers the blob, or null when the lanes cannot take this step.
export unsafe function b3LaneCollidePack(m: b3LaneCollideMain, context: b3StepContext, contactCount: i32, lanes: i32): Blob {
	let world = context.world;
	let ticks = b3LaneNow();
	let checksum = b3ShapesChecksum(world);
	let snapshot = m.snapshot;
	if (snapshot.blob == null || checksum != snapshot.checksum) {
		b3BuildSnapshot(snapshot, world);
		snapshot.checksum = checksum;
	}
	m.shapeStamps = b3GrowI32(m.shapeStamps, world.shapes.length);
	m.bodyStamps = b3GrowI32(m.bodyStamps, world.bodies.length);

	for (let p = 0; p <= lanes; ++p) {
		m.ranges[p] = b3SplitStart(contactCount, p, lanes);
	}
	let w = m.writer;
	w.reset();
	w.u32(0x43433342);
	w.u32(<u32>lanes);
	for (let p = 0; p < 2 * B3_MAX_LANES; ++p) {
		w.u32(0);
	}
	for (let p = 1; p < lanes; ++p) {
		let sectionStart = w.length;
		b3PackCollideSection(m, context, m.ranges[p], m.ranges[p + 1]);
		w.patch(8 + 4 * p, <u32>sectionStart);
		w.patch(8 + 4 * (lanes + p), <u32>(w.length - sectionStart));
	}
	m.bytesIn += <f64>w.length;
	let blob = Blob.fromArray(w.buffer, 0, w.length);
	m.pack += b3LaneNow() - ticks;
	return blob;
}

// Copies lane p's updated contacts back, and does what b3CollideTask does with the world's task context for them:
// the state bits, the counters and the mesh contact spec
export unsafe function b3LaneCollideApply(m: b3LaneCollideMain, context: b3StepContext, p: i32, output: Blob): void {
	let ticks = b3LaneNow();
	let world = context.world;
	let taskContext = world.taskContexts[0];
	let contacts = world.contacts;
	let ids = context.awakeContactIndices;
	let colors = world.constraintGraph.colors;
	let stateBits = b3_simDisjoint | b3_simStartedTouching | b3_simStoppedTouching;
	{
		let used = <i32>output.byteLength;
		m.staging = b3GrowU8(m.staging, used);
		let staging = m.staging;
		output.readInto(0, staging, 0, used);
		m.bytesOut += <f64>used;
		let base = staging.dataStart;
		taskContext.satCallCount += load<i32>(base + O_SAT_CALLS);
		taskContext.satCacheHitCount += load<i32>(base + O_SAT_HITS);
		taskContext.recycledContactCount += load<i32>(base + O_RECYCLED);
		for (let j = 0; j < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++j) {
			taskContext.manifoldCounts[j] += load<i32>(base + <usize>(O_MANIFOLD_COUNTS + 4 * j));
		}
		let at = base + <usize>O_BYTES;
		for (let i = m.ranges[p]; i < m.ranges[p + 1]; ++i) {
			let contactId = ids[i];
			let contact = contacts[contactId];
			let wasTouching = (contact.flags & b3_simTouchingFlag) != 0;
			at = b3ReadState(at, contact);
			let flags = contact.flags;
			if ((flags & stateBits) != 0) {
				b3SetBit(taskContext.contactStateBitSet, <u32>contactId);
			}
			let touching = (flags & b3_simTouchingFlag) != 0;
			if (touching && wasTouching && (flags & b3_simMeshContact) != 0) {
				colors[contact.colorIndex].contacts[contact.localIndex].manifoldCount = <u16>contact.manifoldCount;
			}
		}
	}
	m.apply += b3LaneNow() - ticks;
}
