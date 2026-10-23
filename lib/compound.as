// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/compound.c

import { B3_ASSERT, B3_ASSERT_ENABLED, UINT16_MAX } from "./core";
import { B3_MAX_CHILD_SHAPES, B3_MAX_SHAPE_CAST_POINTS } from "./constants";
import {
	b3AABB,
	b3AABB_Transform,
	b3Add,
	b3InvRotateVector,
	b3InvTransformPoint,
	b3MakeAABB,
	b3Max,
	b3Min,
	b3MulTransforms,
	b3RotateVector,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3AlignUp8 } from "./math_internal";
import {
	B3_COMPOUND_DATA_SIZE,
	B3_COMPOUND_VERSION,
	b3BoxCastInput,
	b3Capsule,
	b3CastOutput,
	b3ChildShape,
	b3CompoundCapsule,
	b3CompoundData,
	b3CompoundDef,
	b3CompoundHull,
	b3CompoundMesh,
	b3CompoundQueryFcn,
	b3CompoundSphere,
	b3HullData,
	b3HullInstance,
	b3Mesh,
	b3MeshData,
	b3MeshInstance,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeProxy,
	b3Sweep,
	b3TreeNode,
} from "./collision";
import {
	B3_DEFAULT_MASK_BITS,
	b3SurfaceMaterial,
	b3_capsuleShape,
	b3_hullShape,
	b3_meshShape,
	b3_sphereShape,
} from "./types";
import {
	b3DynamicTree_Create,
	b3DynamicTree_CreateProxy,
	b3DynamicTree_Destroy,
	b3DynamicTree_GetHeight,
	b3DynamicTree_GetRootBounds,
	b3DynamicTree_Rebuild,
	b3TreeView,
	b3TreeView_BoxCast,
	b3TreeView_Query,
	b3TreeView_RayCast,
} from "./dynamic_tree";
import {
	b3CollideMoverAndHull,
	b3ComputeHullAABB,
	b3HullMap,
	b3HullMap_cleanup,
	b3HullMap_get_or_insert,
	b3HullMap_init,
	b3HullMap_reserve,
	b3HullMap_val,
	b3OverlapHull,
	b3RayCastHull,
	b3ShapeCastHull,
} from "./hull";
import { b3CollideMoverAndMesh, b3ComputeMeshAABB, b3GetMeshNodeBits, b3OverlapMesh, b3RayCastMesh, b3ShapeCastMesh } from "./mesh";
import { b3CollideMoverAndCapsule, b3ComputeCapsuleAABB, b3OverlapCapsule, b3RayCastCapsule, b3ShapeCastCapsule } from "./capsule";
import { b3CollideMoverAndSphere, b3ComputeSphereAABB, b3OverlapSphere, b3RayCastSphere, b3ShapeCastSphere } from "./sphere";
import { b3ComputeProxyAABB, b3MakeLocalProxy } from "./shape";

export function b3GetCompoundTreeView(compound: b3CompoundData): b3TreeView {
	let view = new b3TreeView(compound.nodes, null);
	return view;
}

// functions from compound.c follow

// C byte sizes, so the offsets and byteCount match the C blob
const B3_TREE_NODE_BYTES: i32 = 32;
const B3_SURFACE_MATERIAL_BYTES: i32 = 40;
const B3_COMPOUND_CAPSULE_BYTES: i32 = 32;
const B3_HULL_INSTANCE_BYTES: i32 = 36;
const B3_MESH_INSTANCE_BYTES: i32 = 52;
const B3_COMPOUND_SPHERE_BYTES: i32 = 20;

// C returns NULL when the material offset is zero.
export function b3GetCompoundMaterials(compound: b3CompoundData): StaticArray<b3SurfaceMaterial>? {
	if (compound.materialOffset == 0) {
		return null;
	}

	return compound.materials;
}

export function b3GetCompoundCapsule(compound: b3CompoundData, index: i32): b3CompoundCapsule {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < compound.capsuleCount && compound.capsuleOffset > 0);

	let result = b3CompoundCapsule();
	if (compound.capsuleOffset == 0) {
		return result;
	}

	return compound.capsules[index];
}

export function b3GetCompoundHull(compound: b3CompoundData, index: i32): b3CompoundHull {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < compound.hullCount && compound.hullOffset > 0);

	let result = new b3CompoundHull();
	if (compound.hullOffset == 0) {
		return result;
	}

	let hullInstances = compound.hullInstances;
	let hullOffset = hullInstances[index].hullOffset;
	result.hull = compound.hulls[<i32>hullOffset];
	result.transform = hullInstances[index].transform;
	result.materialIndex = hullInstances[index].materialIndex;
	return result;
}

export function b3GetCompoundMesh(compound: b3CompoundData, index: i32): b3CompoundMesh {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < compound.meshCount && compound.meshOffset > 0);

	let result = new b3CompoundMesh();
	if (compound.meshOffset == 0) {
		return result;
	}

	let meshInstances = compound.meshInstances;
	let meshOffset = meshInstances[index].meshOffset;
	result.meshData = compound.meshes[<i32>meshOffset];
	result.transform = meshInstances[index].transform;
	result.scale = meshInstances[index].scale;
	result.materialIndices = compound.meshMaterialIndices.subarray(<i32>meshInstances[index].materialOffset);
	result.materialCount = <i32>meshInstances[index].materialCount;
	return result;
}

export function b3GetCompoundSphere(compound: b3CompoundData, index: i32): b3CompoundSphere {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < compound.sphereCount && compound.sphereOffset > 0);

	let result = b3CompoundSphere();
	if (compound.sphereOffset == 0) {
		return result;
	}

	return compound.spheres[index];
}

// C returns the child by value. This fills a caller owned child so hot callers reuse one; a convex
// child points its material indices at convexMaterial, a one element array the caller owns.
export function b3GetCompoundChildInto(
	compound: b3CompoundData,
	childIndex: i32,
	child: b3ChildShape,
	convexMaterial: Uint16Array,
): void {
	child.hull = null;
	child.mesh = null;

	// Capsule?
	if (0 <= childIndex && childIndex < compound.capsuleCount) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(compound.capsuleOffset > 0);
		let capsules = compound.capsules;
		child.capsule = capsules[childIndex].capsule;
		child.transform = b3Transform_identity;
		convexMaterial[0] = capsules[childIndex].materialIndex;
		child.materialIndices = convexMaterial;
		child.materialCount = 1;
		child.type = b3_capsuleShape;
		return;
	}
	childIndex -= compound.capsuleCount;

	// Hull?
	if (0 <= childIndex && childIndex < compound.hullCount) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(compound.hullOffset > 0);
		let hullInstances = compound.hullInstances;
		let hullOffset = hullInstances[childIndex].hullOffset;
		child.hull = compound.hulls[<i32>hullOffset];
		child.transform = hullInstances[childIndex].transform;
		convexMaterial[0] = hullInstances[childIndex].materialIndex;
		child.materialIndices = convexMaterial;
		child.materialCount = 1;
		child.type = b3_hullShape;
		return;
	}
	childIndex -= compound.hullCount;

	// Mesh?
	if (0 <= childIndex && childIndex < compound.meshCount) {
		let compoundMesh = b3GetCompoundMesh(compound, childIndex);

		child.mesh = new b3Mesh(compoundMesh.meshData!, compoundMesh.scale);
		child.transform = compoundMesh.transform;
		child.materialIndices = compoundMesh.materialIndices;
		child.materialCount = compoundMesh.materialCount;
		child.type = b3_meshShape;
		return;
	}
	childIndex -= compound.meshCount;

	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= childIndex && childIndex < compound.sphereCount);

	// Sphere?
	{
		if (B3_ASSERT_ENABLED) B3_ASSERT(compound.sphereOffset > 0);
		let spheres = compound.spheres;
		child.sphere = spheres[childIndex].sphere;
		child.transform = b3Transform_identity;
		convexMaterial[0] = spheres[childIndex].materialIndex;
		child.materialIndices = convexMaterial;
		child.materialCount = 1;
		child.type = b3_sphereShape;
	}
}

export function b3GetCompoundChild(compound: b3CompoundData, childIndex: i32): b3ChildShape {
	let child = new b3ChildShape();
	b3GetCompoundChildInto(compound, childIndex, child, new Uint16Array(1));
	return child;
}

// The compound queries below hold their child only until the tree callback returns, and nothing
// they call reads another compound child, so they share one.
const s_queryChild = new b3ChildShape();
const s_queryChildMaterial = new Uint16Array(1);

@inline function b3GetQueryChild(compound: b3CompoundData, childIndex: i32): b3ChildShape {
	b3GetCompoundChildInto(compound, childIndex, s_queryChild, s_queryChildMaterial);
	return s_queryChild;
}

// C memcmp's the mesh blobs: the header, every section and the zero padding. This compares the same bits.
function b3CompareMeshes(mesh1: b3MeshData, mesh2: b3MeshData): bool {
	if (mesh1 == mesh2) {
		return true;
	}

	if (mesh1.byteCount != mesh2.byteCount) {
		return false;
	}

	if (
		mesh1.version != mesh2.version ||
		mesh1.hash != mesh2.hash ||
		b3BitsDiffer(mesh1.bounds.lowerBound, mesh2.bounds.lowerBound) ||
		b3BitsDiffer(mesh1.bounds.upperBound, mesh2.bounds.upperBound) ||
		reinterpret<u32>(mesh1.surfaceArea) != reinterpret<u32>(mesh2.surfaceArea) ||
		mesh1.treeHeight != mesh2.treeHeight ||
		mesh1.degenerateCount != mesh2.degenerateCount ||
		mesh1.nodeOffset != mesh2.nodeOffset ||
		mesh1.nodeCount != mesh2.nodeCount ||
		mesh1.vertexOffset != mesh2.vertexOffset ||
		mesh1.vertexCount != mesh2.vertexCount ||
		mesh1.triangleOffset != mesh2.triangleOffset ||
		mesh1.triangleCount != mesh2.triangleCount ||
		mesh1.materialOffset != mesh2.materialOffset ||
		mesh1.materialCount != mesh2.materialCount ||
		mesh1.flagsOffset != mesh2.flagsOffset ||
		mesh1.padding != mesh2.padding
	) {
		return false;
	}

	let nodes1 = mesh1.nodes;
	let nodes2 = mesh2.nodes;
	for (let i = 0; i < mesh1.nodeCount; ++i) {
		if (
			b3BitsDiffer(nodes1[i].lowerBound, nodes2[i].lowerBound) ||
			b3BitsDiffer(nodes1[i].upperBound, nodes2[i].upperBound) ||
			b3GetMeshNodeBits(nodes1[i]) != b3GetMeshNodeBits(nodes2[i]) ||
			nodes1[i].triangleOffset != nodes2[i].triangleOffset
		) {
			return false;
		}
	}

	let vertices1 = mesh1.vertices;
	let vertices2 = mesh2.vertices;
	for (let i = 0; i < mesh1.vertexCount; ++i) {
		if (b3BitsDiffer(vertices1[i], vertices2[i])) {
			return false;
		}
	}

	let triangles1 = mesh1.triangles;
	let triangles2 = mesh2.triangles;
	for (let i = 0; i < mesh1.triangleCount; ++i) {
		if (
			triangles1[i].index1 != triangles2[i].index1 ||
			triangles1[i].index2 != triangles2[i].index2 ||
			triangles1[i].index3 != triangles2[i].index3
		) {
			return false;
		}
	}

	for (let i = 0; i < mesh1.triangleCount; ++i) {
		if (mesh1.materialIndices[i] != mesh2.materialIndices[i] || mesh1.flags[i] != mesh2.flags[i]) {
			return false;
		}
	}

	return true;
}

function b3BitsDiffer(a: b3Vec3, b: b3Vec3): bool {
	return (
		reinterpret<u32>(a.x) != reinterpret<u32>(b.x) ||
		reinterpret<u32>(a.y) != reinterpret<u32>(b.y) ||
		reinterpret<u32>(a.z) != reinterpret<u32>(b.z)
	);
}

// C's verstable maps keyed by content become hash buckets plus a content compare. Values are
// assigned in insertion order, so the bucket order never reaches results.
class b3MeshMap {
	buckets: Map<u64, Array<i32>> = new Map<u64, Array<i32>>();
	keys: Array<b3MeshData> = new Array<b3MeshData>();
	vals: Array<i32> = new Array<i32>();
}

// Returns the value of the matching key, or inserts the key with val and returns val.
function b3MeshMap_get_or_insert(map: b3MeshMap, key: b3MeshData, val: i32): i32 {
	let hash = key.hash;
	if (map.buckets.has(hash)) {
		let bucket = map.buckets.get(hash);
		for (let i = 0; i < bucket.length; ++i) {
			let slot = bucket[i];
			if (b3CompareMeshes(map.keys[slot], key)) {
				return map.vals[slot];
			}
		}
	} else {
		map.buckets.set(hash, new Array<i32>());
	}

	map.buckets.get(hash).push(map.keys.length);
	map.keys.push(key);
	map.vals.push(val);
	return val;
}

// FNV-1a over the C byte image of the 40 byte material, standing in for vt_wyhash.
function b3HashMaterial(material: b3SurfaceMaterial): u64 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(material.padding == 0);

	let hash: u64 = 0xcbf29ce484222325;
	hash = (hash ^ <u64>reinterpret<u32>(material.friction)) * 0x100000001b3;
	hash = (hash ^ <u64>reinterpret<u32>(material.restitution)) * 0x100000001b3;
	hash = (hash ^ <u64>reinterpret<u32>(material.rollingResistance)) * 0x100000001b3;
	hash = (hash ^ <u64>reinterpret<u32>(material.tangentVelocity.x)) * 0x100000001b3;
	hash = (hash ^ <u64>reinterpret<u32>(material.tangentVelocity.y)) * 0x100000001b3;
	hash = (hash ^ <u64>reinterpret<u32>(material.tangentVelocity.z)) * 0x100000001b3;
	hash = (hash ^ material.userMaterialId) * 0x100000001b3;
	hash = (hash ^ <u64>material.customColor) * 0x100000001b3;
	hash = (hash ^ <u64>material.padding) * 0x100000001b3;
	return hash;
}

function b3CompareMaterials(mat1: b3SurfaceMaterial, mat2: b3SurfaceMaterial): bool {
	if (B3_ASSERT_ENABLED) B3_ASSERT(mat1.padding == 0 && mat2.padding == 0);

	return (
		reinterpret<u32>(mat1.friction) == reinterpret<u32>(mat2.friction) &&
		reinterpret<u32>(mat1.restitution) == reinterpret<u32>(mat2.restitution) &&
		reinterpret<u32>(mat1.rollingResistance) == reinterpret<u32>(mat2.rollingResistance) &&
		b3BitsDiffer(mat1.tangentVelocity, mat2.tangentVelocity) == false &&
		mat1.userMaterialId == mat2.userMaterialId &&
		mat1.customColor == mat2.customColor &&
		mat1.padding == mat2.padding
	);
}

class b3MaterialMap {
	buckets: Map<u64, Array<i32>> = new Map<u64, Array<i32>>();
	keys: Array<b3SurfaceMaterial> = new Array<b3SurfaceMaterial>();
	vals: Array<i32> = new Array<i32>();
}

// Returns the value of the matching key, or inserts the key with val and returns val.
function b3MaterialMap_get_or_insert(map: b3MaterialMap, key: b3SurfaceMaterial, val: i32): i32 {
	let hash = b3HashMaterial(key);
	if (map.buckets.has(hash)) {
		let bucket = map.buckets.get(hash);
		for (let i = 0; i < bucket.length; ++i) {
			let slot = bucket[i];
			if (b3CompareMaterials(map.keys[slot], key)) {
				return map.vals[slot];
			}
		}
	} else {
		map.buckets.set(hash, new Array<i32>());
	}

	map.buckets.get(hash).push(map.keys.length);
	map.keys.push(key);
	map.vals.push(val);
	return val;
}

// Shared hulls and meshes are referenced, not copied into one blob, and instances hold their
// indices. The offsets and byteCount are computed as C lays out its blob.
export function b3CreateCompound(def: b3CompoundDef): b3CompoundData? {
	let convexCount = def.capsuleCount + def.hullCount + def.sphereCount;
	let shapeCount = convexCount + def.meshCount;

	if (shapeCount >= B3_MAX_CHILD_SHAPES) {
		B3_ASSERT(false);
		return null;
	}

	let tree = b3DynamicTree_Create(shapeCount);

	let childIndex = 0;

	// Instances
	let capsuleCount = def.capsuleCount;
	let capsuleInstances = new StaticArray<b3CompoundCapsule>(capsuleCount);
	let hullCount = def.hullCount;
	let hullInstances = new StaticArray<b3HullInstance>(hullCount);
	let meshCount = def.meshCount;
	let meshInstances = new StaticArray<b3MeshInstance>(meshCount);
	let sphereCount = def.sphereCount;
	let sphereInstances = new StaticArray<b3CompoundSphere>(sphereCount);

	// Determine material capacity.
	let materialCapacity = convexCount;
	for (let i = 0; i < def.meshCount; ++i) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(def.meshes[i].materialCount > 0);
		materialCapacity += def.meshes[i].materialCount;
	}

	// Material map for convex material sharing. Mesh materials are not shared for simplicity.
	let materialMap = new b3MaterialMap();
	let materials = new StaticArray<b3SurfaceMaterial>(materialCapacity);
	let materialCount = 0;

	let meshMaterialTotal = materialCapacity - convexCount;
	let meshMaterialStaging = new Uint16Array(meshMaterialTotal);
	let meshMaterialCursor = 0;

	for (let i = 0; i < def.capsuleCount; ++i) {
		let capsuleDef = def.capsules[i];
		capsuleInstances[i].capsule = capsuleDef.capsule;

		// Look for an existing material.
		// Get the shared material index.
		let materialIndex = b3MaterialMap_get_or_insert(materialMap, capsuleDef.material, materialCount);
		capsuleInstances[i].materialIndex = <u16>materialIndex;

		// Is this a new material?
		if (materialIndex == materialCount) {
			materials[materialIndex] = capsuleDef.material;
			materialCount += 1;
		}

		let aabb = b3ComputeCapsuleAABB(capsuleDef.capsule, b3Transform_identity);
		b3DynamicTree_CreateProxy(tree, aabb, ~(<u64>0), <u64>childIndex);
		childIndex += 1;
	}

	// Hulls
	let sharedHulls = new Array<b3HullData>();
	let sharedHullCount = 0;

	if (hullCount > 0) {
		let hullMap = new b3HullMap();
		b3HullMap_init(hullMap);
		b3HullMap_reserve(hullMap, hullCount);

		for (let i = 0; i < hullCount; ++i) {
			let hullDef = def.hulls[i];
			let hull = hullDef.hull;
			let aabb = b3ComputeHullAABB(hull, hullDef.transform);
			b3DynamicTree_CreateProxy(tree, aabb, ~(<u64>0), <u64>childIndex);
			childIndex += 1;

			// Look for an existing material.
			// Get the shared material index.
			let materialIndex = b3MaterialMap_get_or_insert(materialMap, hullDef.material, materialCount);
			hullInstances[i].materialIndex = <u16>materialIndex;

			// Is this a new material?
			if (materialIndex == materialCount) {
				materials[materialIndex] = hullDef.material;
				materialCount += 1;
			}

			hullInstances[i].transform = hullDef.transform;

			// Look for an existing matching hull
			let itr = b3HullMap_get_or_insert(hullMap, hull, sharedHullCount);

			// Get the unique index for this hull
			let sharedHullIndex = b3HullMap_val(hullMap, itr);

			// C stores the index here and replaces it with the blob offset below. The port keeps the index.
			hullInstances[i].hullOffset = <u32>sharedHullIndex;

			// Is this a new hull?
			if (sharedHullIndex == sharedHullCount) {
				// Create a shared hull.
				sharedHulls.push(hull);
				sharedHullCount += 1;
			}
		}

		b3HullMap_cleanup(hullMap);
	}

	// Meshes
	let sharedMeshes = new Array<b3MeshData>();
	let sharedMeshCount = 0;

	if (meshCount > 0) {
		let meshMap = new b3MeshMap();

		for (let i = 0; i < meshCount; ++i) {
			let meshDef = def.meshes[i];

			let meshData = meshDef.meshData;
			let aabb = b3ComputeMeshAABB(meshData, meshDef.transform, meshDef.scale);
			b3DynamicTree_CreateProxy(tree, aabb, ~(<u64>0), <u64>childIndex);
			childIndex += 1;

			// No effort to share mesh materials. It would be easier to do if the number of materials was limited.
			if (B3_ASSERT_ENABLED) B3_ASSERT(meshData.materialCount == meshDef.materialCount);

			let meshMaterialStart = meshMaterialCursor;

			for (let j = 0; j < meshDef.materialCount; ++j) {
				// Look for an existing material.
				// Get the shared material index.
				let materialIndex = b3MaterialMap_get_or_insert(materialMap, meshDef.materials[j], materialCount);
				meshMaterialStaging[meshMaterialCursor] = <u16>materialIndex;
				meshMaterialCursor += 1;

				// Is this a new material?
				if (materialIndex == materialCount) {
					materials[materialIndex] = meshDef.materials[j];
					materialCount += 1;
				}
			}

			// Look for an existing matching mesh.
			// Get the shared mesh index.
			let sharedMeshIndex = b3MeshMap_get_or_insert(meshMap, meshData, sharedMeshCount);

			// Create mesh instance.
			meshInstances[i].transform = def.meshes[i].transform;
			meshInstances[i].scale = def.meshes[i].scale;

			// C turns these into byte offsets below. The port keeps the element index and the shared index.
			meshInstances[i].materialOffset = <u32>meshMaterialStart;
			meshInstances[i].materialCount = <u32>meshDef.materialCount;
			meshInstances[i].meshOffset = <u32>sharedMeshIndex;

			// Is this a new mesh?
			if (sharedMeshIndex == sharedMeshCount) {
				// Create a shared mesh.
				sharedMeshes.push(meshData);
				sharedMeshCount += 1;
			}
		}
	}

	// Spheres
	for (let i = 0; i < def.sphereCount; ++i) {
		let sphereDef = def.spheres[i];
		sphereInstances[i].sphere = sphereDef.sphere;

		// Look for an existing material.
		// Get the shared material index.
		let materialIndex = b3MaterialMap_get_or_insert(materialMap, sphereDef.material, materialCount);
		sphereInstances[i].materialIndex = <u16>materialIndex;

		// Is this a new material?
		if (materialIndex == materialCount) {
			materials[materialIndex] = sphereDef.material;
			materialCount += 1;
		}

		let aabb = b3ComputeSphereAABB(sphereDef.sphere, b3Transform_identity);
		b3DynamicTree_CreateProxy(tree, aabb, ~(<u64>0), <u64>childIndex);
		childIndex += 1;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(materialCount <= materialCapacity);
	if (B3_ASSERT_ENABLED) B3_ASSERT(materialCount <= UINT16_MAX);
	if (B3_ASSERT_ENABLED) B3_ASSERT(tree.proxyCount > 0);

	b3DynamicTree_Rebuild(tree, true);

	// Tree nodes. The rebuild is dense, so only the live nodes travel.
	let byteCount = b3AlignUp8(B3_COMPOUND_DATA_SIZE);
	let nodeOffset = byteCount;
	byteCount += b3AlignUp8(tree.nodeEnd * B3_TREE_NODE_BYTES);
	let materialOffset = byteCount;
	byteCount += b3AlignUp8(materialCount * B3_SURFACE_MATERIAL_BYTES);
	let capsuleOffset = byteCount;
	byteCount += b3AlignUp8(def.capsuleCount * B3_COMPOUND_CAPSULE_BYTES);

	// Hull data layout has another level of indirection to allow for tight data packing
	// 1. hull instance array : hull count array of b3HullInstance with individual hull transforms and offsets
	// 2. heterogeneous array of shared hull data : each shared hull can have a different byte count, so direct indexing is not
	// possible
	let hullArrayOffset = byteCount;

	// Array of hull instances
	byteCount += b3AlignUp8(hullCount * B3_HULL_INSTANCE_BYTES);

	// Packed shared hull blobs
	for (let i = 0; i < sharedHullCount; ++i) {
		byteCount += b3AlignUp8(sharedHulls[i].byteCount);
	}

	// Mesh data layout has another level of indirection to allow for tight data packing
	// 1. mesh instance array : mesh count array of b3MeshInstance with individual mesh transform, scale, and offset
	// 2. heterogeneous array of shared mesh data : each shared mesh can have a different byte count, so direct indexing is not
	// possible
	let meshArrayOffset = byteCount;

	// Array of mesh instances
	byteCount += b3AlignUp8(meshCount * B3_MESH_INSTANCE_BYTES);

	byteCount += b3AlignUp8(meshMaterialTotal * 2);

	// Packed shared mesh blobs
	for (let i = 0; i < sharedMeshCount; ++i) {
		byteCount += b3AlignUp8(sharedMeshes[i].byteCount);
	}

	let sphereOffset = byteCount;
	byteCount += b3AlignUp8(def.sphereCount * B3_COMPOUND_SPHERE_BYTES);

	let compound = new b3CompoundData();

	compound.version = B3_COMPOUND_VERSION;
	compound.byteCount = byteCount;
	compound.nodeOffset = nodeOffset;
	compound.bounds = b3DynamicTree_GetRootBounds(tree);
	compound.treeHeight = b3DynamicTree_GetHeight(tree);
	compound.nodeCount = tree.nodeEnd;
	compound.materialOffset = materialOffset;
	compound.materialCount = materialCount;
	compound.capsuleOffset = capsuleOffset;
	compound.capsuleCount = capsuleCount;
	compound.hullOffset = hullArrayOffset;
	compound.hullCount = hullCount;
	compound.meshOffset = meshArrayOffset;
	compound.meshCount = meshCount;
	compound.sphereOffset = sphereOffset;
	compound.sphereCount = sphereCount;

	// Tree nodes
	let nodes = new StaticArray<b3TreeNode>(tree.nodeEnd);
	for (let i = 0; i < tree.nodeEnd; ++i) {
		nodes[i] = tree.nodes[i];
	}
	compound.nodes = nodes;

	// Materials
	if (B3_ASSERT_ENABLED) B3_ASSERT(materialCount > 0);
	let destinationMaterials = new StaticArray<b3SurfaceMaterial>(materialCount);
	for (let i = 0; i < materialCount; ++i) {
		destinationMaterials[i] = materials[i];
	}
	compound.materials = destinationMaterials;

	// Capsules
	compound.capsules = capsuleInstances;

	// Hulls
	compound.hullInstances = hullInstances;

	let destinationHulls = new StaticArray<b3HullData>(sharedHullCount);
	for (let i = 0; i < sharedHullCount; ++i) {
		destinationHulls[i] = sharedHulls[i];
	}
	compound.hulls = destinationHulls;

	compound.sharedHullCount = sharedHullCount;

	// Meshes
	compound.meshInstances = meshInstances;
	compound.meshMaterialIndices = meshMaterialStaging;

	let destinationMeshes = new StaticArray<b3MeshData>(sharedMeshCount);
	for (let i = 0; i < sharedMeshCount; ++i) {
		destinationMeshes[i] = sharedMeshes[i];
	}
	compound.meshes = destinationMeshes;

	compound.sharedMeshCount = sharedMeshCount;

	// Spheres
	compound.spheres = sphereInstances;

	b3DynamicTree_Destroy(tree);

	return compound;
}

export function b3DestroyCompound(compound: b3CompoundData): void {}

export function b3ComputeCompoundAABB(shape: b3CompoundData, transform: b3Transform): b3AABB {
	return b3AABB_Transform(transform, shape.bounds);
}

class b3CompoundOverlapContext {
	compound: b3CompoundData;
	// proxy in the compound frame
	proxy: b3ShapeProxy;
	overlap: bool = false;

	constructor(compound: b3CompoundData, proxy: b3ShapeProxy) {
		this.compound = compound;
		this.proxy = proxy;
	}
}

function b3CompoundOverlapCallback(proxyId: i32, userData: u64, overlapContext: b3CompoundOverlapContext): bool {
	let childIndex = <i32>userData;
	let child = b3GetQueryChild(overlapContext.compound, childIndex);

	let overlap = false;
	switch (child.type) {
		case b3_capsuleShape:
			overlap = b3OverlapCapsule(child.capsule, child.transform, overlapContext.proxy);
			break;

		case b3_hullShape:
			overlap = b3OverlapHull(child.hull!, child.transform, overlapContext.proxy);
			break;

		case b3_meshShape:
			overlap = b3OverlapMesh(child.mesh!, child.transform, overlapContext.proxy);
			break;

		case b3_sphereShape:
			overlap = b3OverlapSphere(child.sphere, child.transform, overlapContext.proxy);
			break;

		default:
			B3_ASSERT(false);
			break;
	}

	if (overlap) {
		// Done
		overlapContext.overlap = true;
		return false;
	}

	// Continue the query if there is no overlap
	return true;
}

// This is dealing with multiple transforms:
// - the compound shape transform
// - the compound child shape transforms
export function b3OverlapCompound(shape: b3CompoundData, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 < proxy.count && proxy.count <= B3_MAX_SHAPE_CAST_POINTS);

	// Use local proxy.
	let buffer = new StaticArray<b3Vec3>(B3_MAX_SHAPE_CAST_POINTS);
	let context = new b3CompoundOverlapContext(shape, b3MakeLocalProxy(proxy, shapeTransform, buffer));

	let aabb = b3ComputeProxyAABB(context.proxy);

	// This query must be in the compound frame.
	let view = b3GetCompoundTreeView(shape);
	b3TreeView_Query(view, aabb, ~(<u64>0), false, (proxyId: i32, userData: u64): bool =>
		b3CompoundOverlapCallback(proxyId, userData, context),
	);

	return context.overlap;
}

class b3CompoundCastContext {
	compound: b3CompoundData;
	output: b3CastOutput = b3CastOutput();
	// origin of the shape cast, the box cast callback only carries the advancing fraction
	shapeInput: b3ShapeCastInput? = null;

	constructor(compound: b3CompoundData) {
		this.compound = compound;
	}
}

function b3CompoundRayCastCallback(input: b3RayCastInput, proxyId: i32, userData: u64, castContext: b3CompoundCastContext): f32 {
	let compound = castContext.compound;

	let childIndex = <i32>userData;

	let child = b3GetQueryChild(compound, childIndex);
	let materialIndices = child.materialIndices!;

	// Get the input in child local space.
	let localInput = input;
	localInput.origin = b3InvTransformPoint(child.transform, input.origin);
	localInput.translation = b3InvRotateVector(child.transform.q, input.translation);

	let output = b3CastOutput();

	switch (child.type) {
		case b3_capsuleShape:
			output = b3RayCastCapsule(child.capsule, localInput);
			output.materialIndex = <i32>materialIndices[0];
			break;

		case b3_hullShape:
			output = b3RayCastHull(child.hull!, localInput);
			output.materialIndex = <i32>materialIndices[0];
			break;

		case b3_meshShape:
			{
				output = b3RayCastMesh(child.mesh!, localInput);
				if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= output.materialIndex && output.materialIndex < child.materialCount);
				output.materialIndex = <i32>materialIndices[output.materialIndex];
			}
			break;

		case b3_sphereShape:
			output = b3RayCastSphere(child.sphere, localInput);
			output.materialIndex = <i32>materialIndices[0];
			break;

		default:
			B3_ASSERT(false);
			break;
	}

	if (output.hit) {
		output.point = b3TransformPoint(child.transform, output.point);
		output.normal = b3RotateVector(child.transform.q, output.normal);
		output.childIndex = childIndex;
		castContext.output = output;
		return output.fraction;
	}

	return input.maxFraction;
}

export function b3RayCastCompound(shape: b3CompoundData, input: b3RayCastInput): b3CastOutput {
	let context = new b3CompoundCastContext(shape);
	let view = b3GetCompoundTreeView(shape);
	b3TreeView_RayCast(view, input, ~(<u64>0), false, (treeInput: b3RayCastInput, proxyId: i32, userData: u64): f32 =>
		b3CompoundRayCastCallback(treeInput, proxyId, userData, context),
	);
	return context.output;
}

// C's stack locals of the shape cast callback; as with the child, one call never nests in another
const s_childCastInput = new b3ShapeCastInput();
const s_childCastPoints = new StaticArray<b3Vec3>(B3_MAX_SHAPE_CAST_POINTS);

function b3CompoundShapeCastCallback(input: b3BoxCastInput, proxyId: i32, userData: u64, castContext: b3CompoundCastContext): f32 {
	let compound = castContext.compound;
	let shapeInput = castContext.shapeInput!;

	let childIndex = <i32>userData;

	let child = b3GetQueryChild(compound, childIndex);
	let materialIndices = child.materialIndices!;

	// Get the input in the child local space.
	let localInput = s_childCastInput;
	localInput.proxy = b3MakeLocalProxy(shapeInput.proxy, child.transform, s_childCastPoints);
	localInput.translation = b3InvRotateVector(child.transform.q, shapeInput.translation);
	localInput.maxFraction = input.maxFraction;
	localInput.canEncroach = shapeInput.canEncroach;

	let output = b3CastOutput();

	switch (child.type) {
		case b3_capsuleShape:
			output = b3ShapeCastCapsule(child.capsule, localInput);
			output.materialIndex = <i32>materialIndices[0];
			break;

		case b3_hullShape:
			output = b3ShapeCastHull(child.hull!, localInput);
			output.materialIndex = <i32>materialIndices[0];
			break;

		case b3_meshShape:
			{
				output = b3ShapeCastMesh(child.mesh!, localInput);
				if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= output.materialIndex && output.materialIndex < child.materialCount);
				output.materialIndex = <i32>materialIndices[output.materialIndex];
			}
			break;

		case b3_sphereShape:
			output = b3ShapeCastSphere(child.sphere, localInput);
			output.materialIndex = <i32>materialIndices[0];
			break;

		default:
			B3_ASSERT(false);
			break;
	}

	if (output.hit) {
		output.point = b3TransformPoint(child.transform, output.point);
		output.normal = b3RotateVector(child.transform.q, output.normal);
		output.childIndex = childIndex;
		castContext.output = output;
		return output.fraction;
	}

	return input.maxFraction;
}

export function b3ShapeCastCompound(shape: b3CompoundData, input: b3ShapeCastInput): b3CastOutput {
	let result = b3CastOutput();

	if (input.proxy.count == 0) {
		return result;
	}

	let context = new b3CompoundCastContext(shape);
	context.shapeInput = input;

	// The compound tree is in the compound local frame, so the proxy box needs no origin offset
	let box = b3MakeAABB(input.proxy.points, input.proxy.count, input.proxy.radius);
	let treeInput = b3BoxCastInput(box, input.translation, input.maxFraction);
	let view = b3GetCompoundTreeView(shape);
	b3TreeView_BoxCast(view, treeInput, ~(<u64>0), false, (boxInput: b3BoxCastInput, proxyId: i32, userData: u64): f32 =>
		b3CompoundShapeCastCallback(boxInput, proxyId, userData, context),
	);
	return context.output;
}

export function b3QueryCompound(compound: b3CompoundData, aabb: b3AABB, fcn: b3CompoundQueryFcn): void {
	let view = b3GetCompoundTreeView(compound);
	b3TreeView_Query(view, aabb, B3_DEFAULT_MASK_BITS, false, (proxyId: i32, userData: u64): bool => fcn(compound, <i32>userData));
}

// xf = xfP * xfC
export function b3MakeCompoundChildSweep(compoundTransform: b3Transform, childTransform: b3Transform): b3Sweep {
	let xf = b3MulTransforms(compoundTransform, childTransform);
	let sweep = b3Sweep();
	sweep.localCenter = b3Vec3_zero;
	sweep.c1 = xf.p;
	sweep.c2 = xf.p;
	sweep.q1 = xf.q;
	sweep.q2 = xf.q;
	return sweep;
}

class b3CompoundMoverContext {
	compound: b3CompoundData;
	planes: StaticArray<b3PlaneResult>;
	planeCapacity: i32;
	planeCount: i32 = 0;
	mover: b3Capsule;

	constructor(compound: b3CompoundData, planes: StaticArray<b3PlaneResult>, planeCapacity: i32, mover: b3Capsule) {
		this.compound = compound;
		this.planes = planes;
		this.planeCapacity = planeCapacity;
		this.mover = mover;
	}
}

function b3CompoundMoverCallback(proxyId: i32, userData: u64, moverContext: b3CompoundMoverContext): bool {
	let childIndex = <i32>userData;
	let child = b3GetQueryChild(moverContext.compound, childIndex);
	let materialIndices = child.materialIndices!;

	// Transform mover to child space
	let localMover = b3Capsule();
	localMover.center1 = b3InvTransformPoint(child.transform, moverContext.mover.center1);
	localMover.center2 = b3InvTransformPoint(child.transform, moverContext.mover.center2);
	localMover.radius = moverContext.mover.radius;

	let capacity = moverContext.planeCapacity - moverContext.planeCount;
	if (B3_ASSERT_ENABLED) B3_ASSERT(capacity > 0);

	// C's planes pointer starts at planeOffset
	let planes = moverContext.planes;
	let planeOffset = moverContext.planeCount;
	let planeCount = 0;

	switch (child.type) {
		case b3_capsuleShape:
			planeCount = b3CollideMoverAndCapsule(planes[planeOffset], child.capsule, localMover);
			break;

		case b3_hullShape:
			planeCount = b3CollideMoverAndHull(planes[planeOffset], child.hull!, localMover);
			break;

		case b3_meshShape:
			planeCount = b3CollideMoverAndMesh(planes, planeOffset, capacity, child.mesh!, localMover);
			break;

		case b3_sphereShape:
			planeCount = b3CollideMoverAndSphere(planes[planeOffset], child.sphere, localMover);
			break;

		default:
			B3_ASSERT(false);
			break;
	}

	// Transform results back to shape space
	for (let i = 0; i < planeCount; ++i) {
		ref plane = planes[planeOffset + i];
		plane.plane.normal = b3RotateVector(child.transform.q, plane.plane.normal);
		plane.point = b3TransformPoint(child.transform, plane.point);

		plane.childIndex = childIndex;
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= plane.materialIndex && plane.materialIndex < child.materialCount);
		plane.materialIndex = <i32>materialIndices[plane.materialIndex];
	}

	moverContext.planeCount += planeCount;

	// Continue query while there is room for more planes
	return moverContext.planeCount < moverContext.planeCapacity;
}

export function b3CollideMoverAndCompound(
	planes: StaticArray<b3PlaneResult>,
	capacity: i32,
	shape: b3CompoundData,
	mover: b3Capsule,
): i32 {
	let context = new b3CompoundMoverContext(shape, planes, capacity, mover);

	let aabb = b3AABB();
	aabb.lowerBound = b3Min(mover.center1, mover.center2);
	aabb.upperBound = b3Max(mover.center1, mover.center2);
	let r = b3Vec3(mover.radius, mover.radius, mover.radius);
	aabb.lowerBound = b3Sub(aabb.lowerBound, r);
	aabb.upperBound = b3Add(aabb.upperBound, r);

	let view = b3GetCompoundTreeView(shape);
	b3TreeView_Query(view, aabb, ~(<u64>0), false, (proxyId: i32, userData: u64): bool =>
		b3CompoundMoverCallback(proxyId, userData, context),
	);

	return context.planeCount;
}
