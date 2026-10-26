// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box3d/collision.h and the query, tree, mover, geometry and
// collision groups of include/box3d/types.h

import { b3AABB, b3Matrix3, b3Plane, b3Quat, b3Transform, b3Vec2, b3Vec3 } from "./math_functions";
import { b3ShapeType, b3SurfaceMaterial, b3_capsuleShape } from "./types";

/**
 * @defgroup query Query
 * @brief Query types and functions
 */

/// Low level ray cast input data.
export struct b3RayCastInput {
	/// Start point of the ray cast.
	origin: b3Vec3 = b3Vec3();

	/// Translation of the ray cast.
	/// end = start + translation.
	translation: b3Vec3 = b3Vec3();

	/// The maximum fraction of the translation to consider, typically 1
	maxFraction: f32 = 0;

	constructor(origin: b3Vec3, translation: b3Vec3, maxFraction: f32) {
		this.origin = origin;
		this.translation = translation;
		this.maxFraction = maxFraction;
	}
}

const b3_emptyPoints = new StaticArray<b3Vec3>(0);

/// A shape proxy is used by the GJK algorithm. It can represent a convex shape.
/// C points into hull, capsule or stack storage; here points references the array that holds them.
export class b3ShapeProxy {
	/// The point cloud. Only the first count entries are used.
	points: StaticArray<b3Vec3>;

	/// The number of points. Do not exceed B3_MAX_SHAPE_CAST_POINTS.
	count: i32;

	/// The external radius of the point cloud.
	radius: f32;

	constructor(points: StaticArray<b3Vec3> = b3_emptyPoints, count: i32 = 0, radius: f32 = 0) {
		this.points = points;
		this.count = count;
		this.radius = radius;
	}
}

/// Low level shape cast input in generic form. This allows casting an arbitrary point
/// cloud wrap with a radius. For example, a sphere is a single point with a non-zero radius.
/// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
export class b3ShapeCastInput {
	/// A generic query shape.
	proxy: b3ShapeProxy = new b3ShapeProxy();

	/// The translation of the shape cast.
	translation: b3Vec3 = b3Vec3();

	/// The maximum fraction of the translation to consider, typically 1.
	maxFraction: f32 = 0;

	/// Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.
	canEncroach: bool = false;
}

/// Input for sweeping an AABB through a dynamic tree. The box is in the tree's world float frame.
/// The caller folds the cast shape radius and any world origin into the box, so the tree traversal
/// stays a conservative box sweep and the precise narrow phase happens per shape in the callback.
export struct b3BoxCastInput {
	/// The AABB to cast, in the tree's frame.
	box: b3AABB = b3AABB();

	/// The sweep translation.
	translation: b3Vec3 = b3Vec3();

	/// The maximum fraction of the translation to consider, typically 1.
	maxFraction: f32 = 0;

	constructor(box: b3AABB, translation: b3Vec3, maxFraction: f32) {
		this.box = box;
		this.translation = translation;
		this.maxFraction = maxFraction;
	}
}

/// Low level ray cast or shape-cast output data.
export struct b3CastOutput {
	/// The surface normal at the hit point.
	normal: b3Vec3 = b3Vec3();

	/// The surface hit point.
	point: b3Vec3 = b3Vec3();

	/// The fraction of the input translation at collision.
	fraction: f32 = 0;

	/// The number of iterations used.
	iterations: i32 = 0;

	/// The index of the mesh or height field triangle hit.
	triangleIndex: i32 = 0;

	/// The index of the compound child shape.
	childIndex: i32 = 0;

	/// The material index. May be -1 for null.
	materialIndex: i32 = 0;

	/// Did the cast hit?
	hit: bool = false;
}

/// Same type in single precision.
export type b3WorldCastOutput = b3CastOutput;

/// Used to warm start the GJK simplex. If you call this function multiple times with nearby
/// transforms this might improve performance. Otherwise you can zero initialize this.
/// The distance cache must be initialized to zero on the first call.
/// Users should generally just zero initialize this structure for each call.
export struct b3SimplexCache {
	/// Value use to compare length, area, volume of two simplexes.
	metric: f32 = 0;

	// todo use an index of 0xFF as a sentinel and remove the count
	/// The number of stored simplex points
	count: u16 = 0;

	/// The cached simplex indices on shape A
	indexA: FixedArray<u8, 4>;

	/// The cached simplex indices on shape B
	indexB: FixedArray<u8, 4>;
}

export const b3_emptyDistanceCache: b3SimplexCache = b3SimplexCache();

/// Input parameters for b3ShapeCast
export class b3ShapeCastPairInput {
	proxyA: b3ShapeProxy = new b3ShapeProxy(); ///< The proxy for shape A
	proxyB: b3ShapeProxy = new b3ShapeProxy(); ///< The proxy for shape B
	transform: b3Transform = b3Transform(); ///< Transform of shape B in shape A's frame, the relative pose B in A
	translationB: b3Vec3 = b3Vec3(); ///< The translation of shape B, in A's frame
	maxFraction: f32 = 0; ///< The fraction of the translation to consider, typically 1
	canEncroach: bool = false; ///< Allows shapes with a radius to move slightly closer if already touching
}

/// Input for b3ShapeDistance
export class b3DistanceInput {
	/// The proxy for shape A
	proxyA: b3ShapeProxy = new b3ShapeProxy();

	/// The proxy for shape B
	proxyB: b3ShapeProxy = new b3ShapeProxy();

	/// Transform of shape B in shape A's frame, the relative pose B in A
	/// (b3InvMulWorldTransforms( worldA, worldB )). The query is origin independent and runs in frame A.
	transform: b3Transform = b3Transform();

	/// Should the proxy radius be considered?
	useRadii: bool = false;
}

/// Output for b3ShapeDistance
export struct b3DistanceOutput {
	pointA: b3Vec3 = b3Vec3(); ///< Closest point on shapeA, in shape A's frame
	pointB: b3Vec3 = b3Vec3(); ///< Closest point on shapeB, in shape A's frame
	normal: b3Vec3 = b3Vec3(); ///< A to B normal in shape A's frame. Invalid if distance is zero.
	distance: f32 = 0; ///< The final distance, zero if overlapped
	iterations: i32 = 0; ///< Number of GJK iterations used
	simplexCount: i32 = 0; ///< The number of simplexes stored in the simplex array
}

/// Simplex vertex for debugging the GJK algorithm
export struct b3SimplexVertex {
	wA: b3Vec3 = b3Vec3(); ///< support point in proxyA
	wB: b3Vec3 = b3Vec3(); ///< support point in proxyB
	w: b3Vec3 = b3Vec3(); ///< wB - wA
	a: f32 = 0; ///< barycentric coordinates
	indexA: i32 = 0; ///< wA index
	indexB: i32 = 0; ///< wB index
}

/// Simplex from the GJK algorithm
export struct b3Simplex {
	vertices: FixedArray<b3SimplexVertex, 4>; ///< vertices
	count: i32 = 0; ///< number of valid vertices
}

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
/// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
/// position.
export struct b3Sweep {
	localCenter: b3Vec3 = b3Vec3(); ///< Local center of mass position
	c1: b3Vec3 = b3Vec3(); ///< Starting center of mass world position
	c2: b3Vec3 = b3Vec3(); ///< Ending center of mass world position
	q1: b3Quat = b3Quat(); ///< Starting world rotation
	q2: b3Quat = b3Quat(); ///< Ending world rotation
}

/// Time of impact input
export class b3TOIInput {
	proxyA: b3ShapeProxy = new b3ShapeProxy(); ///< The proxy for shape A
	proxyB: b3ShapeProxy = new b3ShapeProxy(); ///< The proxy for shape B
	sweepA: b3Sweep = b3Sweep(); ///< The movement of shape A
	sweepB: b3Sweep = b3Sweep(); ///< The movement of shape B
	maxFraction: f32 = 0; ///< Defines the sweep interval [0, tMax]
}

/// Describes the TOI output
export type b3TOIState = i32;
export const b3_toiStateUnknown: b3TOIState = 0;
export const b3_toiStateFailed: b3TOIState = 1;
export const b3_toiStateOverlapped: b3TOIState = 2;
export const b3_toiStateHit: b3TOIState = 3;
export const b3_toiStateSeparated: b3TOIState = 4;

/// Time of impact output.
export struct b3TOIOutput {
	/// The type of result.
	state: b3TOIState = 0;

	/// The hit point. A shared point if overlapped.
	point: b3Vec3 = b3Vec3();

	/// The hit normal. Zero if overlapped.
	normal: b3Vec3 = b3Vec3();

	/// The sweep time of the collision. 0 if overlapped.
	fraction: f32 = 0;

	/// The final distance. 0 if overlapped.
	distance: f32 = 0;

	/// Number of outer iterations.
	distanceIterations: i32 = 0;

	/// Total number of push back iterations.
	pushBackIterations: i32 = 0;

	/// Total number of root iterations.
	rootIterations: i32 = 0;

	/// Indicates that the time of impact detected initial
	/// overlap and used a fallback sphere as a last ditch effort
	/// to prevent tunneling.
	usedFallback: bool = false;
}

/**
 * @defgroup tree Dynamic Tree
 */

/// A node in the dynamic tree. Siblings sit together at an even index so two nodes fit in
/// a 64 byte cache line. The root is at index zero and index one always empty.
export struct b3TreeNode {
	/// The node bounding box
	aabb: b3AABB = b3AABB(); // 24

	/// bit 31 : 1 for leaf node
	/// bit 30 : 1 for moved flag
	/// bits 0-29 : index of the sibling pair node or the proxy id for a leaf
	flagIndex: u32 = 0; // 4

	// The C union becomes one field per member; they do not alias.

	/// The height of an internal node. A leaf has zero height.
	height: i32 = 0;

	/// The shape index for a leaf. Truncated from proxy user data.
	shapeIndex: i32 = 0;
}

/// Separate storage for tree leaves.
export struct b3TreeProxy {
	/// User data is an index instead of void* because it is used internally as a shape index.
	userData: u64 = 0;

	/// Category bits for collision filtering.
	categoryBits: u64 = 0;

	/// The leaf node. B3_NULL_INDEX for a free proxy.
	node: i32 = 0;

	/// Next free proxy.
	next: i32 = 0;
}

/// Dynamic tree version for compatibility testing.
export const B3_DYNAMIC_TREE_VERSION: u64 = 0x1d6f4c2a73b80e91;

/// The dynamic tree structure. This should be considered private data.
/// It is placed here for performance reasons.
/// C pointers become arrays; growth replaces the array with a larger copy.
export class b3DynamicTree {
	/// The dynamic tree version. Always the first field. Useful
	/// if the tree is serialized.
	version: u64 = 0;

	/// Array of nodes. The root is at index zero and index 1 is empty.
	/// Otherwise siblings are paired at even indices. Has holes for free node pairs.
	nodes: StaticArray<b3TreeNode> = new StaticArray<b3TreeNode>(0);

	/// Parent index per node. The free list is interweaved.
	parents: StaticArray<i32> = new StaticArray<i32>(0);

	/// Proxy data split from node array as cold data.
	proxies: StaticArray<b3TreeProxy> = new StaticArray<b3TreeProxy>(0);

	/// Every allocated node has a lower index than this.
	nodeEnd: i32 = 0;

	/// The allocated node space
	nodeCapacity: i32 = 0;

	/// Free pairs below nodeEnd
	pairFreeList: i32 = 0;

	/// Number of proxies created
	proxyCount: i32 = 0;

	/// The allocated proxy space
	proxyCapacity: i32 = 0;

	/// Proxy free list
	proxyFreeList: i32 = 0;

	/// Array of nodes for rebuild.
	swapNodes: StaticArray<b3TreeNode> = new StaticArray<b3TreeNode>(0);

	/// Leaf indices for rebuild
	leafIndices: StaticArray<i32> = new StaticArray<i32>(0);

	/// Leaves for the rebuild. May represent a proxy or a retained subtree.
	leafNodes: StaticArray<b3TreeNode> = new StaticArray<b3TreeNode>(0);

	/// Leaf bounding boxes for rebuild
	leafBoxes: StaticArray<b3AABB> = new StaticArray<b3AABB>(0);

	/// Leaf bounding box centers for rebuild
	leafCenters: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(0);

	/// Bins for sorting during rebuild
	binIndices: StaticArray<i32> = new StaticArray<i32>(0);

	/// Allocated space for rebuilding
	rebuildCapacity: i32 = 0;

	/// Rebuild orders the nodes so the children follow parents. Cache friendly for queries
	/// and refitting. The order can be disrupted by proxy creation.
	dfsOrdered: bool = false;
}

/// These are performance results returned by dynamic tree queries.
export struct b3TreeStats {
	/// Number of internal nodes visited during the query
	nodeVisits: i32 = 0;

	/// Number of leaf nodes visited during the query
	leafVisits: i32 = 0;
}

// Callbacks drop C's void* context; the closure carries it.

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
export type b3TreeQueryCallbackFcn = (proxyId: i32, userData: u64) => bool;

/// This function receives the minimum distance squared so far and proxy to check in the closest query.
/// @return minimum distance squared to user objects in the proxy
export type b3TreeQueryClosestCallbackFcn = (distanceSqrMin: f32, proxyId: i32, userData: u64) => f32;

/// This function receives clipped AABB cast input for a proxy. The function returns the new cast
/// fraction.
/// - return a value of 0 to terminate the cast
/// - return a value less than input->maxFraction to clip the cast
/// - return a value of input->maxFraction to continue the cast without clipping
export type b3TreeBoxCastCallbackFcn = (input: b3BoxCastInput, proxyId: i32, userData: u64) => f32;

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
export type b3TreeRayCastCallbackFcn = (input: b3RayCastInput, proxyId: i32, userData: u64) => f32;

/**
 * @defgroup character Character Mover
 */

/// The plane between a character mover and a shape
export struct b3PlaneResult {
	/// Outward pointing plane.
	plane: b3Plane = b3Plane();

	/// Closest point on the shape. May not be unique.
	point: b3Vec3 = b3Vec3();

	/// The index of the mesh or height field triangle hit.
	triangleIndex: i32 = 0;

	/// The index of the compound child shape.
	childIndex: i32 = 0;

	/// The material index.
	materialIndex: i32 = 0;
}

/// These are collision planes that can be fed to b3SolvePlanes. Normally
/// this is assembled by the user from plane results in b3PlaneResult.
export struct b3CollisionPlane {
	/// The collision plane between the mover and some shape.
	plane: b3Plane = b3Plane();

	/// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can
	/// make the plane collision soft. Usually in meters.
	pushLimit: f32 = 0;

	/// The push on the mover determined by b3SolvePlanes. Usually in meters.
	push: f32 = 0;

	/// Indicates if b3ClipVector should clip against this plane. Should be false for soft collision.
	clipVelocity: bool = false;
}

/// Result returned by b3SolvePlanes.
export struct b3PlaneSolverResult {
	/// The final relative translation.
	delta: b3Vec3 = b3Vec3();

	/// The number of iterations used by the plane solver. For diagnostics.
	iterationCount: i32 = 0;
}

/**
 * @defgroup geometry Geometry
 */

/// This holds the mass data computed for a shape.
export struct b3MassData {
	/// The shape mass
	mass: f32 = 0;

	/// The local center of mass position.
	center: b3Vec3 = b3Vec3();

	/// The inertia tensor about the shape center of mass.
	inertia: b3Matrix3 = b3Matrix3();
}

/// A solid sphere
export struct b3Sphere {
	/// The local center
	center: b3Vec3 = b3Vec3();

	/// The radius
	radius: f32 = 0;

	constructor(center: b3Vec3, radius: f32) {
		this.center = center;
		this.radius = radius;
	}
}

/// A solid capsule can be viewed as two hemispheres connected by a cylinder.
export struct b3Capsule {
	/// Local center of the first hemisphere
	center1: b3Vec3 = b3Vec3();

	/// Local center of the second hemisphere
	center2: b3Vec3 = b3Vec3();

	/// The radius of the hemispheres
	radius: f32 = 0;

	constructor(center1: b3Vec3, center2: b3Vec3, radius: f32) {
		this.center1 = center1;
		this.center2 = center2;
		this.radius = radius;
	}
}

/// A hull vertex. Identified by a half-edge with this vertex as its tail.
export struct b3HullVertex {
	/// A half-edge that has this vertex as the origin
	/// Can be used along with edge twins and winding order
	/// to traverse all the edges connected to this vertex.
	edge: u8 = 0;
}

/// Half-edge for hull data structure
export struct b3HullHalfEdge {
	/// Next edge index CCW
	next: u8 = 0;

	/// Twin edge index
	twin: u8 = 0;

	/// index of origin vertex and point
	origin: u8 = 0;

	/// Face to the left of this edge
	face: u8 = 0;

	constructor(next: u8, twin: u8, origin: u8, face: u8) {
		this.next = next;
		this.twin = twin;
		this.origin = origin;
		this.face = face;
	}
}

/// A hull face. Hulls use a half-edge data structure, so a face
/// can be determined from a single half-edge index.
export struct b3HullFace {
	/// An arbitrary half-edge on this face
	edge: u8 = 0;
}

/// 64-bit hull version. Useful for validating serialized data.
export const B3_HULL_VERSION: u64 = 0x6b1e39d4a87c25f3;

// sizeof( b3HullData ) in C, the base of the blob layout.
export const B3_HULL_DATA_SIZE: i32 = 152;

// b3GetHullSoaStride from src/hull.h
@inline
function b3HullSoaStride(count: i32): i32 {
	return (count + 7) & ~7;
}

@inline
function b3AlignUp8i(x: i32): i32 {
	return (x + 7) & ~7;
}

/// A convex hull.
/// C hangs the arrays off the end of the struct at byte offsets. Here each section is its own
/// array; the offset and byteCount fields keep the values C computes.
export class b3HullData {
	/// Version must be first and match B3_HULL_VERSION
	version: u64 = 0;

	/// Hash of this hull (this field is zero when the hash is computed).
	hash: u64 = 0;

	/// Axis-aligned box in local space.
	aabb: b3AABB = b3AABB();

	/// Surface area, typically in squared meters.
	surfaceArea: f32 = 0;

	/// Volume, typically in m^3.
	volume: f32 = 0;

	/// The radius of the largest sphere at the center.
	innerRadius: f32 = 0;

	/// The local centroid.
	center: b3Vec3 = b3Vec3();

	/// The inertia tensor about the centroid.
	centralInertia: b3Matrix3 = b3Matrix3();

	/// The vertex count.
	vertexCount: i32 = 0;

	/// Offset of the vertex array in bytes from the struct address.
	vertexOffset: i32 = 0;

	/// Offset of the point array in bytes from the struct address.
	pointOffset: i32 = 0;

	/// This is the half-edge count (double the edge count).
	edgeCount: i32 = 0;

	/// Offset of the edge array in bytes from the struct address.
	edgeOffset: i32 = 0;

	/// The face count. Hulls faces are convex polygons.
	faceCount: i32 = 0;

	/// Offset of the face plane array in bytes from the struct address.
	planeOffset: i32 = 0;

	/// Offset of the face array in bytes from the struct address.
	faceOffset: i32 = 0;

	/// Offset of structure of array (SOA) vertices.
	soaVertexOffset: i32 = 0;

	/// Offset of structure of array (SOA) unit normal vectors.
	soaNormalOffset: i32 = 0;

	/// Offset of dot(n1, n2) for each full edge.
	edgeCosineOffset: i32 = 0;

	/// The total number of bytes for this hull.
	byteCount: i32 = 0;

	/// Explicit padding for determinism.
	padding: i32 = 0;

	/// vertexCount entries
	vertices: StaticArray<b3HullVertex>;

	/// vertexCount entries
	points: StaticArray<b3Vec3>;

	/// edgeCount half-edges
	edges: StaticArray<b3HullHalfEdge>;

	/// faceCount entries
	planes: StaticArray<b3Plane>;

	/// faceCount entries
	faces: StaticArray<b3HullFace>;

	/// x then y then z, each soaStride(vertexCount) long, padded with the first vertex
	soaVertices: StaticArray<f32>;

	/// x then y then z, each soaStride(faceCount) long, padded with 0
	soaNormals: StaticArray<f32>;

	/// edgeCount / 2 entries
	edgeCosines: StaticArray<f32>;

	/// Allocates every section and sets the counts, offsets and byteCount the way b3CreateHull does.
	constructor(vertexCount: i32 = 0, edgeCount: i32 = 0, faceCount: i32 = 0) {
		let soaVertexCount = b3HullSoaStride(vertexCount);
		let soaNormalCount = b3HullSoaStride(faceCount);

		this.vertices = new StaticArray<b3HullVertex>(vertexCount);
		this.points = new StaticArray<b3Vec3>(vertexCount);
		this.edges = new StaticArray<b3HullHalfEdge>(edgeCount);
		this.planes = new StaticArray<b3Plane>(faceCount);
		this.faces = new StaticArray<b3HullFace>(faceCount);
		this.soaVertices = new StaticArray<f32>(3 * soaVertexCount);
		this.soaNormals = new StaticArray<f32>(3 * soaNormalCount);
		this.edgeCosines = new StaticArray<f32>(edgeCount / 2);

		this.vertexCount = vertexCount;
		this.edgeCount = edgeCount;
		this.faceCount = faceCount;

		let byteCount = b3AlignUp8i(B3_HULL_DATA_SIZE);
		this.vertexOffset = byteCount;
		byteCount += b3AlignUp8i(vertexCount * 1);
		this.pointOffset = byteCount;
		byteCount += b3AlignUp8i(vertexCount * 12);
		this.edgeOffset = byteCount;
		byteCount += b3AlignUp8i(edgeCount * 4);
		this.planeOffset = byteCount;
		byteCount += b3AlignUp8i(faceCount * 16);
		this.faceOffset = byteCount;
		byteCount += b3AlignUp8i(faceCount * 1);
		this.soaVertexOffset = byteCount;
		byteCount += b3AlignUp8i(3 * soaVertexCount * 4);
		this.soaNormalOffset = byteCount;
		byteCount += b3AlignUp8i(3 * soaNormalCount * 4);
		this.edgeCosineOffset = byteCount;
		byteCount += b3AlignUp8i((edgeCount / 2) * 4);
		this.byteCount = byteCount;
	}
}

// offsetof( b3BoxHull, ... ) and sizeof( b3BoxHull ) in C
export const B3_BOX_HULL_SIZE: i32 = 696;
export const B3_BOX_HULL_VERTEX_OFFSET: i32 = 152;
export const B3_BOX_HULL_POINT_OFFSET: i32 = 160;
export const B3_BOX_HULL_EDGE_OFFSET: i32 = 256;
export const B3_BOX_HULL_PLANE_OFFSET: i32 = 352;
export const B3_BOX_HULL_FACE_OFFSET: i32 = 448;
export const B3_BOX_HULL_SOA_VERTEX_OFFSET: i32 = 456;
export const B3_BOX_HULL_SOA_NORMAL_OFFSET: i32 = 552;
export const B3_BOX_HULL_EDGE_COSINE_OFFSET: i32 = 648;

/// Efficient box hull
/// The box fields alias the sections of base, so writes through either name land in the hull.
/// vx, vy, vz are soaVertices[0..8), [8..16), [16..24); nx, ny, nz likewise in soaNormals.
export class b3BoxHull {
	/// The embedded hull.
	base: b3HullData;
	boxVertices: StaticArray<b3HullVertex>; ///< Box vertices.
	boxPoints: StaticArray<b3Vec3>; ///< Box points.
	boxEdges: StaticArray<b3HullHalfEdge>; ///< Box half-edges.
	boxPlanes: StaticArray<b3Plane>; ///< Box face planes.
	boxFaces: StaticArray<b3HullFace>; ///< Box faces.
	soaVertices: StaticArray<f32>; ///< vertex x, y, z
	soaNormals: StaticArray<f32>; ///< normal x, y, z, padded to multiple of 8
	edgeCosines: StaticArray<f32>; ///< dot(n1, n2) for each full edge.

	/// Sizes the sections for 8 vertices, 24 half-edges and 6 faces with the C box offsets.
	/// Topology and the remaining fields are filled by b3MakeTransformedBoxHull.
	constructor() {
		let base = new b3HullData(8, 24, 6);
		base.vertexOffset = B3_BOX_HULL_VERTEX_OFFSET;
		base.pointOffset = B3_BOX_HULL_POINT_OFFSET;
		base.edgeOffset = B3_BOX_HULL_EDGE_OFFSET;
		base.planeOffset = B3_BOX_HULL_PLANE_OFFSET;
		base.faceOffset = B3_BOX_HULL_FACE_OFFSET;
		base.soaVertexOffset = B3_BOX_HULL_SOA_VERTEX_OFFSET;
		base.soaNormalOffset = B3_BOX_HULL_SOA_NORMAL_OFFSET;
		base.edgeCosineOffset = B3_BOX_HULL_EDGE_COSINE_OFFSET;
		base.byteCount = B3_BOX_HULL_SIZE;
		this.base = base;
		this.boxVertices = base.vertices;
		this.boxPoints = base.points;
		this.boxEdges = base.edges;
		this.boxPlanes = base.planes;
		this.boxFaces = base.faces;
		this.soaVertices = base.soaVertices;
		this.soaNormals = base.soaNormals;
		this.edgeCosines = base.edgeCosines;
	}
}

/// This is used to create a re-usable collision mesh. No pointers
/// are held to this data in b3MeshData. So all this data can be temporary.
export class b3MeshDef {
	/// Triangle vertices.
	vertices: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(0);

	/// Stride between vertices. Use 0 for contiguous vertices. Unused: vertices are always contiguous.
	stride: i32 = 0;

	/// Triangle vertex indices. 3 for each triangle. CCW winding unless CW is indicated below.
	indices: StaticArray<i32> = new StaticArray<i32>(0);

	/// Triangle material index. 1 per triangle. Indexes into b3ShapeDef::materials. Null for none.
	/// This allows different run-time material data to be associated with different
	/// instances of this mesh.
	materialIndices: StaticArray<u8>? = null;

	/// Tolerance for vertex welding in length units.
	weldTolerance: f32 = 0;

	/// The vertex count. Must be 3 or more.
	vertexCount: i32 = 0;

	/// The triangle count. Must be 1 or more.
	triangleCount: i32 = 0;

	/// Optionally weld nearby vertices.
	weldVertices: bool = false;

	/// Use the median split instead of SAH to speed up mesh creation. Good
	/// for meshes that are structured like a grid.
	useMedianSplit: bool = false;

	/// Compute triangle adjacency information using shared edges
	identifyEdges: bool = false;

	/// Input indices have clockWise winding order.
	clockWiseWinding: bool = false;
}

/// 64-bit mesh version. Useful for validating serialized data.
export const B3_MESH_VERSION: u64 = 0xaaab9a00f1a8aaf7;

/// Triangle mesh edge flags.
export type b3MeshEdgeFlags = i32;
export const b3_concaveEdge1: b3MeshEdgeFlags = 0x01;
export const b3_concaveEdge2: b3MeshEdgeFlags = 0x02;
export const b3_concaveEdge3: b3MeshEdgeFlags = 0x04;

export const b3_inverseConcaveEdge1: b3MeshEdgeFlags = 0x10;
export const b3_inverseConcaveEdge2: b3MeshEdgeFlags = 0x20;
export const b3_inverseConcaveEdge3: b3MeshEdgeFlags = 0x40;

export const b3_allConcaveEdges: b3MeshEdgeFlags = b3_concaveEdge1 | b3_concaveEdge2 | b3_concaveEdge3;

export const b3_flatEdge1: b3MeshEdgeFlags = b3_concaveEdge1 | b3_inverseConcaveEdge1;
export const b3_flatEdge2: b3MeshEdgeFlags = b3_concaveEdge2 | b3_inverseConcaveEdge2;
export const b3_flatEdge3: b3MeshEdgeFlags = b3_concaveEdge3 | b3_inverseConcaveEdge3;

export const b3_allFlatEdges: b3MeshEdgeFlags = b3_flatEdge1 | b3_flatEdge2 | b3_flatEdge3;

/// A mesh triangle.
export struct b3MeshTriangle {
	index1: i32 = 0; ///< Index of vertex 1.
	index2: i32 = 0; ///< Index of vertex 2.
	index3: i32 = 0; ///< Index of vertex 3.

	constructor(index1: i32, index2: i32, index3: i32) {
		this.index1 = index1;
		this.index2 = index2;
		this.index3 = index3;
	}
}

/// Internal node view of b3MeshNode::data.
export struct b3MeshNodeAsNode {
	/// Split axis. 0, 1, or 2.
	axis: u32(2);
	/// Offset of the second child node.
	childOffset: u32(30);
}

/// Leaf node view of b3MeshNode::data.
export struct b3MeshNodeAsLeaf {
	/// Aligned with axis above and has value of 3 if this is a leaf.
	type: u32(2);

	/// The number of triangles for this leaf node.
	triangleCount: u32(30);
}

/// The C union becomes one field per member; they do not alias. An internal node never
/// writes asLeaf, so asLeaf.type reads 0 there, never the leaf marker 3.
export struct b3MeshNodeData {
	/// Internal node
	asNode: b3MeshNodeAsNode = b3MeshNodeAsNode();

	/// Leaf node
	asLeaf: b3MeshNodeAsLeaf = b3MeshNodeAsLeaf();
}

/// A mesh BVH node.
export struct b3MeshNode {
	/// The lower bound of the node AABB. Strategic placement for SIMD.
	lowerBound: b3Vec3 = b3Vec3();

	/// Anonymous union.
	data: b3MeshNodeData = b3MeshNodeData();

	/// The upper bound of the node AABB.  Strategic placement for SIMD.
	upperBound: b3Vec3 = b3Vec3();

	/// The index of the leaf triangles.
	triangleOffset: u32 = 0;
}

// sizeof( b3MeshData ) and sizeof( b3MeshNode ) in C
export const B3_MESH_DATA_SIZE: i32 = 96;
export const B3_MESH_NODE_SIZE: i32 = 32;

/// This is a sorted triangle collision bounding volume hierarchy.
/// C hangs the arrays off the end of the struct at byte offsets. Here each section is its own array.
export class b3MeshData {
	/// Version must be first.
	version: u64 = 0;

	/// Hash of this mesh (this field is zero when the hash is computed)
	hash: u64 = 0;

	/// The total number of bytes for this mesh.
	byteCount: i32 = 0;

	/// Local axis-aligned box.
	bounds: b3AABB = b3AABB();

	/// Combined surface area of all triangles. Single-sided.
	surfaceArea: f32 = 0;

	/// The height of the bounding volume hierarchy.
	treeHeight: i32 = 0;

	/// The number of degenerate triangles. Diagnostic.
	degenerateCount: i32 = 0;

	/// Offset of the node array in bytes from the struct address.
	nodeOffset: i32 = 0;

	/// The number of BVH nodes.
	nodeCount: i32 = 0;

	/// Offset of the vertex array in bytes from the struct address.
	vertexOffset: i32 = 0;

	/// The number of vertices.
	vertexCount: i32 = 0;

	/// Offset of the triangle array in bytes from the struct address.
	triangleOffset: i32 = 0;

	/// The number of triangles.
	triangleCount: i32 = 0;

	/// Offset of the material array in bytes from the struct address.
	materialOffset: i32 = 0;

	/// The number of materials.
	materialCount: i32 = 0;

	/// Offset of the triangle flag array in bytes from the struct address.
	flagsOffset: i32 = 0;

	/// Explicit padding.
	padding: i32 = 0;

	/// nodeCount entries, the root first
	nodes: StaticArray<b3MeshNode>;

	/// vertexCount entries
	vertices: StaticArray<b3Vec3>;

	/// triangleCount entries
	triangles: StaticArray<b3MeshTriangle>;

	/// triangleCount entries
	materialIndices: StaticArray<u8>;

	/// triangleCount entries of b3MeshEdgeFlags
	flags: StaticArray<u8>;

	/// Allocates every section and sets the counts, offsets and byteCount the way b3CreateMesh does.
	constructor(nodeCount: i32 = 0, vertexCount: i32 = 0, triangleCount: i32 = 0) {
		this.nodes = new StaticArray<b3MeshNode>(nodeCount);
		this.vertices = new StaticArray<b3Vec3>(vertexCount);
		this.triangles = new StaticArray<b3MeshTriangle>(triangleCount);
		this.materialIndices = new StaticArray<u8>(triangleCount);
		this.flags = new StaticArray<u8>(triangleCount);

		this.nodeCount = nodeCount;
		this.vertexCount = vertexCount;
		this.triangleCount = triangleCount;

		let byteCount = b3AlignUp8i(B3_MESH_DATA_SIZE);
		this.nodeOffset = byteCount;
		byteCount += b3AlignUp8i(nodeCount * B3_MESH_NODE_SIZE);
		this.vertexOffset = byteCount;
		byteCount += b3AlignUp8i(vertexCount * 12);
		this.triangleOffset = byteCount;
		byteCount += b3AlignUp8i(triangleCount * 12);
		this.materialOffset = byteCount;
		byteCount += b3AlignUp8i(triangleCount * 1);
		this.flagsOffset = byteCount;
		byteCount += b3AlignUp8i(triangleCount * 1);
		this.byteCount = byteCount;
	}
}

/// This allows mesh data to be re-used with different scales.
export class b3Mesh {
	/// Immutable pointer to the mesh data.
	data: b3MeshData;

	/// This scale may be non-uniform and have negative components. However,
	/// no component may be very small in magnitude.
	scale: b3Vec3;

	constructor(data: b3MeshData, scale: b3Vec3) {
		this.data = data;
		this.scale = scale;
	}
}

/// Data used to create a height field. No pointers are held to this data.
export class b3HeightFieldDef {
	/// Grid point heights
	/// count = countX * countZ
	heights: StaticArray<f32> = new StaticArray<f32>(0);

	/// Grid cell material. Null for none.
	/// A value of 0xFF is reserved for holes
	/// count = (countX - 1) * (countZ - 1)
	materialIndices: StaticArray<u8>? = null;

	/// The height field scale. All components must be positive values.
	scale: b3Vec3 = b3Vec3();

	/// The number of grid lines along the x-axis.
	countX: i32 = 0;

	/// The number of grid lines along the z-axis.
	countZ: i32 = 0;

	/// Global minimum and maximum heights used for quantization. This is important
	/// if you want height fields to be placed next to each other and line up exactly.
	/// In that case, both height fields should use the same minimum and maximum heights.
	/// All height values are clamped to this range.
	/// These values are in unscaled space.
	globalMinimumHeight: f32 = 0;

	/// The maximum.
	globalMaximumHeight: f32 = 0;

	/// Use clock-wise winding. This effectively inverts the height-field along the y-axis.
	clockwiseWinding: bool = false;
}

/// This material index is used to designate holes in a height field.
export const B3_HEIGHT_FIELD_HOLE: i32 = 0xff;

/// 64-bit height-field version. Useful for validating serialized data.
export const B3_HEIGHT_FIELD_VERSION: u64 = 0x8e41e5fb084848f8;

// sizeof( b3HeightFieldData ) in C
export const B3_HEIGHT_FIELD_DATA_SIZE: i32 = 96;

/// A height field with compressed storage.
/// C hangs the arrays off the end of the struct at byte offsets. Here each section is its own array.
export class b3HeightFieldData {
	/// Version must be first and match B3_HEIGHT_FIELD_VERSION
	version: u64 = 0;

	/// Hash of this height field (this field is zero when the hash is computed).
	hash: u64 = 0;

	/// The total number of bytes for this height field.
	byteCount: i32 = 0;

	/// The local axis-aligned bounding box.
	aabb: b3AABB = b3AABB();

	/// The minimum y value.
	minHeight: f32 = 0;

	/// The maximum y value
	maxHeight: f32 = 0;

	/// The quantization scale.
	heightScale: f32 = 0;

	/// The overall scale.
	scale: b3Vec3 = b3Vec3();

	/// The number of grid columns along the local x-axis.
	columnCount: i32 = 0;

	/// The number of grid rows along the local z-axis.
	rowCount: i32 = 0;

	/// Offset of the compressed height array in bytes from the struct address.
	/// uint16_t, one per grid point.
	heightsOffset: i32 = 0;

	/// Offset of the material index array in bytes from the struct address.
	/// uint8_t, one per cell.
	materialOffset: i32 = 0;

	/// Offset of the flag array in bytes from the struct address.
	/// uint8_t, one per triangle.
	flagsOffset: i32 = 0;

	/// Triangle winding.
	clockwise: u8 = 0;

	/// Explicit padding.
	padding: FixedArray<u8, 7>;

	/// columnCount * rowCount entries
	compressedHeights: StaticArray<u16>;

	/// (columnCount - 1) * (rowCount - 1) entries
	materialIndices: StaticArray<u8>;

	/// 2 * (columnCount - 1) * (rowCount - 1) entries
	flags: StaticArray<u8>;

	/// Allocates every section and sets the counts, offsets and byteCount the way b3CreateHeightField does.
	constructor(columnCount: i32 = 0, rowCount: i32 = 0) {
		let heightCount = columnCount * rowCount;
		let cellCount = columnCount > 0 && rowCount > 0 ? (columnCount - 1) * (rowCount - 1) : 0;
		let triangleCount = 2 * cellCount;

		this.compressedHeights = new StaticArray<u16>(heightCount);
		this.materialIndices = new StaticArray<u8>(cellCount);
		this.flags = new StaticArray<u8>(triangleCount);

		this.columnCount = columnCount;
		this.rowCount = rowCount;

		let byteCount = b3AlignUp8i(B3_HEIGHT_FIELD_DATA_SIZE);
		this.heightsOffset = byteCount;
		byteCount += b3AlignUp8i(heightCount * 2);
		this.materialOffset = byteCount;
		byteCount += b3AlignUp8i(cellCount * 1);
		this.flagsOffset = byteCount;
		byteCount += b3AlignUp8i(triangleCount * 1);
		this.byteCount = byteCount;
	}
}

/// Definition for a capsule in a compound shape.
export struct b3CompoundCapsuleDef {
	/// Local capsule.
	capsule: b3Capsule = b3Capsule();

	/// Material properties.
	material: b3SurfaceMaterial = b3SurfaceMaterial();
}

/// Definition for a convex hull in a compound shape.
export class b3CompoundHullDef {
	/// Shared hull.
	hull: b3HullData;

	/// Transform of the shared hull into compound local space.
	transform: b3Transform;

	/// Material properties.
	material: b3SurfaceMaterial;

	constructor(hull: b3HullData, transform: b3Transform, material: b3SurfaceMaterial) {
		this.hull = hull;
		this.transform = transform;
		this.material = material;
	}
}

/// Definition for a triangle mesh in a compound shape.
export class b3CompoundMeshDef {
	/// Shared mesh.
	meshData: b3MeshData;

	/// Transform of the shared mesh into compound local space.
	transform: b3Transform = b3Transform();

	/// Local space non-uniform mesh scale. May have negative components.
	scale: b3Vec3 = b3Vec3();

	/// Material properties.
	/// This array must line up with the material indices on the triangles.
	materials: StaticArray<b3SurfaceMaterial> = new StaticArray<b3SurfaceMaterial>(0);

	/// Number of materials.
	materialCount: i32 = 0;

	constructor(meshData: b3MeshData) {
		this.meshData = meshData;
	}
}

/// Definition for a sphere in a compound shape.
export struct b3CompoundSphereDef {
	/// Local sphere.
	sphere: b3Sphere = b3Sphere();

	/// Material properties.
	material: b3SurfaceMaterial = b3SurfaceMaterial();
}

/// Definition for creating a compound shape. All this data is fully cloned
/// into the run-time compound shape.
export class b3CompoundDef {
	/// Capsule instances.
	capsules: StaticArray<b3CompoundCapsuleDef> = new StaticArray<b3CompoundCapsuleDef>(0);

	/// Number of capsules.
	capsuleCount: i32 = 0;

	/// Hulls instances.
	hulls: StaticArray<b3CompoundHullDef> = [];

	/// Number of hull instances.
	hullCount: i32 = 0;

	/// Mesh instances.
	meshes: StaticArray<b3CompoundMeshDef> = [];

	/// Number of mesh instances.
	meshCount: i32 = 0;

	/// Sphere instances.
	spheres: StaticArray<b3CompoundSphereDef> = new StaticArray<b3CompoundSphereDef>(0);

	/// Number of spheres.
	sphereCount: i32 = 0;
}

/// The baked compound version depends on the mesh and hull versions.
export const B3_COMPOUND_VERSION: u64 = 0x9e4b17d3a25c68f1 ^ B3_MESH_VERSION ^ B3_HULL_VERSION;

/// A hull instance in a compound, private to src/compound.c and shared here for the class field.
/// hullOffset is an index into b3CompoundData::hulls, where C stores a byte offset into the blob.
export struct b3HullInstance {
	transform: b3Transform = b3Transform();
	hullOffset: u32 = 0;
	materialIndex: u16 = 0;
	padding: u16 = 0;
}

/// A mesh instance in a compound, private to src/compound.c and shared here for the class field.
/// meshOffset is an index into b3CompoundData::meshes and materialOffset an element index into
/// b3CompoundData::meshMaterialIndices, where C stores byte offsets into the blob.
export struct b3MeshInstance {
	transform: b3Transform = b3Transform();
	scale: b3Vec3 = b3Vec3();
	meshOffset: u32 = 0;
	materialOffset: u32 = 0;
	materialCount: u32 = 0;
}

// sizeof( b3CompoundData ) in C
export const B3_COMPOUND_DATA_SIZE: i32 = 96;

/// The data for a baked compound shape. This is a potentially large yet highly optimized
/// data structure. It can contain thousands of child shapes, yet at runtime it populates
/// into the world as a single shape in the runtime broad-phase.
/// C packs every section and the shared hull and mesh blobs into one allocation. Here each
/// section is its own array and the shared blobs are references; b3CreateCompound fills them.
export class b3CompoundData {
	/// The compound version is always first.
	version: u64 = 0;

	/// The total number of bytes for this compound.
	byteCount: i32 = 0;

	// Bounds of the tree.
	bounds: b3AABB = b3AABB();

	// Tree height for diagnostics.
	treeHeight: i32 = 0;

	/// Offset of the tree node array in bytes from the struct address.
	nodeOffset: i32 = 0;

	// The number of tree nodes.
	nodeCount: i32 = 0;

	/// Offset of the material array in bytes from the struct address.
	materialOffset: i32 = 0;

	/// The number of materials.
	materialCount: i32 = 0;

	/// Offset of the capsule array in bytes from the struct address.
	capsuleOffset: i32 = 0;

	/// The number of capsules.
	capsuleCount: i32 = 0;

	/// Offset of the hull instance array in bytes from the struct address.
	hullOffset: i32 = 0;

	/// The number of hull instances.
	hullCount: i32 = 0;

	/// The number of unique hulls. Diagnostic.
	sharedHullCount: i32 = 0;

	/// Offset of the mesh instance array in bytes from the struct address.
	meshOffset: i32 = 0;

	/// The number of mesh instances.
	meshCount: i32 = 0;

	/// The number of unique meshes. Diagnostic.
	sharedMeshCount: i32 = 0;

	/// Offset of the sphere array in bytes from the struct address.
	sphereOffset: i32 = 0;

	/// The number of spheres.
	sphereCount: i32 = 0;

	/// nodeCount tree nodes
	nodes: StaticArray<b3TreeNode> = new StaticArray<b3TreeNode>(0);

	/// materialCount entries
	materials: StaticArray<b3SurfaceMaterial> = new StaticArray<b3SurfaceMaterial>(0);

	/// capsuleCount entries
	capsules: StaticArray<b3CompoundCapsule> = new StaticArray<b3CompoundCapsule>(0);

	/// hullCount instances
	hullInstances: StaticArray<b3HullInstance> = new StaticArray<b3HullInstance>(0);

	/// sharedHullCount unique hulls
	hulls: StaticArray<b3HullData> = [];

	/// meshCount instances
	meshInstances: StaticArray<b3MeshInstance> = new StaticArray<b3MeshInstance>(0);

	/// The mesh material map, indexed by b3MeshInstance::materialOffset.
	/// A typed array so b3CompoundMesh and b3ChildShape can hold subarray views like C pointers.
	meshMaterialIndices: Uint16Array = new Uint16Array(0);

	/// sharedMeshCount unique meshes
	meshes: StaticArray<b3MeshData> = [];

	/// sphereCount entries
	spheres: StaticArray<b3CompoundSphere> = new StaticArray<b3CompoundSphere>(0);

	// Not in C: the mesh child objects and material index views b3GetCompoundChild hands out, built once per
	// compound so a contact update allocates none
	childMeshes: StaticArray<b3Mesh>? = null;
	childMeshMaterials: StaticArray<Uint16Array>? = null;
}

/// A capsule that lives in a compound.
export struct b3CompoundCapsule {
	/// Local capsule.
	capsule: b3Capsule = b3Capsule();

	/// Index to a shared material.
	materialIndex: u16 = 0;

	/// Padding for determinism.
	padding: u16 = 0;
}

/// A hull that lives in a compound.
export class b3CompoundHull {
	/// Pointer to the unique shared hull.
	hull: b3HullData? = null;

	/// The transform of this hull instance.
	transform: b3Transform = b3Transform();

	/// Index to a shared material.
	materialIndex: u16 = 0;

	/// Padding for determinism.
	padding: u16 = 0;
}

/// A mesh with non-uniform scale that lives in a compound.
export class b3CompoundMesh {
	/// Pointer to the unique shared mesh.
	meshData: b3MeshData? = null;

	/// The transform of this mesh instance.
	transform: b3Transform = b3Transform();

	/// Non-uniform scale of this mesh instance.
	scale: b3Vec3 = b3Vec3();

	/// This is used to access the surface material from b3GetCompoundMaterials.
	/// Requires an extra level of indirection.
	/// materialIndex = materialIndices[triangle->materialIndex]
	materialIndices: Uint16Array? = null;

	/// The number of materials. 1 for convex hapes.
	materialCount: i32 = 0;
}

/// A sphere that lives in a compound.
export struct b3CompoundSphere {
	/// Local sphere.
	sphere: b3Sphere = b3Sphere();

	/// Index to a shared material.
	materialIndex: u16 = 0;

	/// Padding for determinism.
	padding: u16 = 0;
}

/// Child shape of a compound
export class b3ChildShape {
	// The C tagged union becomes one field per member.
	capsule: b3Capsule = b3Capsule(); ///< Capsule.
	hull: b3HullData? = null; ///< Hull.
	mesh: b3Mesh? = null; ///< Mesh.
	sphere: b3Sphere = b3Sphere(); ///< Sphere.

	/// Transform of the shape into compound local space.
	transform: b3Transform = b3Transform();

	/// Material indices. Index 0 is used for convex shapes.
	materialIndices: Uint16Array? = null;

	/// The number of materials. This is one for convex shapes. For
	/// meshes it is determined by the maximum triangle material index.
	materialCount: i32 = 0;

	/// The shape type (union tag).
	type: b3ShapeType = b3_capsuleShape;
}

/// Callback for compound overlap queries. The closure carries C's context.
export type b3CompoundQueryFcn = (compound: b3CompoundData, childIndex: i32) => bool;

/**
 * @defgroup collision Shape Collision
 */

/// A manifold point is a contact point belonging to a contact manifold.
/// It holds details related to the geometry and dynamics of the contact points.
/// Box3D uses speculative collision so some contact points may be separated.
/// You may use the maxNormalImpulse to determine if there was an interaction during
/// the time step.
export struct b3ManifoldPoint {
	/// Location of the contact point relative to the bodyA center of mass in world space.
	anchorA: b3Vec3 = b3Vec3();

	/// Location of the contact point relative to the bodyB center of mass in world space.
	anchorB: b3Vec3 = b3Vec3();

	/// The separation of the contact point, negative if penetrating
	separation: f32 = 0;

	/// The impulse along the manifold normal vector. Since Box3D uses sub-stepping, this is
	/// result from the final sub-step.
	normalImpulse: f32 = 0;

	/// The total normal impulse applied across sub-stepping and restitution. This is important
	/// to identify speculative contact points that had an interaction in the time step.
	/// This includes the warm starting impulse, the sub-step delta impulse, and the restitution
	/// impulse.
	totalNormalImpulse: f32 = 0;

	/// Relative normal velocity pre-solve. Negative when approaching. This is only
	/// computed if hit events are enabled.
	normalVelocity: f32 = 0;

	/// Cached separation used for contact recycling
	baseSeparation: f32 = 0;

	/// Local point for matching
	/// Uniquely identifies a contact point between two shapes
	featureId: u32 = 0;

	/// Triangle index if one of the shapes is a mesh or height field
	triangleIndex: i32 = 0;

	/// Did this contact point exist in the previous step?
	persisted: bool = false;
}

/// A contact manifold describes the contact points between colliding shapes.
/// @note Box3D uses speculative collision so some contact points may be separated.
export struct b3Manifold {
	/// The manifold points. There may be 1 to 4 valid points.
	points: FixedArray<b3ManifoldPoint, 4>;

	/// The unit normal vector in world space, points from shape A to shape B
	normal: b3Vec3 = b3Vec3();

	/// Central friction angular impulse (applied about the normal)
	twistImpulse: f32 = 0;

	/// Central friction linear impulse
	frictionImpulse: b3Vec3 = b3Vec3();

	/// Rolling resistance angular impulse
	rollingImpulse: b3Vec3 = b3Vec3();

	/// The number of contact points, will be 0 to 4
	pointCount: i32 = 0;
}

/// Cached separating axis feature.
export type b3SeparatingFeature = i32;
export const b3_invalidAxis: b3SeparatingFeature = 0;
export const b3_backsideAxis: b3SeparatingFeature = 1;
export const b3_faceAxisA: b3SeparatingFeature = 2;
export const b3_faceAxisB: b3SeparatingFeature = 3;
export const b3_edgePairAxis: b3SeparatingFeature = 4;
export const b3_closestPointsAxis: b3SeparatingFeature = 5;

/// These are for testing
export const b3_manualFaceAxisA: b3SeparatingFeature = 6;
export const b3_manualFaceAxisB: b3SeparatingFeature = 7;
export const b3_manualEdgePairAxis: b3SeparatingFeature = 8;

/// Cached triangle feature.
export type b3TriangleFeature = i32;
export const b3_featureNone: b3TriangleFeature = 0;
export const b3_featureTriangleFace: b3TriangleFeature = 1;
export const b3_featureHullFace: b3TriangleFeature = 2;
/// v1-v2
export const b3_featureEdge1: b3TriangleFeature = 3;
/// v2-v3
export const b3_featureEdge2: b3TriangleFeature = 4;
/// v3-v1
export const b3_featureEdge3: b3TriangleFeature = 5;
export const b3_featureVertex1: b3TriangleFeature = 6;
export const b3_featureVertex2: b3TriangleFeature = 7;
export const b3_featureVertex3: b3TriangleFeature = 8;

/// Separating axis test cache. Provides temporal acceleration of collision routines.
export struct b3SATCache {
	/// The separation when the cache is populated. Negative for overlap.
	separation: f32 = 0;

	/// b3SeparatingFeature.
	type: u8 = 0;

	/// Index of the feature on shape A.
	indexA: u8 = 0;

	/// Index of the feature on shape B.
	indexB: u8 = 0;

	/// Was the cache re-used?
	hit: u8 = 0;
}

/// Contact points are always the result of two edges intersecting.
/// It can be two edges of the same shape, which is just a shape vertex.
/// Or a contact point can be the result of two edges crossing from different shapes.
/// This is designed to support hull versus hull, but it is adapted to work
/// with all shape types. The feature pair is used to identify contact points
/// for temporal coherence and warm starting.
export struct b3FeaturePair {
	/// Incoming type (either edge on shape A or shape B)
	owner1: u8 = 0;
	/// Incoming edge index (into associated shape array)
	index1: u8 = 0;
	/// Outgoing type (either edge on shape A or shape B)
	owner2: u8 = 0;
	/// Outgoing edge index (into associated shape array)
	index2: u8 = 0;
}

/// A local manifold point and normal in frame A.
export struct b3LocalManifoldPoint {
	/// Local point in frame A.
	point: b3Vec3 = b3Vec3();

	/// The contact point separation. Negative for overlap.
	separation: f32 = 0;

	/// The feature pair for this point.
	pair: b3FeaturePair = b3FeaturePair();

	/// The triangle index when collide with a mesh or height-field.
	triangleIndex: i32 = 0;
}

const b3_emptyLocalPoints = new StaticArray<b3LocalManifoldPoint>(0);

/// A local manifold with no dynamic information. Used by b3Collide functions.
/// C points into a shared point buffer; here points is the buffer and pointOffset the start,
/// so C's points[i] is points[pointOffset + i].
export class b3LocalManifold {
	/// Local normal in frame A.
	normal: b3Vec3 = b3Vec3();

	/// The triangle normal.
	triangleNormal: b3Vec3 = b3Vec3();

	/// The manifold points. From a point buffer.
	points: StaticArray<b3LocalManifoldPoint> = b3_emptyLocalPoints;

	/// Start of this manifold's points in the point buffer.
	pointOffset: i32 = 0;

	/// The number of manifold points. Only bounded by the buffer capacity.
	pointCount: i32 = 0;

	/// The index of the triangle.
	triangleIndex: i32 = 0;

	i1: i32 = 0; ///< Vertex 1 index.
	i2: i32 = 0; ///< Vertex 2 index.
	i3: i32 = 0; ///< Vertex 3 index.

	/// The squared distance of a sphere from a triangle. For ghost collision reduction.
	squaredDistance: f32 = 0;

	/// The triangle feature involved.
	feature: b3TriangleFeature = 0;

	/// b3MeshEdgeFlags.
	triangleFlags: i32 = 0;
}

// collision.h

/// Get proxy user data
export function b3DynamicTree_GetUserData(tree: b3DynamicTree, proxyId: i32): u64 {
	return tree.proxies[proxyId].userData;
}

/// Get the AABB of a proxy
export function b3DynamicTree_GetAABB(tree: b3DynamicTree, proxyId: i32): b3AABB {
	return tree.nodes[tree.proxies[proxyId].node].aabb;
}

/// Get read only hull vertices.
export function b3GetHullVertices(hull: b3HullData): StaticArray<b3HullVertex> {
	return hull.vertices;
}

/// Get read only hull points.
export function b3GetHullPoints(hull: b3HullData): StaticArray<b3Vec3> {
	return hull.points;
}

/// Get read only hull half edges.
export function b3GetHullEdges(hull: b3HullData): StaticArray<b3HullHalfEdge> {
	return hull.edges;
}

/// Get read only hull planes.
export function b3GetHullPlanes(hull: b3HullData): StaticArray<b3Plane> {
	return hull.planes;
}

/// Get read only hull faces.
export function b3GetHullFaces(hull: b3HullData): StaticArray<b3HullFace> {
	return hull.faces;
}

/// Get read only SOA vertices. This is an array of vertices with all x values,
/// y values, and z values as separate arrays. The array lengths are padded to
/// a multiple of 8. The padded values are repeats of the first value.
export function b3GetHullSoaVertices(hull: b3HullData): StaticArray<f32> {
	return hull.soaVertices;
}

/// Get read only SOA unit normal vectors. This is an array of normals with all x values,
/// y values, and z values as separate arrays. The array lengths are padded to
/// a multiple of 8. The padded values are 0.
export function b3GetHullSoaNormals(hull: b3HullData): StaticArray<f32> {
	return hull.soaNormals;
}

/// Get the dot products of the two adjacent face normals for each full edge.
export function b3GetHullEdgeCosines(hull: b3HullData): StaticArray<f32> {
	return hull.edgeCosines;
}

/// Get read only mesh BVH nodes.
export function b3GetMeshNodes(mesh: b3MeshData): StaticArray<b3MeshNode> {
	return mesh.nodes;
}

/// Get read only mesh vertices.
export function b3GetMeshVertices(mesh: b3MeshData): StaticArray<b3Vec3> {
	return mesh.vertices;
}

/// Get read only mesh triangles.
export function b3GetMeshTriangles(mesh: b3MeshData): StaticArray<b3MeshTriangle> {
	return mesh.triangles;
}

/// Get read only mesh materials. The count is equal to the triangle count.
export function b3GetMeshMaterialIndices(mesh: b3MeshData): StaticArray<u8> {
	return mesh.materialIndices;
}

/// Get read only mesh flags. The count is equal to the triangle count.
export function b3GetMeshFlags(mesh: b3MeshData): StaticArray<u8> {
	return mesh.flags;
}

/// Get read only compressed heights. One uint16_t per grid point.
export function b3GetHeightFieldCompressedHeights(hf: b3HeightFieldData): StaticArray<u16> {
	return hf.compressedHeights;
}

/// Get read only material indices. One uint8_t per cell.
export function b3GetHeightFieldMaterialIndices(hf: b3HeightFieldData): StaticArray<u8> {
	return hf.materialIndices;
}

/// Get read only triangle flags. One uint8_t per triangle.
export function b3GetHeightFieldFlags(hf: b3HeightFieldData): StaticArray<u8> {
	return hf.flags;
}

/// Query callback. The closure carries C's context.
export type b3MeshQueryFcn = (a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32) => bool;

// Calls a b3MeshQueryFcn, or the visit method of a visitor class directly, which parallel parts can run
@inline export function b3VisitMeshTriangle<V>(visitor: V, a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32): bool {
	if (isFunction<V>()) {
		return visitor(a, b, c, triangleIndex);
	}
	return visitor.visit(a, b, c, triangleIndex);
}

/// This is used for computing 2D hulls in service of contact manifold simplification.
/// Placed here for testing. Internal.
export struct b3Point2D {
	p: b3Vec2 = b3Vec2(); // 2D point
	separation: f32 = 0; // separation of the associated contact point
	originalIndex: i32 = 0; // original index of the associated contact point
}
