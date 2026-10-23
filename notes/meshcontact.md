# meshcontact: mesh and height field manifolds, compounds

## Ported

| C | AnimaScript |
|---|---|
| `src/mesh_contact.c` | `lib/mesh_contact.as` (new) |
| `src/compound.c` | `lib/compound.as`, below the decls marker |

mesh_contact.c, every function in C order: the triangle index query context and
callback, `b3QueryMeshTriangles`, `b3QueryHeightFieldTriangles`,
`b3RefreshCache` (query bounds, sorted merge of the old triangle caches, the
once only capacity warning), `b3TentativeTriangle`, `b3FoundEdges` /
`b3AddEdge` / `b3FindEdge`, `b3FoundVertices` / `b3AddVertex`,
`b3IsBetterCullCandidate`, `b3CullPoints`, `b3ReduceCluster`, `b3Cluster`,
`b3ComputeMeshManifolds` (4 lane triangle transform and hull bounds cull through
the scalar W4 lanes, per survivor triangle collide, ghost edge and vertex
filtering with the sphere QSORT path, clustering, cluster reduction, old
manifold matching with impulse and point persistence, per triangle material
mixing, rolling resistance and tangent velocity).

compound.c, every compiled function in C order: `b3GetCompoundMaterials`,
`b3GetCompoundCapsule`, `b3GetCompoundHull`, `b3GetCompoundMesh`,
`b3GetCompoundSphere`, `b3GetCompoundChild`, the mesh and material maps,
`b3CreateCompound`, `b3DestroyCompound` (no-op), `b3ComputeCompoundAABB`,
`b3OverlapCompound`, `b3RayCastCompound`, `b3ShapeCastCompound`,
`b3QueryCompound`, `b3MakeCompoundChildSweep`, `b3CollideMoverAndCompound`, and
their callbacks.

## Signatures other modules call

```as
// lib/mesh_contact.as
export function b3ComputeMeshManifolds(world: b3World, workerIndex: i32, contact: b3Contact, shapeA: b3Shape,
	materialMap: Uint16Array?, xfA: b3WorldTransform, shapeB: b3Shape, xfB: b3WorldTransform, isFast: bool): bool

// lib/compound.as
export function b3GetCompoundTreeView(compound: b3CompoundData): b3TreeView
export function b3GetCompoundMaterials(compound: b3CompoundData): StaticArray<b3SurfaceMaterial>?
export function b3GetCompoundCapsule(compound: b3CompoundData, index: i32): b3CompoundCapsule
export function b3GetCompoundHull(compound: b3CompoundData, index: i32): b3CompoundHull
export function b3GetCompoundMesh(compound: b3CompoundData, index: i32): b3CompoundMesh
export function b3GetCompoundSphere(compound: b3CompoundData, index: i32): b3CompoundSphere
export function b3GetCompoundChild(compound: b3CompoundData, childIndex: i32): b3ChildShape
export function b3CreateCompound(def: b3CompoundDef): b3CompoundData?
export function b3DestroyCompound(compound: b3CompoundData): void
export function b3ComputeCompoundAABB(shape: b3CompoundData, transform: b3Transform): b3AABB
export function b3OverlapCompound(shape: b3CompoundData, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool
export function b3RayCastCompound(shape: b3CompoundData, input: b3RayCastInput): b3CastOutput
export function b3ShapeCastCompound(shape: b3CompoundData, input: b3ShapeCastInput): b3CastOutput
export function b3QueryCompound(compound: b3CompoundData, aabb: b3AABB, fcn: b3CompoundQueryFcn): void
export function b3MakeCompoundChildSweep(compoundTransform: b3Transform, childTransform: b3Transform): b3Sweep
export function b3CollideMoverAndCompound(planes: StaticArray<b3PlaneResult>, capacity: i32,
	shape: b3CompoundData, mover: b3Capsule): i32
```

The current callers in `lib/contact.as`, `lib/shape.as`, `lib/sensor.as`,
`lib/broad_phase.as` and `lib/box3d.as` already use these shapes.

## Signatures that differ from C, and why

- `b3ComputeMeshManifolds` drops `b3Arena arena` (CONVENTIONS). `materialMap` is
  a nullable `Uint16Array`, the child's `materialIndices` view in the compound
  path, null where C passes NULL.
- `b3CollideMoverAndCompound` keeps C's `(planes, capacity, ...)`: no caller
  passes an offset pointer. Inside, the mesh child call passes
  `(planes, planeCount, capacity - planeCount, ...)` to the mesh mover's
  `(planes, planeOffset, capacity, ...)`, and the convex children write
  `planes[planeCount]` through their `ref b3PlaneResult`.
- `b3GetCompoundMaterials` returns null where C returns NULL (material offset 0,
  which `b3CreateCompound` never produces).
- `b3CreateCompound` returns null where C returns NULL (too many children).
- `b3QueryCompound` takes the closure; C's `void* context` is dropped.
- Private: `b3RefreshCache` takes the bounds by value; the found edge and vertex
  helpers take `ref` structs; `b3ReduceCluster(points, pointOffset, count,
  normal)` for C's point pointer; `b3Cluster.pointOffset` replaces `points`; C's
  maps become `b3MaterialMap_get_or_insert` / `b3MeshMap_get_or_insert`
  returning the value instead of an iterator. The tree callbacks are static
  functions taking their context class last, wrapped in a closure at the call.

## Deviations

- Compound layout: the compound holds one array per section. Shared hulls and
  meshes are referenced (`compound.hulls`, `compound.meshes`), not copied into a
  blob, and the instances keep the shared index in `hullOffset` / `meshOffset`
  and the element start in `materialOffset`, as notes/types.md decided. Every
  offset field and `byteCount` is still computed with the C byte sizes (node 32,
  material 40, capsule 32, hull instance 36, mesh instance 52, sphere 20), so the
  header matches C exactly. The hull and mesh data are immutable after creation,
  so sharing instead of copying cannot change results.
- Dedup maps: C keys verstable by content (memcmp). Materials: a bucket map keyed
  by an FNV hash of the 40 byte image with a bitwise field compare (so `-0` and
  `+0` friction are different materials, as with memcmp). Hulls: the
  `b3HullMap` from lib/hull.as. Meshes: buckets keyed by `mesh.hash` (part of
  the content) with a field by field bitwise compare of header, nodes (union bits
  through `b3GetMeshNodeBits`), vertices, triangles, materials and flags. Values
  are assigned in insertion order, so no hash order reaches the results.
- `b3GetCompoundChild` allocates a `b3ChildShape` per call and, for convex
  children, a one element `Uint16Array` holding the material index (C points at
  the instance's field). Mesh children get a `subarray` view of
  `meshMaterialIndices`. This allocates in every tree callback; a reusable child
  could replace it later without changing results.
- Mesh manifolds: C's arena buffers are local arrays. The manifold buffer holds
  `b3LocalManifold` objects created on first use; a slot whose collide produced
  no points is reused for the next survivor, as C reuses the bump memory. The
  accepted and tentative pointer arrays hold references to those objects. C's
  `b3ContactCache* cache = &triangleCaches[index].cache` points into a growable
  array, so the cache is copied out, passed by `ref` field, and stored back. The
  survivor triangle is copied into a three element array for the triangle
  colliders, which take the three vertices as `StaticArray<b3Vec3>`.
- `contact.manifolds = NULL` becomes `b3AllocateManifolds(world, 0)` (the shared
  empty array). The memset of reused manifolds assigns a zero struct
  (`let zero: b3Manifold;`), not `b3Manifold()`.
- The capacity warning goes through `b3Log` with a built string.
- `b3MakeLocalProxy` and `b3ComputeProxyAABB` come from lib/shape.as.

## Skipped

- `b3ValidateCompound(bytes, byteCount)`: it validates a serialized byte blob
  (replay only); the port has no blob to validate.
- `b3CompoundTimeOfImpact` and `b3CompoundTimeOfImpactFcn`: declared in
  compound.h but inside `#if 0` in compound.c at the pin. shape.c has its own
  compound TOI path, which uses `b3QueryCompound` and `b3MakeCompoundChildSweep`.
- `b3HashMesh` / `vt_wyhash` (replaced, see Deviations).
- mesh_contact.c: `b3IsSorted` (validation only), `b3TrianglesShareEdge` and
  the edge connected clustering (`#if 0`), the `B3_MAX_MANIFOLD_POINTS != 4`
  branch of `b3ReduceCluster` (`b3Hull2D` / `b3SimplifyHull2D`; the constant is
  4).

## Calls into unported code

None. Every callee exists and the calls match its actual signature:
`b3CollideTriangleAndSphere`, `b3CollideTriangleAndCapsule`,
`b3CollideTriangleAndHull` (lib/triangle_manifold.as); `b3OverlapHull`,
`b3RayCastHull`, `b3ShapeCastHull`, `b3CollideMoverAndHull`,
`b3ComputeHullAABB`, `b3HullMap_*` (lib/hull.as); `b3MakeFeatureId`
(lib/manifold.as); `b3MakeLocalProxy`, `b3ComputeProxyAABB`,
`b3GetMeshTriangle`, `b3GetShapeMaterials` (lib/shape.as);
`b3AllocateManifolds`, `b3FreeManifolds` (lib/physics_world.as).

## Verification

`verify/run.sh compound mesh_contact`, exact, no `nanEqual`:

| module | lines | values | mismatches |
|---|---|---|---|
| compound | 18685 | 183358 | 0 |
| mesh_contact | 489724 | 3765703 | 0 |

Both pass under `-O` and with `RASC_FLAGS=` (no `-O`). With
`-O -D B3_ENABLE_ASSERT` mesh_contact passes and compound passes every line up to
the deliberate too-many-children case, where `B3_ASSERT(false)` fires as it does
in a C debug build. The C drivers built at `-O0` against the source files
instead of the archive print identical vectors. `rasc check` is clean on both
lib files apart from the C-named unused `proxyId` callback parameters.

compound (`verify/cref/compound.c`, `verify/as/compound.as`): a pool of 10 hulls
(two identical boxes, an offset box, cylinder, cone, five random hulls, one
rebuilt from another's points) and 7 meshes (two identical box meshes built
separately, grids with 3 and 5 materials, torus, wave, platform), 7 materials
with a repeat and a `-0` friction. 60 seeded compounds of mixed capsules, hulls,
meshes (scaled, some reflected) and spheres, every tenth one large (134
children); the first forces sharing by pointer and by identical content. 27
compounds deduplicate hulls, 8 meshes. Printed field by field and array by
array: header, all 2572 tree nodes with the union value, materials, capsules,
shared hulls and meshes, hull and mesh instances with their shared index and
material start (C's byte offsets converted to indices), the mesh material map,
spheres, the C blob tail check, and `b3GetCompoundChild` for all 1286 children.
Then 240 AABBs, 720 queries (494 callbacks, early stops), 2400 ray casts (711
hits), 1800 shape casts with point, segment and box proxies (951 hits) plus one
empty proxy per compound, 1800 overlaps (665 true), 1800 mover collisions with
random capacities (1112 planes), 1200 child sweeps and 1105 TOI from them (710
separated, 390 hit, 5 overlapped). The too many children case returns null.
C line coverage of compound.c is 91%; the misses are the zero offset returns
behind asserts, the pointer-equal compare shortcuts, the default switch arms and
`b3ValidateCompound`.

mesh_contact (`verify/cref/mesh_contact.c`, `verify/as/mesh_contact.as`): a
real world from `b3CreateWorld` in C for the task context, block allocator and
default mixing callbacks; the AS world gets one task context and the same
callbacks. 120 seeded sequences of 40 calls each (4800 calls) with one contact
carried through the sequence: shape A is one of 6 meshes (grid with materials,
wave, torus, box, a 60x60 fine grid, platform; scaled, a quarter reflected) or 4
height fields (wave with holes, grid, a custom field with materials and holes,
a 90x90 fine wave), at identity, translated or rotated. Shape B is a sphere, a
capsule (a third lying flat along the mesh plane, unrotated) or one of 5 hulls,
walking over the surface with drift, rotation and occasional jumps, with
speculative points on or off and random `isFast`. Materials: none (the else
branch), inline, three per triangle, and three through a compound style
material map. Between calls the driver writes random solver impulses so the
next call has impulses to match and preserve. Per call: the return, every
manifold (normal, impulses, all points with anchors, separation, impulses,
feature id, triangle index, persisted), friction, restitution, rolling
resistance, tangent velocity, the query bounds, the SAT counters and every
triangle cache entry with its SAT or simplex cache. 2306 calls touch, 19295
manifolds (up to 122 clusters), 43805 points of which 16984 persisted, 421704
cache entries, 279 calls at the 512 triangle capacity (mesh and height field).
C line coverage of mesh_contact.c is 97%; the misses are the default switch
arms, `count <= 1` in `b3CullPoints` (the caller returns first), the
`clusterPointCount == 0` and null membership guards, and the collinear return of
`b3CullPoints`: its candidate test breaks a zero area tie on separation against
`FLT_MAX`, so `bestIndex` stays null only with infinite or NaN separations.

The checks ran against lib/triangle_manifold.as, lib/hull.as and
lib/physics_world.as as they stood on 2026-10-23; rerun when those change.

## rasc findings

No compiler or std changes, nothing new. Confirmed working and relied on:
nested `FixedArray<FixedArray<b3Vec3, 4>, 3>` locals with runtime indices;
passing a struct field of a struct local (`cache.satCache`) to a `ref`
parameter; `ref` bindings to struct elements of a class's `StaticArray` field
(`contact.manifolds[i]`) and to FixedArray elements inside them; QSORT closures
that swap elements of a captured `StaticArray` of structs; `Uint16Array?`
parameters and `subarray` views. Hit again, already known: `T | null` is
refused, spelled `T?`.
