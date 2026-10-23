# mesh: triangle meshes and height fields

## Ported

| C | AnimaScript |
|---|---|
| `src/mesh.c` | `lib/mesh.as` |
| `src/height_field.c` | `lib/height_field.as` |

Mesh: vertex welding with its spatial hash, binned SAH, half and median
splits, the recursive BVH build, DFS triangle sorting, edge identification with
concave and inverse concave flags, `b3CreateMesh`, the grid, wave, torus, box,
hollow box and platform generators, `b3GetHeight`, `b3IsValidMesh`,
`b3ComputeMeshAABB`, `b3RayCastMesh`, `b3ShapeCastMesh`, `b3OverlapMesh`,
`b3QueryMesh`, `b3CollideMoverAndMesh`, `b3DestroyMesh` (no-op).

Height field: `b3CreateHeightField` with height compression and the convexity
flags, the cell corner decode, `b3GetHeightFieldTriangle`,
`b3GetHeightFieldMaterial`, `b3ComputeHeightFieldAABB`, `b3RayCastHeightField`,
`b3ShapeCastHeightField`, `b3OverlapHeightField`, `b3QueryHeightField`,
`b3CollideMoverAndHeightField`, `b3CreateGrid`, `b3CreateWave`,
`b3DestroyHeightField` (no-op).

## Signatures other modules call

```as
// lib/mesh.as
b3CreateMesh(def: b3MeshDef, degenerateTriangleIndices: StaticArray<i32>?, degenerateCapacity: i32): b3MeshData?
b3CreateGridMesh(xCount: i32, zCount: i32, cellWidth: f32, materialCount: i32, identifyEdges: bool): b3MeshData?
b3CreateWaveMesh(xCount: i32, zCount: i32, cellWidth: f32, amplitude: f32, rowFrequency: f32, columnFrequency: f32): b3MeshData?
b3CreateTorusMesh(radialResolution: i32, tubularResolution: i32, radius: f32, thickness: f32): b3MeshData?
b3CreateBoxMesh(center: b3Vec3, extent: b3Vec3, identifyEdges: bool): b3MeshData?
b3CreateHollowBoxMesh(center: b3Vec3, extent: b3Vec3): b3MeshData?
b3CreatePlatformMesh(center: b3Vec3, height: f32, topWidth: f32, bottomWidth: f32): b3MeshData?
b3DestroyMesh(mesh: b3MeshData): void
b3GetHeight(mesh: b3MeshData): i32
b3IsValidMesh(meshData: b3MeshData?): bool
b3GetMeshNodeBits(node: b3MeshNode): u32
b3ComputeMeshAABB(shape: b3MeshData, transform: b3Transform, scale: b3Vec3): b3AABB
b3RayCastMesh(mesh: b3Mesh, input: b3RayCastInput): b3CastOutput
b3ShapeCastMesh(mesh: b3Mesh, input: b3ShapeCastInput): b3CastOutput
b3OverlapMesh(shape: b3Mesh, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool
b3QueryMesh(mesh: b3Mesh, bounds: b3AABB, fcn: b3MeshQueryFcn): void
b3CollideMoverAndMesh(planes: StaticArray<b3PlaneResult>, planeOffset: i32, capacity: i32, shape: b3Mesh, mover: b3Capsule): i32

// lib/height_field.as
b3CreateHeightField(data: b3HeightFieldDef): b3HeightFieldData
b3CreateGrid(rowCount: i32, columnCount: i32, scale: b3Vec3, makeHoles: bool): b3HeightFieldData
b3CreateWave(rowCount: i32, columnCount: i32, scale: b3Vec3, rowFrequency: f32, columnFrequency: f32, makeHoles: bool): b3HeightFieldData
b3DestroyHeightField(heightField: b3HeightFieldData): void
b3GetHeightFieldTriangle(heightField: b3HeightFieldData, triangleIndex: i32): b3Triangle
b3GetHeightFieldMaterial(heightField: b3HeightFieldData, triangleIndex: i32): i32
b3ComputeHeightFieldAABB(shape: b3HeightFieldData, transform: b3Transform): b3AABB
b3RayCastHeightField(heightField: b3HeightFieldData, input: b3RayCastInput): b3CastOutput
b3ShapeCastHeightField(heightField: b3HeightFieldData, input: b3ShapeCastInput): b3CastOutput
b3OverlapHeightField(shape: b3HeightFieldData, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool
b3QueryHeightField(heightField: b3HeightFieldData, bounds: b3AABB, fcn: b3MeshQueryFcn): void
b3CollideMoverAndHeightField(planes: StaticArray<b3PlaneResult>, planeOffset: i32, capacity: i32, shape: b3HeightFieldData, mover: b3Capsule): i32
```

`b3MeshQueryFcn` is the collision.as alias `(a: b3Vec3, b: b3Vec3, c: b3Vec3,
triangleIndex: i32) => bool`; the closure carries C's context. The height
field query ignores the return value, as C does.

## Signatures that differ from C, and why

- Mover functions take `(planes, planeOffset, capacity, ...)`: C's compound
  callback passes `planes + planeCount`, so the pointer becomes the array plus
  the index it points at (the `b3LoadW4(data, index)` rule). Results land in
  `planes[planeOffset .. planeOffset + count)`. The capsule and sphere movers
  write one result through `ref b3PlaneResult` instead.
- `const b3RayCastInput*` and `const b3Capsule*` are passed by value (small,
  read only). Class inputs (`b3ShapeCastInput`, `b3ShapeProxy`, `b3Mesh`) are
  references, as everywhere.
- Creators return `b3MeshData?` where C may return NULL. The height field
  creators never return NULL in C either.
- `b3IsValidMesh` takes `b3MeshData?` for C's NULL check.
- `b3GetMeshNodeBits` is new: the 32 bits C's `data` union holds (leaf view on
  a leaf, else node view), used for the hash image and by the driver.
- Private: `b3WeldVertices` takes the `b3WeldData` members as parameters,
  the split functions and `b3BuildRecursive` take `(primitives, base)` for C's
  `primitives + base` pointer, `b3StoreLeaf` takes `(nodes, index)`,
  `b3BuildRecursive` returns its height through `ref i32` bound to a
  `FixedArray<i32, 2>` element, node pointers are node indices, and
  `b3GetHeightFieldCellCorners` returns a `FixedArray<b3Vec3, 4>`.

## Deviations

- Hashes: `b3Hash64NonZero` runs over a serialized C byte image of the blob
  (`b3SerializeMesh`, `b3SerializeHeightField`, a DataView in C field order
  with the union bits and zero padding), so `hash` matches C for every mesh and
  height field built in the checks.
- verstable maps become `Map<u64, i32>`. Welding and edge identification only
  look keys up and never iterate, so the map's internal order cannot change
  results.
- `b3MeshDef.stride` is checked as C checks it (0, or a multiple of 4 from 12 to
  4096, else NULL), but vertices are always read contiguously, as types.md
  decided for `b3MeshDef`. A valid stride other than 0 or 12 reads differently
  from C; none is tested.
- C's NULL checks on `def->vertices` and `def->indices` are gone (the arrays
  are never null); a short array traps instead of reading past it.
- `b3SortMeshTriangles` swaps in the sorted arrays instead of copying back.
- Float to int casts (`(int)floorf`, the bin index, the u16 height cast) go
  through `b3FloatToIntSat` / `b3FloatToUIntSat`, so out of range input
  saturates as on the reference target instead of trapping.
- Traversal stacks: ray cast, shape cast, overlap and mover share one module
  global `StaticArray<i32>(256)`; none of them calls user code. `b3QueryMesh`
  allocates its own, since its callback may query again. A `FixedArray` of 256
  is refused (lengths stop at 255).
- `b3MakeLocalProxy` and `b3ComputeProxyAABB` (`src/shape.c`) are private
  copies in both files until `lib/shape.as` ports them; replace them with imports
  then.
- `b3GetMeshTriangle` and `b3GetHeightFieldTriangleCount` are shape.h inlines,
  owned by `lib/shape.as`; the drivers import them from there and verify them
  against C.
- The mover proxies use `b3MakeSegmentProxy` and the height field ray cast
  `b3MakePointProxy` from lib/distance.as, for C's `&mover->center1` and
  `&input->origin`.
- The unused `center1`/`center2` of `b3CreateHeightField` are dropped.

## Skipped

- `b3DumpHeightData`, `b3LoadHeightField` (file IO).
- Validation build only code (`B3_ENABLE_VALIDATION`, off in the reference
  build): `b3IsDegenerate`, `b3IsNonDegenerate`, `b3GetNodeAABB`,
  `b3IsConsistent`, `b3ValidateSplit` and that build's `b3IsValidMesh`.
- The `*Write` accessors (`b3GetMeshNodesWrite`, `b3GetRootWrite`, ...): the
  arrays are written directly. `b3GetRoot` is index 0.
- `b3GetStridedVertex`, `b3CopyVerticesWithStride` (see stride above),
  `b3CreateSpatialHash` / `b3DestroySpatialHash` (a constructor, and the
  runtime frees).

## Verification

`verify/run.sh mesh height_field`, every value compared exactly, no `nanEqual`:

| module | lines | values | mismatches |
|---|---|---|---|
| mesh | 210614 | 2281334 | 0 |
| height_field | 79261 | 873926 | 0 |

Both also pass with `RASC_FLAGS=` (no `-O`) and with `-O -D B3_ENABLE_ASSERT`
(no assert fires). `rasc check` is clean on both lib files and both drivers.

mesh (81 builds, 74 valid, 72 queried; each query block runs for 6 scales
including reflections and non-uniform scale):

- Builds: all 20 generator calls (grids with and without materials and edges,
  waves, tori, boxes, hollow boxes, platforms), 40 seeded soups with exact,
  jittered and collinear duplicate vertices, repeated index and zero area
  triangles, random welding tolerance, SAH or median split, edge
  identification, clockwise winding, optional materials and a degenerate index
  buffer of random capacity, 12 per triangle vertex grids welded back
  together, and the NULL cases (bad strides, too few vertices, no triangles, all
  degenerate). Every header field including `hash`, every node (bounds, union
  bits, triangle offset), vertex, triangle, material and flag, `b3GetHeight`,
  `b3IsValidMesh`, the degenerate indices.
- 70788 `b3GetMeshTriangle` results, 1728 transformed AABBs.
- 17280 ray casts (8669 hits): random rays and rays aimed at triangle
  centroids, partial max fractions; every output field.
- 5184 box queries with 50307 callbacks, 1333 of them stopped early by the
  callback.
- 5184 overlaps (1601 true) with point, segment and hull proxies with radius,
  7344 shape casts (4682 hits, with and without encroach), 6912 mover
  collisions with 13678 planes at random offsets and capacities (1953 hit the
  capacity).

height_field (42 builds, 36 queried):

- Builds: 3 grids, 3 waves (with holes), 30 seeded fields with random scale,
  height range (including a zero range that clamps to the linear slop),
  clockwise winding, random holes and optional materials, and 6 more with
  infinities and NaN heights (construction only). Header fields including
  `hash`, every compressed height, material and both flags per cell, and
  29272 `b3GetHeightFieldTriangle` + `b3GetHeightFieldMaterial` results.
- 7200 ray casts (2955 hits): random, vertical and aimed at triangles.
- 360 queries with 15402 callbacks, 144 transformed AABBs.
- 1440 overlaps (956 true), 2196 shape casts (1249 hits) including drops from
  above, 1440 mover collisions with 4275 planes.

The cast stage ran against `lib/distance.as` as it stood on 2026-10-23; rerun
when that file changes.

Not covered: a BVH deep enough to overflow the 256 entry sort stack (C returns
NULL there, the port too, untested), strided vertices.

## rasc findings

No new ones, no compiler or std changes. Hit on the way, already known:

- `T | null` is refused ("union types are not supported; write T? for a
  nullable type"); the message is clear.
- `FixedArray` lengths stop at 255, so C's `stack[256]` became a
  `StaticArray<i32>`.

Worked as hoped and worth knowing: a `ref i32` parameter binds a
`FixedArray<i32, N>` element of a local, across recursion; `Map<u64, i32>`;
`|=` on `StaticArray<u8>` elements; `ref` bindings to `StaticArray` struct
elements in loops.
