# types: the foundation modules

## What is ported

| C | AnimaScript |
|---|---|
| `include/box3d/math_functions.h`, `src/math_functions.c` | `lib/math_functions.as` |
| `src/math_internal.h` | `lib/math_internal.as` |
| `include/box3d/base.h`, `src/core.h`, `src/core.c`, `b3Hash` from `src/timer.c`, rapidhash | `lib/core.as` |
| `include/box3d/constants.h` | `lib/constants.as` |
| `include/box3d/id.h`, the world, body, shape, joint, event and result parts of `types.h`, `src/types.c`, the joint and explosion defaults from `src/joint.c` | `lib/types.as` |
| `include/box3d/collision.h` inline functions, plus the query, tree, mover, geometry and collision groups of `types.h` | `lib/collision.as` |
| `src/aabb.h`, `src/aabb.c` | `lib/aabb.as` |

The modules import each other in a cycle (math_functions needs collision for
`b3RayCastInput` and the triangle features, collision needs types for
`b3SurfaceMaterial`); rasc resolves cyclic imports fine.

## Verification

`verify/run.sh math`: 81595 lines, 2767851 values, 0 mismatches. The same
wasm passes under `--runtime stub`, without `-O`, and with
`-D B3_ENABLE_ASSERT` (no assert fires on the inputs).

Covered: every scalar, vector, vec2, matrix2, quaternion, transform, matrix,
AABB, plane, segment and triangle function in both headers and `math_functions.c`,
`b3RayCastAABB`, `b3EnlargeAABB`, `b3FarthestPointOnAABB`, `b3Perimeter`, the
saturating casts against C casts, `b3Hash` and `b3Hash64NonZero` over lengths
0 to 511, all id store/load functions, every constant, every `b3Default*Def`
field at length units 1 and 0.37, the unit dependent constants at 1, 0.37 and
100, and the blob layouts (offsets and byteCount of `b3HullData`, `b3MeshData`,
`b3HeightFieldData` against the C layout code, and the `b3BoxHull` offsets).
Inputs are fixed cases plus seeded sweeps through `vspecialf` (signed zeros,
denormals, infinities, NaN payloads, huge, tiny). `instantiate()` in
`verify/as/math.as` builds every struct and class and is called by the runner.

Explained differences, not failures:

- 442 lines differ only in NaN payload or sign; `verify/math.mjs` runs with
  `nanEqual` (set `NAN_EXACT=1` to see them). Root cause: the C driver is built
  at -O2 as instructed and clang commutes operations whose operands are both
  NaN, which picks the other payload. With the driver at -O0, 179 remain, all
  from functions inside libbox3d.a (built optimized) plus one libm quirk: Apple
  `remainderf` returns a signaling NaN unchanged where musl's (rasc
  `Mathf.rem`) quiets it.
- `b3SafeScale` is fed quieted inputs in both drivers. clang lowers
  `a > b ? a : b` against a constant to `fmaxnm`, which turns a signaling NaN
  into a quiet NaN where the select returns the constant. Only signaling NaN
  inputs differ, and no arithmetic produces one.

## Design notes for other porters

### Structs, aliases, enums

- All plain data is a `struct` with zeroed defaults and a positional
  constructor where C code builds literals: `b3Vec3(x, y, z)`,
  `b3Quat(v, s)`, `b3Transform(p, q)`, `b3Matrix3(cx, cy, cz)`,
  `b3AABB(lower, upper)`, `b3Plane(normal, offset)`, `b3Capsule(c1, c2, r)`,
  `b3Sphere(c, r)`, `b3RayCastInput(o, t, maxFraction)`,
  `b3BoxCastInput(box, t, maxFraction)`, `b3Filter(cat, mask, group)`,
  `b3HullHalfEdge(next, twin, origin, face)`, `b3MeshTriangle(i1, i2, i3)`,
  `b3TrianglePoint(point, feature)`. `S()` is always the zero value.
- `b3Pos = b3Vec3`, `b3WorldTransform = b3Transform`,
  `b3WorldCastOutput = b3CastOutput` are type aliases, as in C single
  precision. An alias cannot be called, so build them with `b3Vec3(...)`.
- Enums are `export type b3X = i32` plus one `export const` per enumerator.
- Statics such as `b3Vec3_zero`, `b3Quat_identity`, `B3_BOUNDS3_EMPTY`,
  `b3_emptyDistanceCache`, `b3_nullBodyId` are value globals.
- Ids are integer structs, so `==` compares every field: that is
  `B3_ID_EQUALS` (b3ContactId also compares its always zero padding).
  `B3_IS_NULL(id)` is `id.index1 == 0`. `b3StoreContactId` and
  `b3LoadContactId` take a `StaticArray<u32>` of 3.
- Construct structs from a named import (`import { b3Sphere } ...`): rasc
  refuses `ns.b3Sphere()` through `import * as ns` (see rasc findings).

### Classes

Classes, because they hold pointers, strings, callbacks or C only uses them
through a pointer: `b3WorldDef`, `b3BodyDef`, `b3ShapeDef`, `b3ShapeProxy`,
`b3ShapeCastInput`, `b3ShapeCastPairInput`, `b3DistanceInput`, `b3TOIInput`,
`b3DynamicTree`, `b3HullData`, `b3BoxHull`, `b3MeshDef`, `b3MeshData`,
`b3Mesh`, `b3HeightFieldDef`, `b3HeightFieldData`, `b3CompoundHullDef`,
`b3CompoundMeshDef`, `b3CompoundDef`, `b3CompoundData`, `b3CompoundHull`,
`b3CompoundMesh`, `b3ChildShape`, `b3LocalManifold`, `b3SensorEvents`,
`b3ContactEvents`, `b3BodyEvents`, `b3JointEvents`, `b3ContactData`.

C copies some of these by value (`b3ShapeProxy proxyA = input->proxyA;`).
A class assignment aliases, so clone where C mutates the copy.

`b3ShapeProxy(points: StaticArray<b3Vec3>, count, radius)`: `points` is the
array C points into; only the first `count` are used. For a hull pass
`b3GetHullPoints(hull)` directly. C's `&capsule->center1` (two consecutive
fields) and `&center` need a small array holding those points. Every proxy
holder defaults to an empty proxy.

`b3LocalManifold` adds `pointOffset`: C points into a shared point buffer
(`manifold->points = pointBuffer + totalPointCount`), so `points` is the
buffer and C's `points[i]` is `points[pointOffset + i]`.

`b3DynamicTree`: every C pointer is a `StaticArray` (initially length 0);
growth replaces the array with a larger copy.

`b3ContactData.manifolds` is a nullable `StaticArray<b3Manifold>` referencing
the contact's manifold storage. Event containers hold the world's
`Array<T>` event arrays plus the C counts.

### Blob classes: hull, mesh, height field, compound

Each C section is its own array; every scalar field is kept, including the
offsets and `byteCount`, which keep the values C computes. Accessors with the C
names return the arrays and never null (an empty section is a length 0 array).

`b3HullData(vertexCount, edgeCount, faceCount)` allocates
`vertices`, `points`, `edges`, `planes`, `faces`, `soaVertices`,
`soaNormals`, `edgeCosines` and sets the counts, offsets and byteCount exactly
as `b3CreateHull` does (verified). The SOA arrays hold x, then y, then z, each
`b3GetHullSoaStride(count)` long, so C's `vy = vx + soaVertexCount` is
index `soaVertexCount + i`. b3CreateHull becomes: build the temporaries,
`let hull = new b3HullData(vertexCount, edgeCount, faceCount)`, set
`version`, fill the arrays, then bounds, mass and hash. `b3CloneHull` and
`b3CloneAndTransformHull` must deep copy the arrays (C memcpy's the blob).

`b3BoxHull()` makes `base = new b3HullData(8, 24, 6)` with the C box offsets
and `byteCount = 696` (`B3_BOX_HULL_*` constants, verified against offsetof),
and aliases `boxVertices`, `boxPoints`, `boxEdges`, `boxPlanes`, `boxFaces`,
`soaVertices`, `soaNormals`, `edgeCosines` to base's arrays, so writes through
either name land in the hull. C's `vx`, `vy`, `vz` are `soaVertices[0..8)`,
`[8..16)`, `[16..24)`; `nx`, `ny`, `nz` likewise in `soaNormals`.
`b3MakeTransformedBoxHull` becomes `let boxHull = new b3BoxHull()`, copy the
`s_boxHull` topology (vertices, edges, faces, counts, version), then fill the
rest as C does. Callers pass `boxHull.base` where C passes `&boxHull.base`.

`b3MeshData(nodeCount, vertexCount, triangleCount)` allocates `nodes`,
`vertices`, `triangles`, `materialIndices`, `flags` and sets counts, offsets
and byteCount as `b3CreateMesh` does (verified). `materialCount` and the rest
are set by the porter. `b3Mesh(data, scale)` is a class.

`b3MeshNode.data` keeps C's union names: `data.asNode.axis`,
`data.asNode.childOffset`, `data.asLeaf.type`, `data.asLeaf.triangleCount`,
bit-fields of 2 and 30 bits. The two views do not alias: an internal node
never writes `asLeaf`, so `asLeaf.type` reads 0 there, never the leaf marker 3,
which is what `b3IsLeaf` needs. Never turn a leaf into an internal node in place
without clearing `asLeaf`.

`b3HeightFieldData(columnCount, rowCount)` allocates `compressedHeights`
(u16, one per grid point), `materialIndices` (u8 per cell), `flags` (u8 per
triangle) with the C offsets and byteCount (verified).

`b3CompoundData()` starts empty; `b3CreateCompound` fills `nodes`
(`b3TreeNode`), `materials`, `capsules`, `hullInstances`, `hulls` (the shared
unique hulls), `meshInstances`, `meshMaterialIndices`, `meshes` (shared unique
meshes), `spheres`, plus the scalars. The compound.c private
`b3HullInstance` and `b3MeshInstance` live in collision.as for the field
types: `hullOffset` is an index into `hulls`, `meshOffset` an index into
`meshes`, `materialOffset` an element index into `meshMaterialIndices`, where C
stores byte offsets. `meshMaterialIndices` is a `Uint16Array` so
`b3CompoundMesh.materialIndices` and `b3ChildShape.materialIndices` can be
`subarray` views, keeping C's pointer indexing (`child.materialIndices[0]`).
Convex children need a view or a 1 element array holding their material index.

`b3ChildShape` keeps one field per union member (`capsule`, `hull?`, `mesh?`,
`sphere`) plus `type`. `b3TreeNode` keeps `height` and `shapeIndex` as two
non aliasing fields.

Hashes: `b3Hash64NonZero(bytes: Uint8Array, n)` is rapidhash, bit exact. The
blobs are not byte images, so a hull or mesh hash only matches C if the porter
hashes a serialized C layout. Box3D only checks the hash is nonzero and uses it
to find shared hulls; if an order there depends on hash values, serialize.

### Callbacks

Function type aliases with the C names; the `void* context` parameter is
dropped and the closure carries it (closures work under stub and frame).
`b3PlaneResultFcn` takes `(shapeId, planes: StaticArray<b3PlaneResult>, count)`.
Debug draw and task callback types only keep `b3WorldDef` compiling.

### Defaults

`b3DefaultWorldDef`, `b3DefaultBodyDef`, `b3DefaultShapeDef` return new class
instances; the other defaults return structs. The joint and explosion defaults
(C: `src/joint.c`) are in `lib/types.as`; `lib/joint.as` should re-export them,
not define them again.

### Pointer helpers

- `b3GetLengthAndNormalize(length: ref f32, a)`, `b3GetAxisAngle(radians: ref f32, q)`,
  `b3RayCastAABB(..., minFraction: ref f32, maxFraction: ref f32)`: rasc only
  binds a ref to a struct leaf or a FixedArray element, not to a plain f32
  local, so hold the output in a struct field or `let out: FixedArray<f32, 1>;`
  and pass `out[0]`.
- `b3EnlargeAABB(a: ref b3AABB, b)`.
- `b3MakeAABB(points: StaticArray<b3Vec3>, count, radius)`.
- `b3MakeQuatFromMatrix(m)`, `b3IsValidRay(input)`, `b3IsWithinSegments(result)`
  take values.
- `b3Store(dst: StaticArray<f32>, offset, src)`.
- `b3AlignUp8` takes and returns i32.

### Asserts and casts

- `B3_ASSERT(cond)` from core.as compiles out unless built with
  `-D B3_ENABLE_ASSERT`, but under `-O` rasc still evaluates `cond` and keeps
  the empty call (see rasc findings). In hot code write
  `if (B3_ASSERT_ENABLED) B3_ASSERT(cond);`, which folds away entirely.
  `B3_VALIDATE` does not exist; drop validation code per CONVENTIONS.
- `b3FloatToIntSat(f32): i32` and `b3FloatToUIntSat(f32): u32` are C's casts on
  the reference target: NaN to 0, out of range saturates (verified against C
  casts over the special sweep). Use them wherever C casts a float that could
  be NaN or out of range.
- `FLT_MAX`, `FLT_MIN`, `FLT_EPSILON`, `INT32_MAX`, `INT32_MIN`, `UINT8_MAX`,
  `UINT16_MAX`, `UINT32_MAX`, `UINT64_MAX` are in core.as.
- Constants that read the length units are `@inline` functions:
  `B3_HUGE()`, `B3_LINEAR_SLOP()`, `B3_MIN_CAPSULE_LENGTH()`,
  `B3_OVERLAP_SLOP()`, `B3_SPECULATIVE_DISTANCE()`, `B3_MESH_REST_OFFSET()`,
  `B3_CONTACT_RECYCLE_DISTANCE()`, `B3_MAX_AABB_MARGIN()`.

## Deviations and skipped items

- `b3QueryFilter` drops `name`: it only tags recordings, which are skipped,
  and dropping it keeps the filter a struct that copies like C.
- `b3BodyDef.name` and `b3ShapeDef.name` are strings, empty for none.
- `b3MeshDef.stride` is kept but unused; vertices are always contiguous.
- `b3MeshDef.materialIndices`, `b3HeightFieldDef.materialIndices`,
  `b3ShapeDef.materials` are nullable arrays (C NULL).
- Allocation hooks are accepted and ignored; `b3GetByteCount` returns 0;
  `b3Alloc`, `b3Free`, `b3GrowAlloc` and friends are not ported (the runtime
  owns memory). `b3StrCpy`, mutexes, semaphores and threads are skipped.
- The default assert handler returns 1 and the assert traps; the default log
  handler is silent (no host print). `b3Log` takes a ready string.
- `b3GetTicks`, `b3GetMilliseconds`, `b3GetMillisecondsAndReset` return 0;
  `b3Yield`, `b3Sleep` do nothing. `b3IsAVX2Available` is false.
- Skipped: debug draw (`b3HexColor`, `b3DebugMaterial`, `b3MakeDebugColor`,
  `b3DebugShape`, `b3DebugDraw`, `b3DefaultDebugDraw`, `b3GetGraphColor`),
  C++ operators, double precision paths, `B3_CHECK_DEF` (write
  `B3_ASSERT(def.internalValue == B3_SECRET_COOKIE)`), `B3_ARRAY_COUNT`,
  tracy macros, `b3AtomicFetch*` (the atomic structs exist, single threaded).
- Not mine, declared in these headers: `b3IntersectSegmentTriangle`,
  `b3IntersectSegmentSphere`, `b3ComputeMassProperties`, `b3IsValidMassData`,
  `b3GetProxySupport`, `b3GetPointSupport`, `b3IsSweepNormalized` and every non
  inline function of collision.h belong to the modules that define them in C.
  `b3ClosestPointOnTriangle`, `b3SphereInertia`, `b3CylinderInertia`,
  `b3BoxInertia` are in math_functions.as with their C definitions.
- `b3Triangle` lives in math_internal.as, where C declares it.

## rasc findings

### `-O` never inlines struct constructors or struct returning functions

```as
struct V { x: f32 = 0; y: f32 = 0; constructor(x: f32, y: f32) { this.x = x; this.y = y; } }
@inline function addI(a: V, b: V): V { return V(a.x + b.x, a.y + b.y); }
export function f(x: f32): f32 { return addI(V(x, x), V(1, 2)).y; }
// --wat -O: f calls V#constructor0 twice and addI once; @inline is ignored
```

Every `b3Vec3(...)` and every math helper stays a call under `-O`. Expect the
port to need `-O2` (wasm-opt, not installed here) for speed.

### A function whose body folds to nothing stays a call, and its argument is evaluated

```as
function A(c: bool): void { if (isDefined(XX)) { if (!c) unreachable(); } }
function pure(x: f32): bool { return x * x > 1; }
export function f(x: f32): f32 { A(pure(x)); return x; }
// --wat -O: f computes x * x > 1 (pure was inlined) and calls the empty A
```

Same with `@inline`. Workaround: `export const ON: bool = isDefined(XX);`
and `if (ON) A(pure(x));`, which folds to nothing. That is why
`B3_ASSERT_ENABLED` exists.

### Struct construction through a namespace import

```as
// a.as
export struct S { x: i32 = 0; }
export class C { y: i32 = 0; }
// b.as
import * as a from "./a";
export function f(): i32 {
  let c = new a.C();   // fine
  let t: a.S;          // fine
  let s = a.S();       // error: unknown function 'S' in namespace
  return c.y + t.x + s.x;
}
```

Workaround: import struct names directly.

### `ref` to a plain local

Already in notes/findings.md: `ref f32` binds a struct leaf or a FixedArray
element but not a primitive local, though the message lists locals.

### Smaller notes

- A type alias of a struct cannot be called as a constructor
  (`type P = V; P(1, 2)` is "unknown function 'P'").
- `T | null` is refused; nullable class types are spelled `T?`.
- StaticArray literals cannot hold structs (`[s]` for `StaticArray<S>`);
  allocate and assign by index.
- `rasc check` warns "both sides of this comparison are the same" on `a != a`;
  use `isNaN(a)`.
