# hull

## Ported

`src/hull.h` + `src/hull.c` -> `lib/hull.as`, every function in C order: the
quickhull builder, `b3CreateHull`, cylinders, cones, rocks, the complex hull,
bounds and bulk properties, edge cosines, the SOA sections, support vertex and
face, validation, clone, clone and transform, hash and compare, the hull map,
mass, AABB, swept AABB, overlap, ray cast, shape cast, mover collision, extent,
projected area, every box hull maker, `b3ScaleBox`, and the 2D hull and its
simplifier. hull.c has no debug draw and no file I/O, so nothing is skipped.

### The builder

C links vertices, half edges and faces through pointers into one work block,
with intrusive lists whose sentinels are embedded in the builder and in each
face. The port keeps the algorithm and every list operation, with indices for
pointers and `B3_NULL_INDEX` for NULL:

- `b3HullBuilder` is a class with one `StaticArray` per pool
  (`vertexBase`, `edgeBase`, `faceBase`, `horizon`, `cone`, `mergedFaces`,
  `horizonStack`) sized exactly as `b3ComputeHullWorkSizes` sizes the C pools.
- Every list node lives in `links`: vertex links at `[0, vertexCapacity)`,
  then the orphaned, vertex and face list sentinels, then one link per face
  slot (`faceLinkBase + face`), then one conflict list head per face slot
  (`conflictHeadBase + face`). C's `(b3QHFace*)node` cast is
  `node - faceLinkBase`.
- The edge and face free lists are LIFO through `edge.next` and the face
  link's `next`, exactly as in C, so every slot is reused in the same order and
  stale reads (if any) see what C sees.

Output order depends only on list order, and the port matches C bit for bit,
including on inputs where upstream crashes or hangs (below).

## Signatures that differ from C

| C | AnimaScript | Why |
|---|---|---|
| `b3CreateHull(const b3Vec3* points, int, int)` | `b3CreateHull(points: StaticArray<b3Vec3>, pointCount, maxVertexCount): b3HullData?` | Only the first `pointCount` are read. A C call with `&verts[offset]` copies the span into its own array first. |
| `b3CreateCylinder/Cone/Rock/ComplexHull` | return `b3HullData?` | C can return NULL. |
| `b3CloneHull(const b3HullData*)` | `b3CloneHull(hull: b3HullData?): b3HullData?` | Deep copy of every section. |
| `b3CloneAndTransformHull(const b3HullData*, ...)` | `(original: b3HullData?, transform, scale): b3HullData?` | |
| `b3MakeBoxHull` and the other box makers | return a new `b3BoxHull` | Pass `box.base` where C passes `&box.base`. |
| `b3ScaleBox(b3Vec3*, b3Transform*, ...)` | `(halfWidths: ref b3Vec3, transform: ref b3Transform, postScale, minHalfWidth)` | |
| `b3RayCastHull(shape, const b3RayCastInput*)` | `(shape, input: b3RayCastInput): b3CastOutput` | Small struct by value. |
| `b3ShapeCastHull(shape, const b3ShapeCastInput*)` | `(shape, input: b3ShapeCastInput)` | Class. |
| `b3OverlapHull(shape, xf, const b3ShapeProxy*)` | `(shape, shapeTransform, proxy: b3ShapeProxy): bool` | |
| `b3CollideMoverAndHull(b3PlaneResult*, shape, const b3Capsule*)` | `(result: ref b3PlaneResult, shape, mover: b3Capsule): i32` | Pass a `StaticArray` element, `planes[i]`. |
| `b3Hull2D(b3Point2D*, int, b3Point2D*)`, `b3SimplifyHull2D` | `StaticArray<b3Point2D>` for both arrays | `hull` needs room for `2 * count` points. |
| `b3HullMap_*` (verstable) | `b3HullMap` class, `b3HullMap_itr = i32` slot index | See below. |
| `b3DestroyHull` | no-op | The runtime owns memory. |

Unchanged: `b3FindHullSupportVertex(hull, direction): i32`,
`b3FindHullSupportFace(hull, direction): i32`,
`b3ComputeHullMass(shape, density): b3MassData`,
`b3ComputeHullAABB(shape, transform): b3AABB`,
`b3ComputeSweptHullAABB(shape, xf1, xf2): b3AABB`,
`b3ComputeHullExtent(hull, origin): b3ShapeExtent`,
`b3ComputeHullProjectedArea(hull, direction): f32`,
`b3IsValidHull(hull): bool`, `b3HashHullData(hull): u64`,
`b3CompareHullData(hull1, hull2): bool`, `b3GetHullSoaStride(count): i32`,
`B3_HULL_SOA_PADDING`.

Additions:

- `b3SerializeHullData(hull): Uint8Array` writes the C blob byte for byte (the
  152 byte header, each section at its offset, zero gaps). A `b3BoxHull` base
  serializes to the 696 byte `b3BoxHull`. The hash is `b3Hash64NonZero` over
  these bytes with `hash = 0`, as in C, and `b3CompareHullData` compares them
  as C's `memcmp` does. Compare therefore serializes both hulls; it only runs
  on hash hits in the hull map.
- `b3ValidateHull(hull)` is the `B3_ENABLE_VALIDATION` body of
  `b3IsValidHull`. The reference build compiles validation out, so
  `b3IsValidHull` returns true unless `B3_ENABLE_VALIDATION` (core.as) is true;
  `b3ValidateHull` lets the driver check the body against a C copy.
- Hull map: `b3HullMap_init`, `_reserve` (no-op), `_cleanup`, `_size`,
  `_bucket_count` (slot count, not verstable's), `_is_end(itr)`,
  `_get(map, key)`, `_insert(map, key, val)` (replaces key and value of a
  match, as verstable does), `_get_or_insert(map, key, val)`,
  `_key(map, itr)`, `_val(map, itr)`, `_setVal(map, itr, val)` for C's
  `itr.data->val`, `_first(map)`, `_next(map, itr)` (C takes only the
  iterator), `_erase_itr(map, itr)`, `_erase(map, key)`. Lookup is by hash,
  then content. Iteration runs in slot order, not verstable's bucket order, so
  anything that iterates the map (only the world byte counter in C) sees a
  different order. `b3HullMapByteCount` returns 0.

Box hulls: like C, `b3MakeTransformedBoxHull` leaves `edgeCosines` zero (the
static template never fills them). The private `b3HullBuilder_IsConsistent`
and `b3CheckConsistency` are ported behind `B3_DEBUG`, which is 0, so they are
compiled but not exercised.

## Verification

`verify/run.sh hull`: 226310 lines, 8347288 values, 0 mismatches. Also 0
mismatches with `RASC_FLAGS=` (no `-O`), under `--runtime stub`, and with
`-D B3_ENABLE_ASSERT` (with `-DHULL_NO_SPECIALS` on both drivers to drop the
NaN support directions, which break the support vertex precondition as they
would in a C debug build; no other assert fires).

Files: `verify/cref/hull.c`, `verify/as/hull.as`, `verify/hull.mjs`, and the
generated sample point sets `verify/cref/hull_data.h` / `verify/as/hull_data.as`
(45 sets, 2221 points as float bits: waterfall, quad and thin box from
`sample_issues.cpp` at 0.01 scale, `data/hulls/axe.txt` (1790 points) parsed
with `strtof` like the sample loader, the 37 metal wheel pieces, the two
ConvexJitter clouds with their swizzle, the `sample_geometry` Hull cloud and
the `sample_stacking` wedge; built by a throwaway C program that compiles the
sample literals as the samples do).

Each hull is compared field by field and element by element (header, vertices,
points, edges, planes, faces, both SOA sections including padding, edge
cosines), then as its raw C blob in 32 bit words, then validity, the C copy of
the validating `b3IsValidHull`, and the hash. Per hull: mass at three
densities, AABB and swept AABB under random transforms, support vertex and
face over 7 fixed, 40 random and 3 special directions, 44 ray casts
(random, zero length, from the centroid, from outside), 4 GJK overlaps, 4
shape casts, 4 mover collisions, 2 extents, 7 projected areas, then a clone
(compared with `b3CompareHullData`) and four clone and transforms (uniform,
non uniform, one axis mirrored, all mirrored with a near zero axis), each
printed and queried again.

Inputs: 45 sample sets at their own limit and at 4, 6, 12 and 1000; the
WheelStack combined hull; 720 seeded clouds in 12 families (uniform, sphere,
the HullReduction noisy box, nearly coplanar at 0 to 1e-3 thickness,
duplicates, nearly collinear, one repeated point, scales 1e-4 to 1e4, far from
the origin, transformed box corners, 0 to 6 points, lattices with and without
jitter) with mixed vertex limits; 60 each of cylinders (3 to 32 sides), cones
(4 to 32 slices), rocks and complex hulls; 60 box sets (box, cube, offset,
transformed, scaled with mirrored post scale, `b3ScaleBox`, axis aligned rays
for exact zero denominators); the hull map as the world database and as
compound baking, with duplicates, clones, boxes, erase and iteration; 400 2D
hulls with welds and simplification; dense sphere clouds over the face and
half edge limits (log messages are compared too); NULL clone inputs; 200
corrupted clones against the validating check. 920 hulls built, 284 of them
NULL, 3504 transformed clones.

### Upstream bug: near coplanar clouds crash or hang b3CreateHull

Six of the seeded clouds are skipped on both sides (`skip` lines), all
family 3 at thickness 1e-5: 2D points uniform in [-1, 1]^2 with z uniform in
[-1e-5, 1e-5] (random k = 3, 27, 267, 303, 627, 699; 78, 49, 86, 57, 15 and
36 points). The C reference segfaults in `b3NewellPlane` on two of them and
never returns on four. A debug build of hull.c asserts at hull.c:544
(`twin->twin == edge` in `b3NewellPlane`, called from `b3HullBuilder_ConnectEdges`
in `ConnectFaces` in `MergeCoplanar`) or at hull.c:1093 (the
`b3CheckConsistency` after `ConnectFaces`). In the hangs `ConnectEdges` merges a
face with itself (`twin->face == prev->face`) and then walks its half rebuilt
ring. The port does exactly the same: it traps in `b3NewellPlane` (index -1)
on the two crashers and loops on the other four. It is kept faithful; a guard
belongs upstream. Repro, in either language:

```c
vseed( 5000 + k );            // verify/cref/common.h
int n = vrandint( 4, 100 );
vrandint( 0, 5 );             // the family's eps pick, 3 here
for ( int i = 0; i < n; ++i ) { p[i].x = vrandf( -1, 1 ); p[i].y = vrandf( -1, 1 ); p[i].z = vrandf( -1e-5f, 1e-5f ); }
b3CreateHull( p, n, n );      // k = 3: crash, k = 267: hang
```

## rasc findings

### A nested ternary of float literals loses the f32 context

```as
export function f(p: i32): f32 {
	let a: f32 = p == 0 ? 0.0 : 1e-4;                // fine
	let b: f32 = p == 0 ? 0.0 : p == 1 ? 1e-7 : 1e-3; // error: cannot implicitly convert f64 to f32
	return a + b;
}
```

One level takes the annotation's type, two levels do not. Workaround: cast
each literal, `<f32>1e-7`.

No other compiler issue: the port compiles and matches with and without `-O`,
under frame and stub. `-g` gives function names in wasm stack traces, which is
how the assert trap above was located.
