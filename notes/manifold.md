# manifold: contact manifolds for convex shapes and triangles

## Ported

| C | AnimaScript |
|---|---|
| `src/manifold.c` | `lib/manifold.as`, below the `// functions from manifold.c follow` marker |
| `src/convex_manifold.c`, `src/convex_manifold_wide.inl` at width 4 | `lib/convex_manifold.as` |
| `src/triangle_manifold.c` | `lib/triangle_manifold.as` |

Every function, in C order, statics unexported:

- manifold.c: `b3FindIncidentFace`, `b3MakeFeaturePair`, `b3FlipPair`, `b3ClipPolygon`.
- convex_manifold.c: `b3ClipSegment`, `b3ClipSegmentToHullFace`, the hull and
  capsule face and edge queries, `b3CollideSpheres`, `b3CollideCapsuleAndSphere`,
  `b3CollideHullAndSphere`, `b3CollideCapsules`, the hull and capsule face and
  edge contact builders, `b3CollideHullAndCapsule`, the 4 point
  `b3ReduceManifoldPoints`, `b3BuildPolygon`, `b3BuildFaceAContact`,
  `b3BuildFaceBContact`, `b3BuildEdgeContact`, `b3ComputeSeparatingAxisAtWidth`,
  `b3ComputeSeparatingAxis`, `b3CollideHullsAtWidth` with every cache case,
  including the manual testing ones, and `b3CollideHulls`.
- convex_manifold_wide.inl: `b3RoundUpToSIMDWidth`, `b3NegativeTransformFromSoA`,
  `b3GetSupportWide`, `b3GetFaceSeparation`, `b3GetFaceDots`,
  `b3TestEdgeCandidateSorted`, `b3GetFacePlaneSeparations`,
  `b3FilterEdgeCandidates`, `b3ComputeSeparatingAxisW4` with both edge pair
  loops: the `B3_SIMD_NONE` scalar loop (what the scalar reference runs, the
  default) and the lane loop on `lib/simd.as` (`-D B3_SAT_WIDE_EDGES`).
- triangle_manifold.c: `b3TriangleData`, the feature table,
  `b3GetTriangleFeature`, `b3CollideTriangleAndSphere`, the capsule clip, queries
  and contact builders, `b3CollideTriangleAndCapsule`, `b3GetTriangleSupport`,
  the hull face, triangle face and edge queries, `b3CollideHullFace`,
  `b3CollideTriangleFace`, `b3CollideTriangleAndHullEdges`,
  `b3CollideTriangleAndHull` with every cache case, and the
  `b3_triangleConvexCalls` / `b3_triangleCacheHits` globals (unused in C too).

## Signatures the contact code calls

```as
// lib/convex_manifold.as
export function b3CollideSpheres(manifold: b3LocalManifold, capacity: i32, sphereA: b3Sphere, sphereB: b3Sphere,
                                 transformBtoA: b3Transform): void
export function b3CollideCapsuleAndSphere(manifold: b3LocalManifold, capacity: i32, capsuleA: b3Capsule, sphereB: b3Sphere,
                                          transformBtoA: b3Transform): void
export function b3CollideHullAndSphere(manifold: b3LocalManifold, capacity: i32, hullA: b3HullData, sphereB: b3Sphere,
                                       transformBtoA: b3Transform, cache: ref b3SimplexCache): void
export function b3CollideCapsules(manifold: b3LocalManifold, capacity: i32, capsuleA: b3Capsule, capsuleB: b3Capsule,
                                  transformBtoA: b3Transform): void
export function b3CollideHullAndCapsule(manifold: b3LocalManifold, capacity: i32, hullA: b3HullData, capsuleB: b3Capsule,
                                        transformBtoA: b3Transform, cache: ref b3SimplexCache): void
export function b3CollideHulls(manifold: b3LocalManifold, capacity: i32, hullA: b3HullData, hullB: b3HullData,
                               transformBtoA: b3Transform, cache: ref b3SATCache): void
export function b3CollideHullsAtWidth(manifold: b3LocalManifold, capacity: i32, hullA: b3HullData, hullB: b3HullData,
                                      transformBtoA: b3Transform, cache: ref b3SATCache, simdWidth: i32): void
export function b3ComputeSeparatingAxis(hullA: b3HullData, hullB: b3HullData, xfB: b3Transform, earlyReturn: bool): b3AxisQuery
export function b3ComputeSeparatingAxisAtWidth(hullA, hullB, xfB, earlyReturn, simdWidth: i32): b3AxisQuery
export function b3ComputeSeparatingAxisW4(hullA, hullB, xfB, earlyReturn): b3AxisQuery

// lib/triangle_manifold.as
export function b3CollideTriangleAndSphere(manifold: b3LocalManifold, capacity: i32, triangleA: StaticArray<b3Vec3>,
                                           sphereB: b3Sphere): void
export function b3CollideTriangleAndCapsule(manifold: b3LocalManifold, capacity: i32, triangleA: StaticArray<b3Vec3>,
                                            capsuleB: b3Capsule, cache: ref b3SimplexCache): void
export function b3CollideTriangleAndHull(manifold: b3LocalManifold, capacity: i32, v1: b3Vec3, v2: b3Vec3, v3: b3Vec3,
                                         triangleFlags: i32, hullB: b3HullData, cache: ref b3SATCache,
                                         enableSpeculative: bool): void

// lib/manifold.as
export function b3FindIncidentFace(hull: b3HullData, refNormal: b3Vec3, vertexIndex: i32): i32
export function b3MakeFeaturePair(owner1: b3FeatureOwner, index1: i32, owner2: b3FeatureOwner, index2: i32): b3FeaturePair
export function b3FlipPair(pair: b3FeaturePair): b3FeaturePair
export function b3ClipPolygon(out: StaticArray<b3ClipVertex>, polygon: StaticArray<b3ClipVertex>, count: i32,
                              clipPlane: b3Plane, edge: i32, refPlane: b3Plane): i32
```

Caches: `b3SimplexCache` (collision.as, sphere and capsule against hull or
triangle) and `b3SATCache` (collision.as, hull against hull or triangle), both
structs passed as `ref`. In `b3ContactCache` that is `cache.simplexCache` /
`cache.satCache` of `contact.convexContact` (a class field, so it binds), and
for meshes copy the `b3TriangleCache` out of `meshContact.triangleCache` (a
growable Array), call, and store it back.

The manifold is the collision.as `b3LocalManifold` class: `points` is the point
buffer and C's `manifold->points + i` is `points[pointOffset + i]`, so the mesh
code points each manifold at its slice by setting `pointOffset`. A triangle is
a `StaticArray<b3Vec3>` of 3 (mesh_contact.c passes its `vertices[3]` local).

## Differences from C and why

- `const b3Sphere*` / `const b3Capsule*` are values, caches are `ref`, hulls
  are the class. `*cache = (b3SATCache){ 0 }` is `cache = b3SATCache()`.
- C stack buffers are module statics: the clip buffers, the clip point list,
  the segment pair, the capsule point pair, the SAT scratch (`dotA`, `planeDot*`,
  edge index lists, the `bFN*`, `bW*`, `aN*`, `aD*`, `aV0*`, `aTol` streams and
  the lane reduction arrays), the distance inputs, the edge manifold. Nothing
  re-enters while one is live; two lanes need two module instances.
- C's convex clip buffers are `B3_MAX_CLIP_POINTS` (64); the port's are 128.
  Upstream overflows them (see Upstream issues); with the larger buffers the
  port truncates to 64 points as C intends instead of trapping. Where C does
  not overflow the results are identical.
- Pointer and out parameter helpers: `b3ClipSegment` and friends take a
  `StaticArray<b3ClipVertex>` of 2; `b3NegativeTransformFromSoA` takes the SoA
  array plus three offsets where C takes three pointers; `b3GetSupportWide` and
  `b3GetFaceSeparation` return `b3WideSupport { value, vertexIndex }` instead of
  writing `float*` / `int*`; `b3GetFaceDots`, `b3GetFacePlaneSeparations`,
  `b3FilterEdgeCandidates` take arrays. `b3TriangleData` is passed by value.
- `b3CollideHullsAtWidth`'s `*manifold = edgeManifold` (keeping the point
  buffer) copies every scalar field through `b3CopyLocalManifoldFields`, so the
  zeroed `triangleNormal`, `feature`, and so on also land as in C.
- The width 8 dispatch is gone: `b3ComputeSeparatingAxisAtWidth` always runs W4,
  as the scalar build does (`B3_SIMD_HAS_WIDTH_8` is undefined there).
- `B3_VALIDATE` lines are dropped; asserts are `if (B3_ASSERT_ENABLED) B3_ASSERT(...)`.
- Edge pair loop selection: C picks the scalar loop with `B3_SIMD_NONE`; the port
  has the same choice as the module constant `B3_SIMD_NONE`, true unless built
  with `-D B3_SAT_WIDE_EDGES`.

### Shared file

`lib/manifold.as` (from the decls task): the declarations are unchanged. I added
names to its two import lines at the top (core, math_functions, math_internal
and the hull accessors from collision), since the appended functions need them.

## Skipped

- `b3ValidatePolygon` (only under `B3_ENABLE_VALIDATION`, off) and every
  `B3_VALIDATE`.
- The `B3_MAX_MANIFOLD_POINTS != 4` reduction through `b3Hull2D` /
  `b3SimplifyHull2D`; C does not compile it with the constant at 4.
- `b3ComputeSeparatingAxisW8` and all width 8 code.
- The discarded `triangleFlags` local (`B3_FORCE_GHOST_COLLISIONS`) in
  `b3QueryTriangleAndHullEdges`; C computes it and casts it to void.

## Upstream issues found

1. `b3BuildFaceAContact` overflows its 64 entry clip buffers. `b3BuildPolygon`
   caps the incident polygon at 64 vertices, but each side plane clip of a
   convex polygon can add one. A 64 sided pyramid (`b3CreateHull` of a 64 point
   circle plus an apex; 256 half edges, the limit) resting its base on a
   `b3MakeBoxHull(3, 0.5, 3)` top face, offset so one box side plane cuts off
   exactly one base vertex (`xf.p = (1.0, 0.74, 0)`), gives ASan
   `stack-buffer-overflow` in `b3ClipPolygon` called from `b3BuildFaceAContact`
   (`convex_manifold.c:1091`). Built against `convex_manifold.c`, `manifold.c`
   and `wide4.c` with `-fsanitize=address`:
   ```c
   b3Vec3 pts[65];
   for ( int i = 0; i < 64; ++i )
   {
   	float a = 2.0f * 3.14159265f * i / 64.0f;
   	pts[i] = ( b3Vec3 ){ 2.0f * cosf( a ), -0.25f, 2.0f * sinf( a ) };
   }
   pts[64] = ( b3Vec3 ){ 0.0f, 1.5f, 0.0f };
   b3HullData* pyramid = b3CreateHull( pts, 65, 128 );
   b3BoxHull box = b3MakeBoxHull( 3.0f, 0.5f, 3.0f );
   b3LocalManifoldPoint buffer[8];
   b3LocalManifold m = { 0 };
   m.points = buffer;
   b3SATCache cache = { 0 };
   b3Transform xf = { { 1.0f, 0.74f, 0.0f }, b3Quat_identity };
   b3CollideHulls( &m, 4, &box.base, pyramid, xf, &cache ); // stack-buffer-overflow
   ```
2. `b3CollideHullFace` (triangle_manifold.c) has the same overflow with its 64
   entry buffers: a large triangle under that pyramid's base, with one triangle
   edge cutting off one base vertex, clips to 65 points (ASan at
   `triangle_manifold.c:726`): the same pyramid with
   `b3CollideTriangleAndHull( &m, 64, (1.9995, -0.26, -50), (-100, -0.26, 0),
   (1.9995, -0.26, 50), 0, pyramid, &cache, true )` and `cache.type =
   b3_manualFaceAxisB`, which a fresh cache reaches too. The triangle face
   path already uses `2 * B3_MAX_CLIP_POINTS` and is safe. Fix for both: size
   the buffers `2 * B3_MAX_CLIP_POINTS` or clamp the count after each clip.
3. `b3CollideSpheres` leaves `pointCount` unchanged when separated or at
   capacity 0, where every other collide function zeroes it first. Harmless,
   since contact.c zeroes its local manifold, but the port keeps it.
4. A zero length capsule in deep contact with a triangle gives `t = 0 / 0` in
   `b3QueryTriangleAndCapsuleEdges`, so `b3Lerp` asserts in a debug build and a
   release build returns a manifold with a zero normal. Shapes cannot hit this:
   `b3CreateCapsuleShape` turns capsules shorter than `B3_LINEAR_SLOP` into
   spheres. Direct callers can.

## Verification

`verify/run.sh manifold convex_manifold convex_manifold_wide triangle_manifold`:

| module | lines | values | mismatches |
|---|---|---|---|
| manifold | 259116 | 2655160 | 0 |
| convex_manifold | 964541 | 11484120 | 0 |
| convex_manifold_wide | 272303 | 3085264 | 0 |
| triangle_manifold | 1118916 | 14589660 | 0 |

Identical with `RASC_FLAGS=` (no `-O`). With `RASC_FLAGS="-O -D B3_ENABLE_ASSERT"`
the first three pass; triangle_manifold traps in `b3Lerp`'s alpha assert on the
first point capsule in deep contact, which is upstream issue 4 (C asserts there
too). With the driver's point capsules lengthened to 0.01 it passes all
1118916 lines with asserts on. `nanEqual` is not needed.

Shared driver pieces: `verify/cref/manifold_common.h` and
`verify/as/manifold_common.as` (sentinel filled manifold and point buffer, the
printers, the hull library, pose generators). Every call prints the manifold
fields (point count, normal, triangle normal, triangle index, `i1..i3`, squared
distance, feature, flags) and every buffer slot below `offset + capacity`,
including slots the call must not touch (all start as distinct sentinels and
the manifold sometimes starts at `pointOffset` 1 to 4), each point with
position, separation, the 4 feature pair bytes, `b3MakeFeatureId` and the
triangle index; then the cache (SAT: separation, type, indexA, indexB, hit;
simplex: metric, count, all 4 index slots each side).

The hull library (identical on both sides, every hull printed in full first:
counts, bounds, mass, points, half edges with cosines, planes, faces, both SoA
streams): `b3MakeCubeHull`, two `b3MakeBoxHull` (flat, thin), offset,
transformed and scaled boxes, two cylinders, two cones, `b3CreateRock`,
`b3CreateComplexHull`, the sample point sets 0, 2, 4, 41 to 44 (sets 1 and 3
fail to build in both, as their logs say), a rock and a sample hull through
`b3CloneAndTransformHull` with non uniform scale, and three random clouds. 24
hulls.

- manifold: every owner and index combination of `b3MakeFeaturePair`,
  `b3FlipPair` and its inverse, `b3MakeFeatureId`; incident faces for every
  vertex of every hull against the 6 axes and 34 random normals; 20000
  `b3ClipPolygon` calls on 3 to 127 vertex polygons with random pairs, planes
  through a vertex, coplanar planes (noise on both sides) and random cuts.
- convex_manifold: 6000 sphere pairs, 6000 capsule and sphere, 12000 capsule
  pairs (parallel within the clip tolerance, coincident, zero and sub slop
  lengths, zero radius spheres), 4000 hull and sphere and 6000 hull and capsule
  sequences of 6 calls each with the simplex cache fed back, 4500 hull pairs of
  6 calls each with the SAT cache fed back (a repeated pose, small motions,
  jumps, a manual feature 1 time in 8, capacities 0 to 8, widths 4 and 8), plus
  the direct `b3ComputeSeparatingAxis` with early return on and off, and 960
  slide steps of box pairs through touch, face to face and edge to edge.
  The poses: near touch, deep, separated, axis aligned faces with lateral
  offset, crossed 45 degree edges, coincident, exact touch, and 1e3 away. Hull
  phases repeat at 0.37 length units. Results seen: SAT separated by face A
  2831 times, face B 531, edge pair 132, overlapping 1006; hull manifolds with
  1, 3 and 4 points; cache hits on 13876 of 27000 hull calls.
- convex_manifold_wide: the hull phases built with `-D B3_SAT_WIDE_EDGES`
  against a C reference that compiles `convex_manifold.c` and
  `convex_manifold_wide.inl` into the driver with `B3_SIMD_NONE` undefined for
  the inl (`verify/cref/convex_manifold_wide.c`), so the lane loop over
  `lib/simd.as` is checked bit for bit against C's lane loop through
  `simd_scalar.h`. The C side also reports on stderr how the two C loops
  compare: 0 of 9000 queries and 0 of 27000 hull collisions differ.
- triangle_manifold: 20000 triangle and sphere, 9000 capsule sequences of 6
  calls (lying, standing, across an edge, parallel to an edge, random, point
  capsules) with the simplex cache fed back, 11000 hull sequences of 6 calls
  with the SAT cache fed back, the triangle moved into the hull frame each step,
  every one of the 64 edge flag combinations cycled through, speculative on and
  off, the manual features, and hull centers just below the plane for the back
  side hysteresis. Triangles: random, near equilateral, slivers, large (10),
  tiny (0.05), axis aligned. Features seen: every one of the 9; hull
  manifolds of 1 to 8 points; cache types 0 to 4 and 6 to 8 on output.

Line coverage of the C sources over the same drivers (llvm-cov, out of band):
manifold.c 100%, convex_manifold_wide.inl 100%, convex_manifold.c 97%,
triangle_manifold.c 98%. Not reached, all guards that the callers or the
geometry rule out: `capacity < 1` in the hull capsule edge builder and
`capacity < 4` in the reduction (callers check first), the reduction's empty
first pick (the caller already returned when no point is within the
speculative distance), step 2 and 3 early outs of the reduction (5 or more clip
points within 4 linear slops of each other or on one line), the separated
returns of the deep paths (GJK just found overlap), the capsule pair divide
guard, the hull capsule and triangle capsule edge contacts falling outside the
segments in deep contact, the triangle capsule face contact both points above
the speculative distance, the default switch cases, and the no edge axis return
of the triangle capsule path (only NaN inputs reach it).

## Wasm SIMD later

The scalar build runs the 4 lane SAT through `lib/simd.as`, so the lanes map
straight onto `v128` (see the exact mappings in notes/containers.md):

- `b3GetSupportWide`, the hot loop of every face test: three `v128.load`s of
  the SoA x, y, z at `vx + i`, the dot as two `f32x4.mul` and two `f32x4.add`
  in C order, `f32x4.sub` from the bias, `b3EmbedIndexW` as integer and/or with
  `(i, i+1, i+2, i+3)`, `b3MinW` as `bitselect(a, b, le(a, b))`. The embedded
  index makes lane values distinct, but keep the sequential `b3MinIndexW`
  reduction for NaN.
- `b3GetFaceDots` (and so the plane separations and the seed picks) and
  `b3NegativeTransformFromSoA`: loads, three dots, add, `f32x4.neg`, stores.
- The edge pair loop: build with `-D B3_SAT_WIDE_EDGES` so it takes the lane
  loop (loads of the `aN*`, `aD*`, `aV0*`, `aTol` streams, compares, `and`,
  `any_true`, divide, sqrt, `bitselect` for the blend), keeping the scalar lane
  reduction. Today it equals the scalar loop on every vector above; the two
  differ only when `maxCD` is NaN (the scalar loop keeps the edge, the lane loop
  drops it), so a SIMD build should make the same choice C makes on that
  platform.
- `b3FindHullSupportVertex` (hull.as) is already 4 wide for the cached face
  tests.
- The scratch streams are bounds checked StaticArrays; switch them to
  `unchecked()` or raw `v128` loads once wasm SIMD lands.

## rasc findings

No compiler or std changes, and no new failures. Confirmed working, relied on:

- `export * from "./convex_manifold";` makes `verify/as/convex_manifold_wide.as`
  a one line re-export, built with its own `.flags` (`-D B3_SAT_WIDE_EDGES`), so
  one driver runs under two compile time configurations.
- Storing a whole struct through a `ref` parameter (`cache = b3SATCache();`)
  and through a `ref` binding to a StaticArray element
  (`ref pt = points[i]; pt = b3LocalManifoldPoint();`) write through.
- `let localCache = cache;` copies out of a `ref` parameter, as C's
  `b3SATCache localCache = *cache;`.
- A `FixedArray<b3Vec3, 3>` passed by value to an `@inline` function and
  indexed at runtime.
- `<i32>(a >= b)` for C's int valued comparisons in
  `b3TestEdgeCandidateSorted` is exact 0 or 1.

Seen again, already listed elsewhere: `-O` keeps struct constructors and
struct returning helpers as calls, and module constants stay global loads, so
the SAT needs the release bake's binaryen pass for speed.
