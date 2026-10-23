# distance: GJK, shape cast, time of impact, sphere and capsule

## Ported

| C | AnimaScript |
|---|---|
| `src/distance.c` | `lib/distance.as` |
| `src/sphere.c` | `lib/sphere.as` |
| `src/capsule.c` | `lib/capsule.as` |

Every function in the three files is ported, in C order, statics unexported:
support (`b3GetProxySupport`, `b3GetPointSupport`), the barycentric helpers,
`b3GetMetric`, `b3WriteCache`, `b3SolveSimplex2/3/4`, `b3ComputeWitnessPoints`,
`b3ShapeDistance`, `b3ShapeCast`, `b3GetSweepTransform`,
`b3GetFinalSweepTransform`, `b3UniqueCount`, `b3CheckFastEdges`, the
separation function (`b3MakeSeparationFunction`, `b3FindMinSeparation`,
`b3EvaluateSeparation`, `b3ForceFixedAxis`), `b3TimeOfImpact`; sphere and
capsule mass, AABB, swept AABB, overlap, ray casts (solid and hollow sphere),
shape casts and mover collision.

The segment helpers `b3LineDistance`, `b3SegmentDistance` and
`b3PointToSegmentDistance` are defined in `math_functions.c` and were already
ported to `lib/math_functions.as` by the types task; capsule.as uses them.

## Signatures other modules call

```as
// lib/distance.as
export function b3GetProxySupport(proxy: b3ShapeProxy, axis: b3Vec3): i32
export function b3GetPointSupport(points: StaticArray<b3Vec3>, count: i32, axis: b3Vec3): i32
export function b3ShapeDistance(input: b3DistanceInput, cache: ref b3SimplexCache,
                                simplexes: StaticArray<b3Simplex>? = null, simplexCapacity: i32 = 0): b3DistanceOutput
export function b3ShapeCast(input: b3ShapeCastPairInput): b3CastOutput
export function b3GetSweepTransform(sweep: b3Sweep, time: f32): b3Transform
export function b3TimeOfImpact(input: b3TOIInput): b3TOIOutput
export function b3MakeProxy(points: StaticArray<b3Vec3>, count: i32, radius: f32): b3ShapeProxy   // AS only
export function b3MakePointProxy(point: b3Vec3, radius: f32): b3ShapeProxy                        // AS only
export function b3MakeSegmentProxy(point1: b3Vec3, point2: b3Vec3, radius: f32): b3ShapeProxy     // AS only

// lib/sphere.as
export function b3ComputeSphereMass(shape: b3Sphere, density: f32): b3MassData
export function b3ComputeSphereAABB(shape: b3Sphere, transform: b3Transform): b3AABB
export function b3ComputeSweptSphereAABB(shape: b3Sphere, xf1: b3Transform, xf2: b3Transform): b3AABB
export function b3OverlapSphere(shape: b3Sphere, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool
export function b3RayCastSphere(shape: b3Sphere, input: b3RayCastInput): b3CastOutput
export function b3RayCastHollowSphere(sphere: b3Sphere, input: b3RayCastInput): b3CastOutput
export function b3ShapeCastSphere(sphere: b3Sphere, input: b3ShapeCastInput): b3CastOutput
export function b3CollideMoverAndSphere(result: ref b3PlaneResult, shape: b3Sphere, mover: b3Capsule): i32

// lib/capsule.as
export function b3ComputeCapsuleMass(shape: b3Capsule, density: f32): b3MassData
export function b3ComputeCapsuleAABB(shape: b3Capsule, transform: b3Transform): b3AABB
export function b3ComputeSweptCapsuleAABB(shape: b3Capsule, xf1: b3Transform, xf2: b3Transform): b3AABB
export function b3OverlapCapsule(shape: b3Capsule, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool
export function b3RayCastCapsule(shape: b3Capsule, input: b3RayCastInput): b3CastOutput
export function b3ShapeCastCapsule(capsule: b3Capsule, input: b3ShapeCastInput): b3CastOutput
export function b3CollideMoverAndCapsule(result: ref b3PlaneResult, shape: b3Capsule, mover: b3Capsule): i32
```

## Differences from C and why

- `const T*` inputs of small structs (`b3Sphere`, `b3Capsule`, `b3Sweep`,
  `b3RayCastInput`, `b3SimplexCache` in the separation function) are passed by
  value. Class inputs (`b3DistanceInput`, `b3ShapeCastPairInput`, `b3TOIInput`,
  `b3ShapeCastInput`, `b3ShapeProxy`) are passed as references.
- `b3SimplexCache* cache` is `ref b3SimplexCache`. A struct local, a struct
  field of a class, or a `StaticArray` element binds; an element of a growable
  `Array` does not (copy out, call, store back).
- `simplexes` is a nullable `StaticArray<b3Simplex>`, defaulting to null and
  capacity 0. Call sites written as C (`b3ShapeDistance(input, cache, null, 0)`)
  still read the same.
- `b3CollideMoverAndSphere` / `b3CollideMoverAndCapsule` write through
  `ref b3PlaneResult`; pass `planes[i]` of a `StaticArray`. As in C the result
  is only written when the return is 1.
- Static helpers whose only C output is an out array return it instead:
  `b3BarycentricCoordsEdge/Tri/Tet` return `FixedArray<f32, 3/4/5>`.
  `b3FindMinSeparation` keeps `ref i32` outputs; the caller holds them in a
  `FixedArray<i32, 2>`, because rasc refuses a ref to a primitive local.
  `b3ComputeWitnessPoints` writes `ref b3Vec3` locals, and the simplex helpers
  take `ref b3Simplex`.
- `b3SeparationFunction` is a private class, because it holds the two proxy
  references. TOI allocates one for each outer iteration, and `b3ShapeCast` /
  `b3TimeOfImpact` allocate one `b3DistanceInput` per call. C copies the
  proxies by value into these. Here they alias the caller's proxies, which
  nothing mutates.
- C's `(b3ShapeProxy){ &shape->center1, 2, r }` needs an array holding the
  points, so `b3MakePointProxy` and `b3MakeSegmentProxy` build one (sphere,
  capsule, and the height field and mesh ports use them). `b3MakeProxy` copies
  the first `count` points into an array the proxy owns. None of the three
  exist at the pin, where every site builds the proxy inline. C has no
  `b3MakeProxy` in box3d (box2d does). `b3MakeShapeProxy` and
  `b3MakeLocalProxy` belong to shape.c.
- The `b3Log` calls in `b3ShapeCast` (bad input dump) and `b3RayCastSphere`
  (fraction clamp) build their message with `toString()`, not `%.9f` / `%g`.
  The transform dump is split into three lines. The default log handler is
  silent, so results are unaffected.
- `B3_VALIDATE` lines and the `#if B3_ENABLE_VALIDATION && 0` block are
  dropped. Asserts are `if (B3_ASSERT_ENABLED) B3_ASSERT(...)`.

## Skipped

- `b3IsSweepNormalized` is declared in `math_internal.h` but never defined at
  the pin, so there is nothing to port.
- Nothing else is skipped in distance.c, sphere.c or capsule.c.

## Verification

`verify/run.sh distance sphere capsule`, using the shared helpers
`verify/cref/distance_common.h` and `verify/as/distance_common.as` (printers
and seeded test proxies):

| module | lines | values | mismatches |
|---|---|---|---|
| distance | 169269 | 7000119 | 0 |
| sphere | 28000 | 502380 | 0 |
| capsule | 39000 | 609910 | 0 |

Every line matches bit for bit under `-O`, with `RASC_FLAGS=` (no `-O`), with
`-O -D B3_ENABLE_ASSERT` (no assert fires), and under `--runtime stub`.
`nanEqual` is not needed.

- distance: 16 fixed exact-touch cases (coincident points, a point on a
  segment, crossing segments, a point inside a triangle, boxes touching face,
  edge and corner, gap, deep overlap), each with and without radii and run
  twice to warm start. 3000 support queries over random proxies and special
  float axes. 2000 sweep transforms. 8000 GJK pairs over 8 proxy kinds (point,
  segment, triangle, box, 4 to 32 point cloud, duplicate points, flat quad,
  collinear), each called 5 times with the cache fed back (same pose, two
  small steps, a jump). A quarter of rotations are identity, and poses
  include near, deep, tiny and far (offset by about 1e3). Debug simplexes are
  checked at capacity 40 and 3, and every printed simplex shows all 4 vertex
  slots. 9000 shape casts (aimed, random, overlapped start, near touch at 0.25
  to 5 linear slop, grazing, zero translation; max fractions of 1, random and
  tiny; with and without encroach). 9000 TOI (hit, miss, overlapped start,
  fast rotation in place, one step tunneling, segment spun around a triangle
  vertex). Casts and TOI are repeated at 0.37 length units. 20000 more TOI run
  in the stall mode. Every output field is printed: witness points, normal,
  distance, iterations, simplex count, the full cache (metric, count, all 4
  index slots), all cast output fields, and the TOI state, point, normal,
  fraction, distance and the three iteration counters. TOI states seen: 48
  failed, 1329 overlapped, 8606 hit, 19017 separated. Casts: 2430 hits and
  6570 misses.
- sphere: 1000 masses (zero radius, zero density), 2000 AABB and swept AABB,
  5000 overlaps, 10000 ray casts against both solid and hollow spheres (origin
  inside, near the surface, outside; zero length; max fraction 0), 5000 shape
  casts, and 5000 mover collisions (center on the mover axis, a zero length
  mover).
- capsule: 2000 masses (zero, 1e-5 and 1e-20 lengths, which reach the
  `1000 * FLT_MIN` branch), 2000 AABB and swept AABB, 5000 overlaps, 20000 ray
  casts (inside, inside the infinite cylinder past the ends, parallel to the
  axis for the `det < FLT_EPSILON` branch, endcaps, the short capsule sphere
  fallback), 5000 shape casts, and 5000 mover collisions (crossing cores,
  parallel, point mover).

Coverage of the C sources over these vectors (llvm-cov, out of band):
capsule.c 100% of lines, sphere.c 97%, distance.c 83%. In sphere.c only the
`b3RayCastSphere` fraction clamp, a rounding guard, is never hit. In
distance.c the paths never hit are:

- `b3SolveSimplex4` vertex regions A, B, C and edge regions AB, AC, BC.
  D is the newest support point, and GJK only adds it when the origin lies
  beyond the old face ABC, so these regions are geometrically unreachable.
  The `case 4` in the closest point switch and the 4 vertex metric are also
  unreachable, because a 4 vertex simplex returns before them and never
  reaches the cache.
- The `divisor <= 0` returns in the solvers, and `solved == false`. They need
  a degenerate cached simplex that still passes the metric check.
- In `b3ShapeDistance`, the "normal not normalized" overlap fallback, and in
  `b3ShapeCast`, the bad input log and the 20 iteration failure.
- In TOI: `b3ForceFixedAxis`, the 50 root iteration cap, the push back cap,
  and the edge-edge axis for a 3 point cache with near parallel edges (but
  the count 2 edge-edge case and the 25 iteration "progress too slow" failure
  are both covered).

### A C path with undefined results

If the first GJK iteration fails to progress (`distanceSq >= FLT_MAX`, for
example with inputs around 1e19 where `w·w` overflows), C restores `backup`,
whose count is 0. `b3ComputeWitnessPoints` then writes nothing, so
`pointA`/`pointB` are uninitialized locals. The scalar build happens to return
zeros, and the port returns zeros too (struct locals start zeroed), but C's
result there is undefined, so the vectors do not include it.

## rasc findings

No compiler or std changes.

1. A float literal in a nested ternary is not typed from the `f32` target,
   though a single ternary is:
   ```as
   export function one(m: i32, x: f32): f32 {
     let t: f32 = m == 0 ? 1.0 : x;               // fine
     return t;
   }
   export function two(m: i32, x: f32): f32 {
     let t: f32 = m == 0 ? 0.0 : m == 1 ? 1.0 : x; // error: cannot implicitly convert f64 to f32
     return t;
   }
   ```
   Workaround: cast the literals, `<f32>0.0`. This is the same family as
   containers finding 2 (constant float expressions are `f64`).
2. Confirmed, not new: `ref f32` to a plain local is refused, so every
   `b3GetLengthAndNormalize(&length, ...)` site holds the length in a
   `FixedArray<f32, 1>`, and the TOI support indices sit in a
   `FixedArray<i32, 2>`.
3. Useful, not a bug: a nullable `StaticArray<b3Simplex>?` parameter narrows
   inside `if (simplexes != null && ...)` and accepts element stores of a
   whole struct (`simplexes[i] = simplex`).
