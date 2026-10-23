// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/simd.c, src/simd.h, src/simd_scalar.h and src/simd_wide.h
// Dirk Gregorius contributed portions of this code

// The scalar 4 lane path, as C compiles it with BOX3D_DISABLE_SIMD.

import { B3_ASSERT } from "./core";
import { b3Vec3, b3AABB, b3AABB_Union } from "./math_functions";
import { b3TreeNode } from "./collision";

// simd_scalar.h

// scalar math
export struct b3FloatW4 {
  x: f32 = 0;
  y: f32 = 0;
  z: f32 = 0;
  w: f32 = 0;
  constructor(x: f32, y: f32, z: f32, w: f32) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }
}

@inline export function b3ZeroW4(): b3FloatW4 {
  return b3FloatW4(0.0, 0.0, 0.0, 0.0);
}

@inline export function b3SplatW4(scalar: f32): b3FloatW4 {
  return b3FloatW4(scalar, scalar, scalar, scalar);
}

@inline export function b3SetW4(a: f32, b: f32, c: f32, d: f32): b3FloatW4 {
  return b3FloatW4(a, b, c, d);
}

// C takes a float pointer; here the array plus the element index it points at
@inline export function b3LoadW4(data: StaticArray<f32>, index: i32): b3FloatW4 {
  return b3FloatW4(
    data[index],
    data[index + 1],
    data[index + 2],
    data[index + 3]
  );
}

@inline export function b3StoreW4(
  data: StaticArray<f32>,
  index: i32,
  a: b3FloatW4
): void {
  data[index] = a.x;
  data[index + 1] = a.y;
  data[index + 2] = a.z;
  data[index + 3] = a.w;
}

@inline export function b3NegW4(a: b3FloatW4): b3FloatW4 {
  return b3FloatW4(-a.x, -a.y, -a.z, -a.w);
}

@inline export function b3AbsW4(a: b3FloatW4): b3FloatW4 {
  return b3FloatW4(
    a.x < 0.0 ? -a.x : a.x,
    a.y < 0.0 ? -a.y : a.y,
    a.z < 0.0 ? -a.z : a.z,
    a.w < 0.0 ? -a.w : a.w
  );
}

@inline export function b3AddW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  return b3FloatW4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

@inline export function b3SubW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  return b3FloatW4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

@inline export function b3MulW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  return b3FloatW4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

@inline export function b3DivW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  return b3FloatW4(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
}

@inline export function b3SqrtW4(a: b3FloatW4): b3FloatW4 {
  return b3FloatW4(
    Mathf.sqrt(a.x),
    Mathf.sqrt(a.y),
    Mathf.sqrt(a.z),
    Mathf.sqrt(a.w)
  );
}

@inline export function b3MulAddW4(a: b3FloatW4, b: b3FloatW4, c: b3FloatW4): b3FloatW4 {
  return b3FloatW4(
    a.x + b.x * c.x,
    a.y + b.y * c.y,
    a.z + b.z * c.z,
    a.w + b.w * c.w
  );
}

@inline export function b3MinW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x <= b.x ? a.x : b.x;
  r.y = a.y <= b.y ? a.y : b.y;
  r.z = a.z <= b.z ? a.z : b.z;
  r.w = a.w <= b.w ? a.w : b.w;
  return r;
}

@inline export function b3MaxW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x >= b.x ? a.x : b.x;
  r.y = a.y >= b.y ? a.y : b.y;
  r.z = a.z >= b.z ? a.z : b.z;
  r.w = a.w >= b.w ? a.w : b.w;
  return r;
}

// clamp a to [-b, b]
@inline export function b3SymClampW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x <= b.x ? a.x : b.x;
  r.y = a.y <= b.y ? a.y : b.y;
  r.z = a.z <= b.z ? a.z : b.z;
  r.w = a.w <= b.w ? a.w : b.w;
  r.x = r.x <= -b.x ? -b.x : r.x;
  r.y = r.y <= -b.y ? -b.y : r.y;
  r.z = r.z <= -b.z ? -b.z : r.z;
  r.w = r.w <= -b.w ? -b.w : r.w;
  return r;
}

// Logical operations on the scalar path are 0/1 float values. Not bit-wise like SIMD.

@inline export function b3AndW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x != 0.0 && b.x != 0.0 ? 1.0 : 0.0;
  r.y = a.y != 0.0 && b.y != 0.0 ? 1.0 : 0.0;
  r.z = a.z != 0.0 && b.z != 0.0 ? 1.0 : 0.0;
  r.w = a.w != 0.0 && b.w != 0.0 ? 1.0 : 0.0;
  return r;
}

@inline export function b3OrW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x != 0.0 || b.x != 0.0 ? 1.0 : 0.0;
  r.y = a.y != 0.0 || b.y != 0.0 ? 1.0 : 0.0;
  r.z = a.z != 0.0 || b.z != 0.0 ? 1.0 : 0.0;
  r.w = a.w != 0.0 || b.w != 0.0 ? 1.0 : 0.0;
  return r;
}

// a & ~b
@inline export function b3AndNotW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x != 0.0 && b.x == 0.0 ? 1.0 : 0.0;
  r.y = a.y != 0.0 && b.y == 0.0 ? 1.0 : 0.0;
  r.z = a.z != 0.0 && b.z == 0.0 ? 1.0 : 0.0;
  r.w = a.w != 0.0 && b.w == 0.0 ? 1.0 : 0.0;
  return r;
}

@inline export function b3SoftMaskW4(
  indexA: FixedArray<i32,
  4>,
  indexB: FixedArray<i32,
  4>
): b3FloatW4 {
  let r = b3FloatW4();
  r.x = indexA[0] == 0 || indexB[0] == 0 ? 1.0 : 0.0;
  r.y = indexA[1] == 0 || indexB[1] == 0 ? 1.0 : 0.0;
  r.z = indexA[2] == 0 || indexB[2] == 0 ? 1.0 : 0.0;
  r.w = indexA[3] == 0 || indexB[3] == 0 ? 1.0 : 0.0;
  return r;
}

@inline export function b3GreaterThanW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x > b.x ? 1.0 : 0.0;
  r.y = a.y > b.y ? 1.0 : 0.0;
  r.z = a.z > b.z ? 1.0 : 0.0;
  r.w = a.w > b.w ? 1.0 : 0.0;
  return r;
}

@inline export function b3LessThanW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x < b.x ? 1.0 : 0.0;
  r.y = a.y < b.y ? 1.0 : 0.0;
  r.z = a.z < b.z ? 1.0 : 0.0;
  r.w = a.w < b.w ? 1.0 : 0.0;
  return r;
}

@inline export function b3EqualsW4(a: b3FloatW4, b: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = a.x == b.x ? 1.0 : 0.0;
  r.y = a.y == b.y ? 1.0 : 0.0;
  r.z = a.z == b.z ? 1.0 : 0.0;
  r.w = a.w == b.w ? 1.0 : 0.0;
  return r;
}

@inline export function b3AllZeroW4(a: b3FloatW4): bool {
  return a.x == 0.0 && a.y == 0.0 && a.z == 0.0 && a.w == 0.0;
}

@inline export function b3AnyTrueW4(mask: b3FloatW4): bool {
  return mask.x != 0.0 || mask.y != 0.0 || mask.z != 0.0 || mask.w != 0.0;
}

@inline export function b3MoveMaskW4(mask: b3FloatW4): i32 {
  return (mask.x != 0.0 ? 1 : 0) | (mask.y != 0.0 ? 2 : 0) | (
    mask.z != 0.0 ? 4 : 0
  ) | (mask.w != 0.0 ? 8 : 0);
}

// component-wise returns mask ? b : a
@inline export function b3BlendW4(a: b3FloatW4, b: b3FloatW4, mask: b3FloatW4): b3FloatW4 {
  let r = b3FloatW4();
  r.x = mask.x != 0.0 ? b.x : a.x;
  r.y = mask.y != 0.0 ? b.y : a.y;
  r.z = mask.z != 0.0 ? b.z : a.z;
  r.w = mask.w != 0.0 ? b.w : a.w;
  return r;
}

@inline function b3EmbedIndexLane(lane: f32, mask: u32, index: i32): f32 {
  let bits = reinterpret<u32>(lane);
  bits = (bits & ~mask) | <u32>index;
  return reinterpret<f32>(bits);
}

export function b3EmbedIndexW4(value: b3FloatW4, baseIndex: i32, bitCount: i32): b3FloatW4 {
  let mask: u32 = (1 << <u32>bitCount) - 1;
  return b3FloatW4(
    b3EmbedIndexLane(value.x, mask, baseIndex + 0),
    b3EmbedIndexLane(value.y, mask, baseIndex + 1),
    b3EmbedIndexLane(value.z, mask, baseIndex + 2),
    b3EmbedIndexLane(value.w, mask, baseIndex + 3)
  );
}

export function b3MinIndexW4(a: b3FloatW4, bitCount: i32): i32 {
  let m = a.x;
  m = a.y < m ? a.y : m;
  m = a.z < m ? a.z : m;
  m = a.w < m ? a.w : m;

  let bits = reinterpret<u32>(m);
  return <i32>(bits & ((1 << <u32>bitCount) - 1));
}

export type b3AABBV = b3AABB;

@inline export function b3LoadAABBV(aabb: b3AABB): b3AABBV {
  return aabb;
}

@inline export function b3OverlapAABBV(a: b3AABBV, b: b3AABBV): bool {
  return a.lowerBound.x <= b.upperBound.x &&
    a.lowerBound.y <= b.upperBound.y &&
    a.lowerBound.z <= b.upperBound.z &&
    b.lowerBound.x <= a.upperBound.x &&
    b.lowerBound.y <= a.upperBound.y &&
    b.lowerBound.z <= a.upperBound.z;
}

@inline export function b3OverlapNode(av: b3AABBV, node: b3TreeNode): bool {
  return b3OverlapAABBV(av, node.aabb);
}

@inline export function b3OverlapV(a: b3AABB, b: b3AABB): bool {
  return b3OverlapAABBV(a, b);
}

@inline export function b3UnionAABBV(a: b3AABBV, b: b3AABBV): b3AABBV {
  return b3AABB_Union(a, b);
}

// C takes a pointer to the pair; here the node array plus the index of the first node
@inline export function b3UnionPairV<A>(nodes: A, pair: i32): b3AABBV {
  return b3AABB_Union(nodes[pair].aabb, nodes[pair + 1].aabb);
}

@inline export function b3StoreAABBV(
  aabb: ref b3AABB,
  value: b3AABBV,
  condition: bool
): void {
  if (condition) {
    aabb = value;
  }
}

export function b3TransposeW4(
  r0: b3FloatW4,
  r1: b3FloatW4,
  r2: b3FloatW4,
  r3: b3FloatW4,
  c0: ref b3FloatW4,
  c1: ref b3FloatW4,
  c2: ref b3FloatW4,
  c3: ref b3FloatW4
): void {
  let t0 = b3FloatW4(r0.x, r2.x, r0.y, r2.y);
  let t1 = b3FloatW4(r1.x, r3.x, r1.y, r3.y);
  let t2 = b3FloatW4(r0.z, r2.z, r0.w, r2.w);
  let t3 = b3FloatW4(r1.z, r3.z, r1.w, r3.w);
  c0 = b3FloatW4(t0.x, t1.x, t0.y, t1.y);
  c1 = b3FloatW4(t0.z, t1.z, t0.w, t1.w);
  c2 = b3FloatW4(t2.x, t3.x, t2.y, t3.y);
  c3 = b3FloatW4(t2.z, t3.z, t2.w, t3.w);
}

// simd.h

// I don't expect the use case of b3V32 to benefit from Neon code.
// In particular the cross product is very complex in Neon.

export struct b3V32 {
  x: f32 = 0;
  y: f32 = 0;
  z: f32 = 0;
  constructor(x: f32, y: f32, z: f32) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
}

export const b3_zeroV = b3V32(0.0, 0.0, 0.0);
export const b3_halfV = b3V32(0.5, 0.5, 0.5);
export const b3_oneV = b3V32(1.0, 1.0, 1.0);

@inline export function b3AddV(a: b3V32, b: b3V32): b3V32 {
  return b3V32(a.x + b.x, a.y + b.y, a.z + b.z);
}

@inline export function b3SubV(a: b3V32, b: b3V32): b3V32 {
  return b3V32(a.x - b.x, a.y - b.y, a.z - b.z);
}

@inline export function b3MulV(a: b3V32, b: b3V32): b3V32 {
  return b3V32(a.x * b.x, a.y * b.y, a.z * b.z);
}

@inline export function b3DivV(a: b3V32, b: b3V32): b3V32 {
  return b3V32(a.x / b.x, a.y / b.y, a.z / b.z);
}

@inline export function b3NegV(a: b3V32): b3V32 {
  return b3V32(-a.x, -a.y, -a.z);
}

// Unaligned loads are much faster on recent hardware with little to no penalty
// C loads three floats from a pointer, every call site points at a b3Vec3
@inline export function b3LoadV(src: b3Vec3): b3V32 {
  return b3V32(src.x, src.y, src.z);
}

@inline export function b3ZeroV(): b3V32 {
  return b3V32(0.0, 0.0, 0.0);
}

@inline export function b3GetXV(a: b3V32): f32 {
  return a.x;
}

@inline export function b3GetYV(a: b3V32): f32 {
  return a.y;
}

@inline export function b3GetZV(a: b3V32): f32 {
  return a.z;
}

@inline export function b3GetV(a: b3V32, index: i32): f32 {
  return index == 0 ? a.x : index == 1 ? a.y : a.z;
}

@inline export function b3SplatV(x: f32): b3V32 {
  return b3V32(x, x, x);
}

@inline export function b3AbsV(a: b3V32): b3V32 {
  return b3V32(
    a.x < 0.0 ? -a.x : a.x,
    a.y < 0.0 ? -a.y : a.y,
    a.z < 0.0 ? -a.z : a.z
  );
}

@inline export function b3MinV(a: b3V32, b: b3V32): b3V32 {
  return b3V32(
    a.x < b.x ? a.x : b.x,
    a.y < b.y ? a.y : b.y,
    a.z < b.z ? a.z : b.z
  );
}

@inline export function b3MaxV(a: b3V32, b: b3V32): b3V32 {
  return b3V32(
    a.x > b.x ? a.x : b.x,
    a.y > b.y ? a.y : b.y,
    a.z > b.z ? a.z : b.z
  );
}

@inline export function b3CrossV(a: b3V32, b: b3V32): b3V32 {
  let c = b3V32();
  c.x = a.y * b.z - a.z * b.y;
  c.y = a.z * b.x - a.x * b.z;
  c.z = a.x * b.y - a.y * b.x;
  return c;
}

@inline export function b3ModifiedCrossV(a: b3V32, b: b3V32): b3V32 {
  let c = b3V32();
  c.x = a.y * b.z + a.z * b.y;
  c.y = a.z * b.x + a.x * b.z;
  c.z = a.x * b.y + a.y * b.x;
  return c;
}

@inline export function b3AnyLess3V(a: b3V32, b: b3V32): bool {
  return a.x < b.x || a.y < b.y || a.z < b.z;
}

@inline export function b3AnyLessEq3V(a: b3V32, b: b3V32): bool {
  return a.x <= b.x || a.y <= b.y || a.z <= b.z;
}

@inline export function b3AnyGreater3V(a: b3V32, b: b3V32): bool {
  return a.x > b.x || a.y > b.y || a.z > b.z;
}

@inline export function b3AllLessEq3V(a: b3V32, b: b3V32): bool {
  return a.x <= b.x && a.y <= b.y && a.z <= b.z;
}

// Wide vec3
export struct b3Vec3W4 {
  X: b3FloatW4 = b3FloatW4();
  Y: b3FloatW4 = b3FloatW4();
  Z: b3FloatW4 = b3FloatW4();
  constructor(X: b3FloatW4, Y: b3FloatW4, Z: b3FloatW4) {
    this.X = X;
    this.Y = Y;
    this.Z = Z;
  }
}

@inline export function b3LoadVW4(
  x: StaticArray<f32>,
  xi: i32,
  y: StaticArray<f32>,
  yi: i32,
  z: StaticArray<f32>,
  zi: i32
): b3Vec3W4 {
  return b3Vec3W4(b3LoadW4(x, xi), b3LoadW4(y, yi), b3LoadW4(z, zi));
}

@inline export function b3StoreVW4(
  x: StaticArray<f32>,
  xi: i32,
  y: StaticArray<f32>,
  yi: i32,
  z: StaticArray<f32>,
  zi: i32,
  v: b3Vec3W4
): void {
  b3StoreW4(x, xi, v.X);
  b3StoreW4(y, yi, v.Y);
  b3StoreW4(z, zi, v.Z);
}

@inline export function b3SplatVW4(v: b3Vec3): b3Vec3W4 {
  return b3Vec3W4(b3SplatW4(v.x), b3SplatW4(v.y), b3SplatW4(v.z));
}

@inline export function b3NegVW4(a: b3Vec3W4): b3Vec3W4 {
  return b3Vec3W4(b3NegW4(a.X), b3NegW4(a.Y), b3NegW4(a.Z));
}

// s * a
@inline export function b3MulSVW4(s: b3FloatW4, a: b3Vec3W4): b3Vec3W4 {
  return b3Vec3W4(b3MulW4(s, a.X), b3MulW4(s, a.Y), b3MulW4(s, a.Z));
}

// a - s * b
@inline export function b3MulSubSVW4(a: b3Vec3W4, s: b3FloatW4, b: b3Vec3W4): b3Vec3W4 {
  return b3Vec3W4(
    b3SubW4(a.X, b3MulW4(s, b.X)),
    b3SubW4(a.Y, b3MulW4(s, b.Y)),
    b3SubW4(a.Z, b3MulW4(s, b.Z))
  );
}

// a + s * b
@inline export function b3MulAddSVW4(a: b3Vec3W4, s: b3FloatW4, b: b3Vec3W4): b3Vec3W4 {
  return b3Vec3W4(
    b3AddW4(a.X, b3MulW4(s, b.X)),
    b3AddW4(a.Y, b3MulW4(s, b.Y)),
    b3AddW4(a.Z, b3MulW4(s, b.Z))
  );
}

// a - b
@inline export function b3SubVW4(a: b3Vec3W4, b: b3Vec3W4): b3Vec3W4 {
  return b3Vec3W4(b3SubW4(a.X, b.X), b3SubW4(a.Y, b.Y), b3SubW4(a.Z, b.Z));
}

// a + b
@inline export function b3AddVW4(a: b3Vec3W4, b: b3Vec3W4): b3Vec3W4 {
  return b3Vec3W4(b3AddW4(a.X, b.X), b3AddW4(a.Y, b.Y), b3AddW4(a.Z, b.Z));
}

@inline export function b3DotW4(a: b3Vec3W4, b: b3Vec3W4): b3FloatW4 {
  return b3AddW4(
    b3AddW4(b3MulW4(a.X, b.X), b3MulW4(a.Y, b.Y)),
    b3MulW4(a.Z, b.Z)
  );
}

@inline export function b3CrossW4(a: b3Vec3W4, b: b3Vec3W4): b3Vec3W4 {
  let c = b3Vec3W4();
  c.X = b3SubW4(b3MulW4(a.Y, b.Z), b3MulW4(a.Z, b.Y));
  c.Y = b3SubW4(b3MulW4(a.Z, b.X), b3MulW4(a.X, b.Z));
  c.Z = b3SubW4(b3MulW4(a.X, b.Y), b3MulW4(a.Y, b.X));
  return c;
}

@inline export function b3TestBoundsOverlap(
  nodeMin1: b3V32,
  nodeMax1: b3V32,
  nodeMin2: b3V32,
  nodeMax2: b3V32
): bool {
  let separation = b3MaxV(
    b3SubV(nodeMin2, nodeMax1),
    b3SubV(nodeMin1, nodeMax2)
  );
  return b3AllLessEq3V(separation, b3_zeroV);
}

// Test a ray for edge separation with an AABB (Gino, p80).
@inline export function b3TestBoundsRayOverlap(
  nodeMin: b3V32,
  nodeMax: b3V32,
  rayStart: b3V32,
  rayDelta: b3V32
): bool {
  // Setup node
  let nodeCenter = b3MulV(b3_halfV, b3AddV(nodeMin, nodeMax));
  let nodeExtent = b3SubV(nodeMax, nodeCenter);

  // Setup ray
  rayStart = b3SubV(rayStart, nodeCenter);

  // SAT: Edge separation
  let edgeSeparation = b3SubV(
    b3AbsV(b3CrossV(rayDelta, rayStart)),
    b3ModifiedCrossV(b3AbsV(rayDelta), nodeExtent)
  );
  return b3AllLessEq3V(edgeSeparation, b3_zeroV);
}

@inline export function b3UnionV(a: b3AABB, b: b3AABB): b3AABB {
  let result = b3AABB();
  b3StoreAABBV(result, b3UnionAABBV(b3LoadAABBV(a), b3LoadAABBV(b)), true);
  return result;
}

// simd.c

let b3_simdWidth: i32 = 0;

function b3DetectSIMDWidth(): i32 {
  return 4;
}

export function b3GetSIMDWidth(): i32 {
  let width = b3_simdWidth;
  if (width == 0) {
    width = b3DetectSIMDWidth();
    b3_simdWidth = width;
  }

  return width;
}

export function b3SetSIMDWidth(width: i32): void {
  B3_ASSERT(width == 0 || width == 4 || width == 8);
  if (width == 8 && b3DetectSIMDWidth() != 8) {
    width = 4;
  }

  b3_simdWidth = width;
}

@inline function b3SplatXV(a: b3V32): b3V32 {
  return b3V32(a.x, a.x, a.x);
}

@inline function b3SplatYV(a: b3V32): b3V32 {
  return b3V32(a.y, a.y, a.y);
}

@inline function b3SplatZV(a: b3V32): b3V32 {
  return b3V32(a.z, a.z, a.z);
}

@inline function b3AnyGreaterEq3V(a: b3V32, b: b3V32): bool {
  return a.x >= b.x || a.y >= b.y || a.z >= b.z;
}

@inline function b3Dot3V(a: b3V32, b: b3V32): b3V32 {
  let d = a.x * b.x + a.y * b.y + a.z * b.z;
  return b3V32(d, d, d);
}

export function b3TestBoundsTriangleOverlap(
  nodeCenter: b3V32,
  nodeExtent: b3V32,
  vertex1: b3V32,
  vertex2: b3V32,
  vertex3: b3V32
): bool {
  let two = b3SplatV(2.0);

  // Setup triangle
  vertex1 = b3SubV(vertex1, nodeCenter);
  vertex2 = b3SubV(vertex2, nodeCenter);
  vertex3 = b3SubV(vertex3, nodeCenter);

  // Face separation
  let triangleMin = b3MinV(vertex1, b3MinV(vertex2, vertex3));
  let triangleMax = b3MaxV(vertex1, b3MaxV(vertex2, vertex3));

  let separation1 = b3SubV(triangleMin, nodeExtent);
  let separation2 = b3AddV(triangleMax, nodeExtent);

  let faceSeparation = b3MaxV(separation1, b3NegV(separation2));
  if (b3AnyGreater3V(faceSeparation, b3_zeroV)) {
    return false;
  }

  // SAT: Face separation
  let edge1 = b3SubV(vertex2, vertex1);
  let edge2 = b3SubV(vertex3, vertex2);
  let edge3 = b3SubV(vertex1, vertex3);

  let normal = b3CrossV(edge1, edge2);

  let triangleSeparation = b3SubV(
    b3AbsV(b3Dot3V(normal, vertex1)),
    b3Dot3V(b3AbsV(normal), nodeExtent)
  );
  if (b3AnyGreater3V(triangleSeparation, b3_zeroV)) {
    return false;
  }

  // SAT: Edge separation
  let edgeSeparation1 = b3SubV(
    b3SubV(
      b3AbsV(b3CrossV(edge1, b3AddV(vertex1, vertex3))),
      b3AbsV(b3CrossV(edge1, edge3))
    ),
    b3MulV(two, b3ModifiedCrossV(b3AbsV(edge1), nodeExtent))
  );
  if (b3AnyGreater3V(edgeSeparation1, b3_zeroV)) {
    return false;
  }

  let edgeSeparation2 = b3SubV(
    b3SubV(
      b3AbsV(b3CrossV(edge2, b3AddV(vertex1, vertex2))),
      b3AbsV(b3CrossV(edge2, edge1))
    ),
    b3MulV(two, b3ModifiedCrossV(b3AbsV(edge2), nodeExtent))
  );
  if (b3AnyGreater3V(edgeSeparation2, b3_zeroV)) {
    return false;
  }

  let edgeSeparation3 = b3SubV(
    b3SubV(
      b3AbsV(b3CrossV(edge3, b3AddV(vertex2, vertex3))),
      b3AbsV(b3CrossV(edge3, edge2))
    ),
    b3MulV(two, b3ModifiedCrossV(b3AbsV(edge3), nodeExtent))
  );
  if (b3AnyGreater3V(edgeSeparation3, b3_zeroV)) {
    return false;
  }

  return true;
}

export function b3IntersectRayTriangle(
  rayStart: b3V32,
  rayDelta: b3V32,
  vertex1: b3V32,
  vertex2: b3V32,
  vertex3: b3V32
): f32 {
  // Test if ray intersects this triangle sharing same calculations for each triangle
  {
    let edge1 = b3SubV(vertex3, vertex2);
    let edge2 = b3SubV(vertex1, vertex3);
    let edge3 = b3SubV(vertex2, vertex1);

    let midPoint1 = b3MulV(b3_halfV, b3AddV(vertex2, vertex3));
    let midPoint2 = b3MulV(b3_halfV, b3AddV(vertex3, vertex1));
    let midPoint3 = b3MulV(b3_halfV, b3AddV(vertex1, vertex2));

    let normal1 = b3CrossV(edge1, b3SubV(midPoint1, rayStart));
    let normal2 = b3CrossV(edge2, b3SubV(midPoint2, rayStart));
    let normal3 = b3CrossV(edge3, b3SubV(midPoint3, rayStart));

    // B3_TRANSPOSE3( normal1, normal2, normal3 )
    {
      let temp1 = normal1.y;
      let temp2 = normal1.z;
      let temp3 = normal2.z;

      normal1.y = normal2.x;
      normal1.z = normal3.x;
      normal2.z = normal3.y;

      normal2.x = temp1;
      normal3.x = temp2;
      normal3.y = temp3;
    }

    let rayDeltaX = b3SplatXV(rayDelta);
    let rayDeltaY = b3SplatYV(rayDelta);
    let rayDeltaZ = b3SplatZV(rayDelta);

    let volumes = b3AddV(
      b3AddV(b3MulV(normal1, rayDeltaX), b3MulV(normal2, rayDeltaY)),
      b3MulV(normal3, rayDeltaZ)
    );
    if (b3AnyLess3V(volumes, b3_zeroV)) {
      return 1.0;
    }
  }

  // Compute intersection with triangle plane
  let edge1 = b3SubV(vertex2, vertex1);
  let edge2 = b3SubV(vertex3, vertex1);
  let normal = b3CrossV(edge1, edge2);

  let denominator = b3Dot3V(normal, rayDelta);
  if (b3AnyGreaterEq3V(denominator, b3_zeroV)) {
    return 1.0;
  }

  let lambda = b3DivV(b3Dot3V(normal, b3SubV(vertex1, rayStart)), denominator);
  if (b3AnyLessEq3V(lambda, b3_zeroV)) {
    return 1.0;
  }

  lambda = b3MinV(lambda, b3_oneV);
  return b3GetXV(lambda);
}

// simd_wide.h: the generic W names, B3_WIDE( name ) at width 4

export const B3_SIMD_WIDTH: i32 = 4;
export const B3_WIDE_ALIGNMENT: i32 = 4 * B3_SIMD_WIDTH;

// Aliases rather than type aliases, so b3Vec3W( X, Y, Z ) constructs too
export {
  b3FloatW4 as b3FloatW,
  b3Vec3W4 as b3Vec3W,
  b3ZeroW4 as b3ZeroW,
  b3SplatW4 as b3SplatW,
  b3SetW4 as b3SetW,
  b3LoadW4 as b3LoadW,
  b3StoreW4 as b3StoreW,
  b3NegW4 as b3NegW,
  b3AbsW4 as b3AbsW,
  b3AddW4 as b3AddW,
  b3SubW4 as b3SubW,
  b3MulW4 as b3MulW,
  b3DivW4 as b3DivW,
  b3SqrtW4 as b3SqrtW,
  b3MulAddW4 as b3MulAddW,
  b3MinW4 as b3MinW,
  b3MaxW4 as b3MaxW,
  b3SymClampW4 as b3SymClampW,
  b3AndW4 as b3AndW,
  b3OrW4 as b3OrW,
  b3AndNotW4 as b3AndNotW,
  b3SoftMaskW4 as b3SoftMaskW,
  b3GreaterThanW4 as b3GreaterThanW,
  b3LessThanW4 as b3LessThanW,
  b3EqualsW4 as b3EqualsW,
  b3AllZeroW4 as b3AllZeroW,
  b3AnyTrueW4 as b3AnyTrueW,
  b3BlendW4 as b3BlendW,
  b3EmbedIndexW4 as b3EmbedIndexW,
  b3MinIndexW4 as b3MinIndexW,
  b3TransposeW4 as b3TransposeW,
  b3LoadVW4 as b3LoadVW,
  b3StoreVW4 as b3StoreVW,
  b3SplatVW4 as b3SplatVW,
  b3NegVW4 as b3NegVW,
  b3MulSVW4 as b3MulSVW,
  b3MulSubSVW4 as b3MulSubSVW,
  b3MulAddSVW4 as b3MulAddSVW,
  b3SubVW4 as b3SubVW,
  b3AddVW4 as b3AddVW,
  b3DotW4 as b3DotW,
  b3CrossW4 as b3CrossW,
};

// Wasm SIMD lanes. Under ASC_FEATURE_SIMD a b3FloatW4 held in registers is one v128 with lane i at x, y, z, w.
// v128 cannot appear in a declaration of the scalar artifact, so these are generic over T and instantiate only
// from code behind ASC_FEATURE_SIMD. Each matches its b3FloatW4 counterpart bit for bit (notes/containers.md).
// Masks are all ones lanes here, where the scalar path stores 1.0 or 0.0.

@inline export function b3AddV4<T>(a: T, b: T): T {
	return f32x4.add(a, b);
}

@inline export function b3SubV4<T>(a: T, b: T): T {
	return f32x4.sub(a, b);
}

@inline export function b3MulV4<T>(a: T, b: T): T {
	return f32x4.mul(a, b);
}

@inline export function b3DivV4<T>(a: T, b: T): T {
	return f32x4.div(a, b);
}

@inline export function b3SqrtV4<T>(a: T): T {
	return f32x4.sqrt(a);
}

@inline export function b3NegV4<T>(a: T): T {
	return f32x4.neg(a);
}

// a + b * c
@inline export function b3MulAddV4<T>(a: T, b: T, c: T): T {
	return f32x4.add(a, f32x4.mul(b, c));
}

// a - b * c
@inline export function b3MulSubV4<T>(a: T, b: T, c: T): T {
	return f32x4.sub(a, f32x4.mul(b, c));
}

@inline export function b3MinV4<T>(a: T, b: T): T {
	return v128.bitselect(a, b, f32x4.le(a, b));
}

@inline export function b3MaxV4<T>(a: T, b: T): T {
	return v128.bitselect(a, b, f32x4.ge(a, b));
}

@inline export function b3AbsV4<T>(a: T): T {
	return v128.bitselect(f32x4.neg(a), a, f32x4.lt(a, f32x4.splat(0.0)));
}

// mask ? b : a
@inline export function b3BlendV4<T>(a: T, b: T, mask: T): T {
	return v128.bitselect(b, a, mask);
}

// A stored scalar mask (1.0 or 0.0) as lane bits, true where the scalar path tests != 0
@inline export function b3MaskV4<T>(a: T): T {
	return f32x4.ne(a, f32x4.splat(0.0));
}

@inline export function b3AllZeroV4<T>(a: T): bool {
	return i32x4.all_true(f32x4.eq(a, f32x4.splat(0.0)));
}

// (aX bX + aY bY) + aZ bZ
@inline export function b3DotV4<T>(aX: T, aY: T, aZ: T, bX: T, bY: T, bZ: T): T {
	return f32x4.add(f32x4.add(f32x4.mul(aX, bX), f32x4.mul(aY, bY)), f32x4.mul(aZ, bZ));
}

// One component of b3CrossW4: X is (aY, aZ, bY, bZ), Y is (aZ, aX, bZ, bX), Z is (aX, aY, bX, bY)
@inline export function b3CrossV4<T>(a1: T, a2: T, b1: T, b2: T): T {
	return f32x4.sub(f32x4.mul(a1, b2), f32x4.mul(a2, b1));
}

// One row of a symmetric matrix times a vector, m0 x + (m1 y + m2 z)
@inline export function b3MulRowV4<T>(m0: T, m1: T, m2: T, x: T, y: T, z: T): T {
	return f32x4.add(f32x4.mul(m0, x), f32x4.add(f32x4.mul(m1, y), f32x4.mul(m2, z)));
}

// Lane k of each row, one column of b3TransposeW4
@inline export function b3Transpose0V4<T>(r0: T, r1: T, r2: T, r3: T): T {
	return v128.shuffle<f32>(v128.shuffle<f32>(r0, r1, 0, 4, 0, 4), v128.shuffle<f32>(r2, r3, 0, 4, 0, 4), 0, 1, 4, 5);
}

@inline export function b3Transpose1V4<T>(r0: T, r1: T, r2: T, r3: T): T {
	return v128.shuffle<f32>(v128.shuffle<f32>(r0, r1, 1, 5, 1, 5), v128.shuffle<f32>(r2, r3, 1, 5, 1, 5), 0, 1, 4, 5);
}

@inline export function b3Transpose2V4<T>(r0: T, r1: T, r2: T, r3: T): T {
	return v128.shuffle<f32>(v128.shuffle<f32>(r0, r1, 2, 6, 2, 6), v128.shuffle<f32>(r2, r3, 2, 6, 2, 6), 0, 1, 4, 5);
}

@inline export function b3Transpose3V4<T>(r0: T, r1: T, r2: T, r3: T): T {
	return v128.shuffle<f32>(v128.shuffle<f32>(r0, r1, 3, 7, 3, 7), v128.shuffle<f32>(r2, r3, 3, 7, 3, 7), 0, 1, 4, 5);
}

@inline export function b3SetV4<T>(a: f32, b: f32, c: f32, d: f32): T {
	return f32x4.replace_lane(f32x4.replace_lane(f32x4.replace_lane(f32x4.splat(a), 1, b), 2, c), 3, d);
}

// ( (const float*)&w )[lane]
@inline export function b3LaneV4<T>(a: T, lane: i32): f32 {
	if (lane == 0) {
		return f32x4.extract_lane(a, 0);
	}
	if (lane == 1) {
		return f32x4.extract_lane(a, 1);
	}
	if (lane == 2) {
		return f32x4.extract_lane(a, 2);
	}
	return f32x4.extract_lane(a, 3);
}
