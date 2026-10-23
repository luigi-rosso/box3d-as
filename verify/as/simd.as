// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/simd.as, mirrors verify/cref/simd.c

import {
  vtag,
  vi32,
  vf32,
  vbool,
  vend,
  vseed,
  vrand32,
  vrandint,
  vrandf,
  vspecialf
} from "./common";
import { b3Vec3, b3AABB } from "../../lib/math_functions";
import { b3TreeNode } from "../../lib/collision";
import {
  b3FloatW,
  b3Vec3W,
  b3FloatW4,
  b3Vec3W4,
  b3V32,
  b3_halfV,
  b3ZeroW,
  b3SplatW,
  b3SetW,
  b3LoadW,
  b3StoreW,
  b3NegW,
  b3AbsW,
  b3SqrtW,
  b3AddW,
  b3SubW,
  b3MulW,
  b3DivW,
  b3MinW,
  b3MaxW,
  b3SymClampW,
  b3AndW,
  b3OrW,
  b3AndNotW,
  b3GreaterThanW,
  b3LessThanW,
  b3EqualsW,
  b3MulAddW,
  b3AllZeroW,
  b3AnyTrueW,
  b3MoveMaskW4,
  b3BlendW,
  b3SoftMaskW,
  b3EmbedIndexW,
  b3MinIndexW,
  b3TransposeW,
  b3AABBV,
  b3LoadAABBV,
  b3StoreAABBV,
  b3UnionAABBV,
  b3OverlapAABBV,
  b3OverlapNode,
  b3OverlapV,
  b3UnionPairV,
  b3UnionV,
  b3AddV,
  b3SubV,
  b3MulV,
  b3DivV,
  b3NegV,
  b3AbsV,
  b3MinV,
  b3MaxV,
  b3CrossV,
  b3ModifiedCrossV,
  b3SplatV,
  b3ZeroV,
  b3GetXV,
  b3GetYV,
  b3GetZV,
  b3GetV,
  b3AnyLess3V,
  b3AnyLessEq3V,
  b3AnyGreater3V,
  b3AllLessEq3V,
  b3LoadV,
  b3SplatVW,
  b3NegVW,
  b3MulSVW,
  b3MulSubSVW,
  b3MulAddSVW,
  b3SubVW,
  b3AddVW,
  b3DotW,
  b3CrossW,
  b3LoadVW,
  b3StoreVW,
  b3TestBoundsOverlap,
  b3TestBoundsRayOverlap,
  b3TestBoundsTriangleOverlap,
  b3IntersectRayTriangle,
  b3GetSIMDWidth,
  b3SetSIMDWidth,
} from "../../lib/simd";

export { frameBoundary } from "./common";

const N = 3000;

function randW(): b3FloatW {
  let r = b3FloatW4();
  r.x = vspecialf();
  r.y = vspecialf();
  r.z = vspecialf();
  r.w = vspecialf();
  return r;
}

// Masks and logic inputs: mostly 0 and 1, sometimes any special.
function maskf(): f32 {
  let pick = vrand32() % 4;
  if (pick == 0) {
    return vspecialf();
  }
  return (pick & 1) != 0 ? 1.0 : 0.0;
}

function randMask(): b3FloatW {
  let r = b3FloatW4();
  r.x = maskf();
  r.y = maskf();
  r.z = maskf();
  r.w = maskf();
  return r;
}

// Geometry inputs: mostly ordinary values so tests go both ways, sometimes specials.
function geomf(): f32 {
  if (vrand32() % 8 == 0) {
    return vspecialf();
  }
  return vrandf(-10.0, 10.0);
}

function randV32(): b3V32 {
  let v = b3V32();
  v.x = geomf();
  v.y = geomf();
  v.z = geomf();
  return v;
}

function randVec3(): b3Vec3 {
  let v = b3Vec3();
  v.x = geomf();
  v.y = geomf();
  v.z = geomf();
  return v;
}

function randAABB(): b3AABB {
  let a = b3AABB();
  a.lowerBound = randVec3();
  let extent = b3Vec3();
  extent.x = vrandf(0.0, 8.0);
  extent.y = vrandf(0.0, 8.0);
  extent.z = vrandf(0.0, 8.0);
  a.upperBound.x = a.lowerBound.x + extent.x;
  a.upperBound.y = a.lowerBound.y + extent.y;
  a.upperBound.z = a.lowerBound.z + extent.z;
  return a;
}

function randVec3W(): b3Vec3W {
  let v = b3Vec3W4();
  v.X = randW();
  v.Y = randW();
  v.Z = randW();
  return v;
}

function emitW(a: b3FloatW): void {
  vf32(a.x);
  vf32(a.y);
  vf32(a.z);
  vf32(a.w);
}

function emitV32(a: b3V32): void {
  vf32(a.x);
  vf32(a.y);
  vf32(a.z);
}

function emitVec3(a: b3Vec3): void {
  vf32(a.x);
  vf32(a.y);
  vf32(a.z);
}

function emitAABB(a: b3AABB): void {
  emitVec3(a.lowerBound);
  emitVec3(a.upperBound);
}

function emitVec3W(a: b3Vec3W): void {
  emitW(a.X);
  emitW(a.Y);
  emitW(a.Z);
}

// Struct values cannot be function typed arguments, so the sweeps take an op number.
function unaryOp(op: i32, a: b3FloatW): b3FloatW {
  if (op == 0) return b3NegW(a);
  if (op == 1) return b3AbsW(a);
  return b3SqrtW(a);
}

function binaryOp(op: i32, a: b3FloatW, b: b3FloatW): b3FloatW {
  switch (op) {
    case 0:
      return b3AddW(a, b);
    case 1:
      return b3SubW(a, b);
    case 2:
      return b3MulW(a, b);
    case 3:
      return b3DivW(a, b);
    case 4:
      return b3MinW(a, b);
    case 5:
      return b3MaxW(a, b);
    case 6:
      return b3SymClampW(a, b);
    case 7:
      return b3AndW(a, b);
    case 8:
      return b3OrW(a, b);
    case 9:
      return b3AndNotW(a, b);
    case 10:
      return b3GreaterThanW(a, b);
    case 11:
      return b3LessThanW(a, b);
    default:
      return b3EqualsW(a, b);
  }
}

function sweepUnary(tag: string, op: i32): void {
  for (let i = 0; i < N; ++i) {
    let a = randW();
    vtag(tag);
    emitW(a);
    emitW(unaryOp(op, a));
    vend();
  }
}

function sweepBinary(tag: string, op: i32, masks: bool): void {
  for (let i = 0; i < N; ++i) {
    let a = masks ? randMask() : randW();
    let b = masks ? randMask() : randW();
    if (vrand32() % 8 == 0) {
      b = a;
    }
    vtag(tag);
    emitW(a);
    emitW(b);
    emitW(binaryOp(op, a, b));
    vend();
  }
}

export function run(): void {
  vseed(7);

  vtag("zero");
  emitW(b3ZeroW());
  vend();

  for (let i = 0; i < 64; ++i) {
    let s = vspecialf();
    vtag("splat");
    emitW(b3SplatW(s));
    vend();
    let a = vspecialf();
    let b = vspecialf();
    let c = vspecialf();
    let d = vspecialf();
    vtag("set");
    emitW(b3SetW(a, b, c, d));
    vend();
  }

  for (let i = 0; i < 200; ++i) {
    let data = new StaticArray<f32>(16);
    let out = new StaticArray<f32>(16);
    for (let k = 0; k < 16; ++k) {
      data[k] = vspecialf();
    }
    let index = vrandint(0, 13);
    let v = b3LoadW(data, index);
    b3StoreW(out, vrandint(0, 13), v);
    vtag("loadstore");
    vi32(index);
    emitW(v);
    for (let k = 0; k < 16; ++k) {
      vf32(out[k]);
    }
    vend();
  }

  sweepUnary("neg", 0);
  sweepUnary("abs", 1);
  sweepUnary("sqrt", 2);
  sweepBinary("add", 0, false);
  sweepBinary("sub", 1, false);
  sweepBinary("mul", 2, false);
  sweepBinary("div", 3, false);
  sweepBinary("min", 4, false);
  sweepBinary("max", 5, false);
  sweepBinary("symclamp", 6, false);
  sweepBinary("and", 7, true);
  sweepBinary("or", 8, true);
  sweepBinary("andnot", 9, true);
  sweepBinary("gt", 10, false);
  sweepBinary("lt", 11, false);
  sweepBinary("eq", 12, false);

  for (let i = 0; i < N; ++i) {
    let a = randW();
    let b = randW();
    let c = randW();
    vtag("muladd");
    emitW(a);
    emitW(b);
    emitW(c);
    emitW(b3MulAddW(a, b, c));
    vend();
  }

  for (let i = 0; i < N; ++i) {
    let m = randMask();
    let a = randW();
    let b = randW();
    vtag("mask");
    emitW(m);
    vbool(b3AllZeroW(m));
    vbool(b3AnyTrueW(m));
    vi32(b3MoveMaskW4(m));
    emitW(b3BlendW(a, b, m));
    vend();
  }

  for (let i = 0; i < N; ++i) {
    let indexA = FixedArray<i32, 4>();
    let indexB = FixedArray<i32, 4>();
    for (let k = 0; k < 4; ++k) {
      indexA[k] = vrandint(-1, 3);
      indexB[k] = vrandint(-1, 3);
    }
    vtag("softmask");
    for (let k = 0; k < 4; ++k) {
      vi32(indexA[k]);
      vi32(indexB[k]);
    }
    emitW(b3SoftMaskW(indexA, indexB));
    vend();
  }

  for (let i = 0; i < N; ++i) {
    let a = randW();
    let base = vrandint(0, 128);
    let bits = i % 2 == 0 ? 7 : vrandint(1, 24);
    let e = b3EmbedIndexW(a, base, bits);
    vtag("embed");
    emitW(a);
    vi32(base);
    vi32(bits);
    emitW(e);
    vi32(b3MinIndexW(e, bits));
    vi32(b3MinIndexW(a, bits));
    vend();
  }

  for (let i = 0; i < N / 4; ++i) {
    let r0 = randW();
    let r1 = randW();
    let r2 = randW();
    let r3 = randW();
    let c0 = b3FloatW4();
    let c1 = b3FloatW4();
    let c2 = b3FloatW4();
    let c3 = b3FloatW4();
    b3TransposeW(r0, r1, r2, r3, c0, c1, c2, c3);
    vtag("transpose");
    emitW(c0);
    emitW(c1);
    emitW(c2);
    emitW(c3);
    vend();

    // Aliased outputs, as the contact solver's pad lanes are
    let pad = b3ZeroW();
    b3TransposeW(r3, r2, r1, r0, c0, pad, c1, pad);
    vtag("transpose2");
    emitW(c0);
    emitW(c1);
    emitW(pad);
    vend();
  }

  let nodes = new StaticArray<b3TreeNode>(2);
  for (let i = 0; i < N; ++i) {
    let a = randAABB();
    let b = randAABB();
    if (vrand32() % 16 == 0) {
      a.upperBound.x = vspecialf();
    }
    nodes[0].aabb = a;
    nodes[1].aabb = b;
    let av: b3AABBV = b3LoadAABBV(a);
    let stored = randAABB();
    let condition = vrand32() % 2 == 0;
    b3StoreAABBV(stored, b3UnionAABBV(av, b3LoadAABBV(b)), condition);
    vtag("aabb");
    emitAABB(a);
    emitAABB(b);
    vbool(b3OverlapAABBV(av, b3LoadAABBV(b)));
    vbool(b3OverlapNode(av, nodes[1]));
    vbool(b3OverlapV(a, b));
    emitAABB(b3UnionPairV(nodes, 0));
    emitAABB(b3UnionV(a, b));
    vbool(condition);
    emitAABB(stored);
    vend();
  }

  for (let i = 0; i < N; ++i) {
    let a = randV32();
    let b = randV32();
    if (vrand32() % 8 == 0) {
      b = a;
    }
    let index = vrandint(0, 3);
    let s = geomf();
    vtag("v32");
    emitV32(a);
    emitV32(b);
    emitV32(b3AddV(a, b));
    emitV32(b3SubV(a, b));
    emitV32(b3MulV(a, b));
    emitV32(b3DivV(a, b));
    emitV32(b3NegV(a));
    emitV32(b3AbsV(a));
    emitV32(b3MinV(a, b));
    emitV32(b3MaxV(a, b));
    emitV32(b3CrossV(a, b));
    emitV32(b3ModifiedCrossV(a, b));
    emitV32(b3SplatV(s));
    emitV32(b3ZeroV());
    vf32(b3GetXV(a));
    vf32(b3GetYV(a));
    vf32(b3GetZV(a));
    vi32(index);
    vf32(b3GetV(a, index));
    vbool(b3AnyLess3V(a, b));
    vbool(b3AnyLessEq3V(a, b));
    vbool(b3AnyGreater3V(a, b));
    vbool(b3AllLessEq3V(a, b));
    vend();

    let p = randVec3();
    vtag("loadv");
    emitV32(b3LoadV(p));
    vend();
  }

  for (let i = 0; i < N; ++i) {
    let a = randVec3W();
    let b = randVec3W();
    let s = randW();
    let v = randVec3();
    vtag("vec3w");
    emitVec3W(a);
    emitVec3W(b);
    emitW(s);
    emitVec3W(b3SplatVW(v));
    emitVec3W(b3NegVW(a));
    emitVec3W(b3MulSVW(s, a));
    emitVec3W(b3MulSubSVW(a, s, b));
    emitVec3W(b3MulAddSVW(a, s, b));
    emitVec3W(b3SubVW(a, b));
    emitVec3W(b3AddVW(a, b));
    emitW(b3DotW(a, b));
    emitVec3W(b3CrossW(a, b));
    vend();
  }

  for (let i = 0; i < 200; ++i) {
    let x = new StaticArray<f32>(12);
    let y = new StaticArray<f32>(12);
    let z = new StaticArray<f32>(12);
    let ox = new StaticArray<f32>(12);
    let oy = new StaticArray<f32>(12);
    let oz = new StaticArray<f32>(12);
    for (let k = 0; k < 12; ++k) {
      x[k] = vspecialf();
      y[k] = vspecialf();
      z[k] = vspecialf();
    }
    let xi = vrandint(0, 9);
    let yi = vrandint(0, 9);
    let zi = vrandint(0, 9);
    let v = b3LoadVW(x, xi, y, yi, z, zi);
    b3StoreVW(ox, zi, oy, xi, oz, yi, v);
    vtag("loadstorev");
    emitVec3W(v);
    for (let k = 0; k < 12; ++k) {
      vf32(ox[k]);
      vf32(oy[k]);
      vf32(oz[k]);
    }
    vend();
  }

  for (let i = 0; i < 4 * N; ++i) {
    let min1 = randV32();
    let max1 = b3AddV(min1, b3SplatV(vrandf(0.0, 6.0)));
    let min2 = randV32();
    let max2 = b3AddV(min2, b3SplatV(vrandf(0.0, 6.0)));
    let start = randV32();
    let delta = randV32();
    let v1 = randV32();
    let v2 = randV32();
    let v3 = randV32();
    if (i % 3 == 0) {
      // Triangles near the box so the SAT axes all get exercised
      v1 = b3AddV(min1, b3MulV(b3_halfV, v1));
      v2 = b3AddV(min1, b3MulV(b3_halfV, v2));
      v3 = b3AddV(min1, b3MulV(b3_halfV, v3));
    }
    let center = b3MulV(b3_halfV, b3AddV(min1, max1));
    let extent = b3SubV(max1, center);
    vtag("bounds");
    emitV32(min1);
    emitV32(max1);
    emitV32(min2);
    emitV32(max2);
    emitV32(start);
    emitV32(delta);
    emitV32(v1);
    emitV32(v2);
    emitV32(v3);
    vbool(b3TestBoundsOverlap(min1, max1, min2, max2));
    vbool(b3TestBoundsRayOverlap(min1, max1, start, delta));
    vbool(b3TestBoundsTriangleOverlap(center, extent, v1, v2, v3));
    vf32(b3IntersectRayTriangle(start, delta, v1, v2, v3));
    vf32(b3IntersectRayTriangle(start, b3SubV(v1, start), v1, v2, v3));
    vf32(
      b3IntersectRayTriangle(
        start,
        b3SubV(b3MulV(b3_halfV, b3AddV(v2, v3)), start),
        v1,
        v2,
        v3
      )
    );
    let centroid = b3MulV(b3SplatV(0.333333), b3AddV(v1, b3AddV(v2, v3)));
    vf32(
      b3IntersectRayTriangle(
        start,
        b3MulV(b3SplatV(2.0), b3SubV(centroid, start)),
        v1,
        v2,
        v3
      )
    );
    vf32(
      b3IntersectRayTriangle(
        start,
        b3MulV(b3SplatV(2.0), b3SubV(centroid, start)),
        v1,
        v3,
        v2
      )
    );
    vend();
  }

  vtag("width");
  vi32(b3GetSIMDWidth());
  b3SetSIMDWidth(0);
  vi32(b3GetSIMDWidth());
  b3SetSIMDWidth(8);
  vi32(b3GetSIMDWidth());
  b3SetSIMDWidth(4);
  vi32(b3GetSIMDWidth());
  vend();
}
