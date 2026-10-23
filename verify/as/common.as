// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Vector output and seeded inputs for the AnimaScript drivers, the twin of
// verify/cref/common.h. Floats cross to the host as bit patterns so NaN
// payloads survive.

@external("vec", "ch") declare function vec_ch(c: i32): void;
@external("vec", "i32") declare function vec_i32(x: i32): void;
@external("vec", "u32") declare function vec_u32(x: u32): void;
@external("vec", "f32") declare function vec_f32(bits: u32): void;
@external("vec", "u64") declare function vec_u64(x: u64): void;
@external("vec", "end") declare function vec_end(): void;

export function vtag(s: string): void {
  for (let i = 0; i < s.length; ++i) {
    vec_ch(s.charCodeAt(i));
  }
  vec_ch(0);
}

@inline export function vi32(x: i32): void {
  vec_i32(x);
}

@inline export function vu32(x: u32): void {
  vec_u32(x);
}

@inline export function vbool(x: bool): void {
  vec_i32(x ? 1 : 0);
}

@inline export function vf32(x: f32): void {
  vec_f32(reinterpret<u32>(x));
}

@inline export function vu64(x: u64): void {
  vec_u64(x);
}

@inline export function vf64(x: f64): void {
  vec_u64(reinterpret<u64>(x));
}

@inline export function vend(): void {
  vec_end();
}

// splitmix64
let v_state: u64 = 0;

export function vseed(seed: u64): void {
  v_state = seed;
}

export function vrand64(): u64 {
  v_state += 0x9e3779b97f4a7c15;
  let z = v_state;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
  z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
  return z ^ (z >> 31);
}

export function vrand32(): u32 {
  return <u32>(vrand64() >> 32);
}

// [lo, hi)
export function vrandint(lo: i32, hi: i32): i32 {
  return lo + <i32>(vrand32() % <u32>(hi - lo));
}

// [lo, hi]
export function vrandf(lo: f32, hi: f32): f32 {
  let t: f32 = <f32>(vrand32() >> 8) * <f32>(1.0 / 16777216.0);
  return lo + t * (hi - lo);
}

@inline export function vbitsf(bits: u32): f32 {
  return reinterpret<f32>(bits);
}

const v_specialBits: StaticArray<u32> = [
  0x00000000,
  0x80000000,
  0x3f800000,
  0xbf800000,
  0x3f000000,
  0xbf000000,
  0x40000000,
  0x40400000,
  0x00000001,
  0x80000001,
  0x007fffff,
  0x807fffff,
  0x00400000,
  0x00800000,
  0x80800000,
  0x7f7fffff,
  0xff7fffff,
  0x7f800000,
  0xff800000,
  0x7fc00000,
  0xffc00000,
  0x7fc12345,
  0x7fa00001,
  0xffa00001,
  0x7149f2ca,
  0xf149f2ca,
  0x0da24260,
  0x8da24260,
  0x34000000,
  0x33800000,
  0x4b800000,
  0xcb800001,
];

// A quarter special values (signed zeros, denormals, infinities, NaNs with
// payloads, huge and tiny), a quarter raw bit patterns, half ordinary values.
export function vspecialf(): f32 {
  let pick = vrand32() & 3;
  if (pick == 0) {
    return vbitsf(v_specialBits[<i32>(vrand32() % <u32>v_specialBits.length)]);
  }

  if (pick == 1) {
    return vbitsf(vrand32());
  }

  return vrandf(-100.0, 100.0);
}

// Runs a minor collection under --runtime frame. Call it from the host between
// driver calls, never from inside one: only globals are roots.
export unsafe function frameBoundary(): void {
  if (ASC_RUNTIME == 3) {
    __frameMinor();
  }
}
