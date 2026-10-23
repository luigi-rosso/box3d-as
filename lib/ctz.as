// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/ctz.h

import { B3_ASSERT } from "./core";

@inline export function b3CTZ32(block: u32): u32 {
  return ctz<u32>(block);
}

@inline export function b3CLZ32(value: u32): u32 {
  return clz<u32>(value);
}

@inline export function b3CTZ64(block: u64): u32 {
  return <u32>ctz<u64>(block);
}

@inline export function b3PopCount64(block: u64): i32 {
  return <i32>popcnt<u64>(block);
}

@inline export function b3IsPowerOf2(x: i32): bool {
  return (x & (x - 1)) == 0;
}

export function b3BoundingPowerOf2(x: i32): i32 {
  if (x <= 1) {
    return 1;
  }

  return 32 - <i32>b3CLZ32(<u32>x - 1);
}

export function b3RoundUpPowerOf2(x: i32): i32 {
  if (x <= 1) {
    return 1;
  }

  return 1 << (32 - <i32>b3CLZ32(<u32>x - 1));
}

export function b3LowerPowerOf2Exponent(x: i32): i32 {
  B3_ASSERT(x > 0);
  let clz = <i32>b3CLZ32(<u32>x);

  // Position of most significant bit = floor(log2(M))
  return 31 - clz;
}
