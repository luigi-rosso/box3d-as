// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/bitset.c

import { B3_ASSERT } from "./core";
import { b3PopCount64 } from "./ctz";

// Bit set provides fast operations on large arrays of bits.
export class b3BitSet {
  bits: StaticArray<u64> = new StaticArray<u64>(0);
  blockCapacity: u32 = 0;
  blockCount: u32 = 0;
}

function b3InitBitSet(bitSet: b3BitSet, bitCapacity: u32): void {
  bitSet.blockCapacity = (bitCapacity + sizeof<u64>() * 8 - 1) / (
    sizeof<u64>() * 8
  );
  bitSet.blockCount = 0;
  bitSet.bits = new StaticArray<u64>(<i32>bitSet.blockCapacity);
}

export function b3CreateBitSet(bitCapacity: u32): b3BitSet {
  let bitSet = new b3BitSet();
  b3InitBitSet(bitSet, bitCapacity);
  return bitSet;
}

export function b3DestroyBitSet(bitSet: b3BitSet): void {
  bitSet.blockCapacity = 0;
  bitSet.blockCount = 0;
  bitSet.bits = new StaticArray<u64>(0);
}

export function b3SetBitCountAndClear(bitSet: b3BitSet, bitCount: u32): void {
  let blockCount: u32 = (bitCount + sizeof<u64>() * 8 - 1) / (sizeof<u64>() * 8);
  if (bitSet.blockCapacity < blockCount) {
    b3DestroyBitSet(bitSet);
    let newBitCapacity: u32 = bitCount + (bitCount >> 1);
    b3InitBitSet(bitSet, newBitCapacity);
  }

  bitSet.blockCount = blockCount;
  let bits = bitSet.bits;
  for (let i: u32 = 0; i < blockCount; ++i) {
    bits[<i32>i] = 0;
  }
}

export function b3GrowBitSet(bitSet: b3BitSet, blockCount: u32): void {
  B3_ASSERT(blockCount > bitSet.blockCount);
  if (blockCount > bitSet.blockCapacity) {
    let oldCapacity = bitSet.blockCapacity;
    bitSet.blockCapacity = blockCount + blockCount / 2;
    let newBits = new StaticArray<u64>(<i32>bitSet.blockCapacity);
    let oldBits = bitSet.bits;
    for (let i: u32 = 0; i < oldCapacity; ++i) {
      newBits[<i32>i] = oldBits[<i32>i];
    }
    bitSet.bits = newBits;
  }

  bitSet.blockCount = blockCount;
}

export function b3InPlaceUnion(setA: b3BitSet, setB: b3BitSet): void {
  B3_ASSERT(setA.blockCount == setB.blockCount);
  let blockCount = setA.blockCount;
  let bitsA = setA.bits;
  let bitsB = setB.bits;
  for (let i: u32 = 0; i < blockCount; ++i) {
    bitsA[<i32>i] |= bitsB[<i32>i];
  }
}

// In C this lives in table.c because that file includes ctz.h
export function b3CountSetBits(bitSet: b3BitSet): i32 {
  let popCount = 0;
  let blockCount = bitSet.blockCount;
  let bits = bitSet.bits;
  for (let i: u32 = 0; i < blockCount; ++i) {
    popCount += b3PopCount64(bits[<i32>i]);
  }

  return popCount;
}

@inline export function b3SetBit(bitSet: b3BitSet, bitIndex: u32): void {
  let blockIndex = bitIndex / 64;
  B3_ASSERT(blockIndex < bitSet.blockCount);
  bitSet.bits[<i32>blockIndex] |= (<u64>1) << <u64>(bitIndex % 64);
}

@inline export function b3SetBitGrow(bitSet: b3BitSet, bitIndex: u32): void {
  let blockIndex = bitIndex / 64;
  if (blockIndex >= bitSet.blockCount) {
    b3GrowBitSet(bitSet, blockIndex + 1);
  }
  bitSet.bits[<i32>blockIndex] |= (<u64>1) << <u64>(bitIndex % 64);
}

@inline export function b3ClearBit(bitSet: b3BitSet, bitIndex: u32): void {
  let blockIndex = bitIndex / 64;
  if (blockIndex >= bitSet.blockCount) {
    return;
  }
  bitSet.bits[<i32>blockIndex] &= ~((<u64>1) << <u64>(bitIndex % 64));
}

@inline export function b3GetBit(bitSet: b3BitSet, bitIndex: u32): bool {
  let blockIndex = bitIndex / 64;
  if (blockIndex >= bitSet.blockCount) {
    return false;
  }
  return (bitSet.bits[<i32>blockIndex] & ((<u64>1) << <u64>(bitIndex % 64))) != 0;
}

@inline export function b3GetBitSetBytes(bitSet: b3BitSet): i32 {
  return <i32>bitSet.blockCapacity * sizeof<u64>();
}
