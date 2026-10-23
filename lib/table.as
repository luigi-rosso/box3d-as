// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/table.c

import { B3_ASSERT } from "./core";
import { B3_SHAPE_POWER, B3_MAX_SHAPES, B3_MAX_CHILD_SHAPES } from "./constants";
import { b3RoundUpPowerOf2 } from "./ctz";

// I need a good hash because the keys are built from pairs of increasing integers.
// A simple hash like hash = (integer1 XOR integer2) has many collisions.
// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
// https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
// todo try: https://www.jandrewrogers.com/2019/02/12/fast-perfect-hashing/
// todo try:
// https://probablydance.com/2018/06/16/fibonacci-hashing-the-optimization-that-the-world-forgot-or-a-better-alternative-to-integer-modulo/
@inline export function b3KeyHash(key: u64): u32 {
  let h = key;
  h ^= h >> 33;
  h *= 0xff51afd7ed558ccd;
  h ^= h >> 33;
  h *= 0xc4ceb9fe1a85ec53;
  h ^= h >> 33;

  return <u32>h;
}

export struct b3SetItem {
  key: u64 = 0;
  hash: u32 = 0;
}

export class b3HashSet {
  items: StaticArray<b3SetItem> = new StaticArray<b3SetItem>(0);
  capacity: u32 = 0;
  count: u32 = 0;
}

export const B3_SHAPE_MASK: i32 = B3_MAX_SHAPES - 1;
export const B3_CHILD_MASK: i32 = B3_MAX_CHILD_SHAPES - 1;

@inline export function b3ShapePairKey(s1: i32, s2: i32, c: i32): u64 {
  if (s1 < s2) {
    return (<u64>(B3_SHAPE_MASK & s1) << <u64>(64 - B3_SHAPE_POWER)) |
      (<u64>(B3_SHAPE_MASK & s2) << <u64>(64 - 2 * B3_SHAPE_POWER)) | <u64>(
      B3_CHILD_MASK & c
    );
  }

  return (<u64>(B3_SHAPE_MASK & s2) << <u64>(64 - B3_SHAPE_POWER)) |
    (<u64>(B3_SHAPE_MASK & s1) << <u64>(64 - 2 * B3_SHAPE_POWER)) | <u64>(
    B3_CHILD_MASK & c
  );
}

export function b3CreateSet(capacity: i32): b3HashSet {
  let set = new b3HashSet();

  // Capacity must be a power of 2
  if (capacity > 16) {
    set.capacity = <u32>b3RoundUpPowerOf2(capacity);
  } else {
    set.capacity = 16;
  }

  set.count = 0;
  set.items = new StaticArray<b3SetItem>(<i32>set.capacity);

  return set;
}

export function b3DestroySet(set: b3HashSet): void {
  set.items = new StaticArray<b3SetItem>(0);
  set.count = 0;
  set.capacity = 0;
}

export function b3ClearSet(set: b3HashSet): void {
  set.count = 0;
  let items = set.items;
  let capacity = <i32>set.capacity;
  for (let i = 0; i < capacity; ++i) {
    items[i] = b3SetItem();
  }
}

function b3FindSlot(set: b3HashSet, key: u64, hash: u32): i32 {
  let capacity = set.capacity;
  let index = <i32>(hash & (capacity - 1));
  let items = set.items;
  while (items[index].hash != 0 && items[index].key != key) {
    index = (index + 1) & <i32>(capacity - 1);
  }

  return index;
}

function b3AddKeyHaveCapacity(set: b3HashSet, key: u64, hash: u32): void {
  let index = b3FindSlot(set, key, hash);
  let items = set.items;
  B3_ASSERT(items[index].hash == 0);

  items[index].key = key;
  items[index].hash = hash;
  set.count += 1;
}

function b3GrowTable(set: b3HashSet): void {
  let oldCount = set.count;

  let oldCapacity = set.capacity;
  let oldItems = set.items;

  set.count = 0;
  // Capacity must be a power of 2
  set.capacity = 2 * oldCapacity;
  set.items = new StaticArray<b3SetItem>(<i32>set.capacity);

  // Transfer items into new array
  for (let i: u32 = 0; i < oldCapacity; ++i) {
    ref item = oldItems[<i32>i];
    if (item.hash == 0) {
      // this item was empty
      continue;
    }

    b3AddKeyHaveCapacity(set, item.key, item.hash);
  }

  B3_ASSERT(set.count == oldCount);
}

export function b3ContainsKey(set: b3HashSet, key: u64): bool {
  // key of zero is a sentinel
  B3_ASSERT(key != 0);
  let hash = b3KeyHash(key);
  let index = b3FindSlot(set, key, hash);
  return set.items[index].key == key;
}

// The caller already has the hash, which lets a batch prefetch the slots before probing them.
export function b3ContainsHashedKey(set: b3HashSet, key: u64, hash: u32): bool {
  B3_ASSERT(key != 0);
  B3_ASSERT(hash == b3KeyHash(key));
  let index = b3FindSlot(set, key, hash);
  return set.items[index].key == key;
}

export function b3GetHashSetBytes(set: b3HashSet): i32 {
  return <i32>set.capacity * <i32>sizeof<b3SetItem>();
}

export function b3AddKey(set: b3HashSet, key: u64): bool {
  // key of zero is a sentinel
  B3_ASSERT(key != 0);

  let hash = b3KeyHash(key);
  B3_ASSERT(hash != 0);

  let index = b3FindSlot(set, key, hash);
  if (set.items[index].hash != 0) {
    // Already in set
    B3_ASSERT(set.items[index].hash == hash && set.items[index].key == key);
    return true;
  }

  if (2 * set.count >= set.capacity) {
    b3GrowTable(set);
  }

  b3AddKeyHaveCapacity(set, key, hash);
  return false;
}

// See https://en.wikipedia.org/wiki/Open_addressing
export function b3RemoveKey(set: b3HashSet, key: u64): bool {
  let hash = b3KeyHash(key);
  let i = b3FindSlot(set, key, hash);
  let items = set.items;
  if (items[i].hash == 0) {
    // Not in set
    return false;
  }

  // Mark item i as unoccupied
  items[i].key = 0;
  items[i].hash = 0;

  B3_ASSERT(set.count > 0);
  set.count -= 1;

  // Attempt to fill item i
  let j = i;
  let capacity = set.capacity;
  for (;;) {
    j = (j + 1) & <i32>(capacity - 1);
    if (items[j].hash == 0) {
      break;
    }

    // k is the first item for the hash of j
    let k = <i32>(items[j].hash & (capacity - 1));

    // determine if k lies cyclically in (i,j]
    // i <= j: | i..k..j |
    // i > j: |.k..j  i....| or |....j     i..k.|
    if (i <= j) {
      if (i < k && k <= j) {
        continue;
      }
    } else {
      if (i < k || k <= j) {
        continue;
      }
    }

    // Move j into i
    items[i] = items[j];

    // Mark item j as unoccupied
    items[j].key = 0;
    items[j].hash = 0;

    i = j;
  }

  return true;
}
