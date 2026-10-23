// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/table.as, mirrors verify/cref/table.c

import {
  vtag,
  vi32,
  vu32,
  vu64,
  vbool,
  vend,
  vseed,
  vrand32,
  vrand64,
  vrandint
} from "./common";
import {
  b3HashSet,
  b3KeyHash,
  b3ShapePairKey,
  b3CreateSet,
  b3DestroySet,
  b3ClearSet,
  b3AddKey,
  b3RemoveKey,
  b3ContainsKey,
  b3ContainsHashedKey,
  b3GetHashSetBytes,
} from "../../lib/table";

export { frameBoundary } from "./common";

const keys = new StaticArray<u64>(1 << 16);
let keyCount = 0;

// One line per occupied slot, so slot placement and backward shift deletion are compared exactly.
function emitSet(tag: string, set: b3HashSet): void {
  vtag(tag);
  vu32(set.capacity);
  vu32(set.count);
  vi32(b3GetHashSetBytes(set));
  vend();
  for (let i: u32 = 0; i < set.capacity; ++i) {
    let item = set.items[<i32>i];
    if (item.hash != 0 || item.key != 0) {
      vtag("slot");
      vu32(i);
      vu64(item.key);
      vu32(item.hash);
      vend();
    }
  }
}

function makeKey(): u64 {
  let kind = vrand32() % 4;
  if (kind == 3) {
    let key = vrand64();
    return key == 0 ? 1 : key;
  }

  // Pair keys from small increasing shape indices, as the broad phase makes them
  let s1 = vrandint(0, 64 + keyCount / 4);
  let s2 = vrandint(0, 64 + keyCount / 4);
  let c = kind == 0 ? vrandint(0, 8) : 0;
  if (s1 == s2 && c == 0) {
    c = 1;
  }
  return b3ShapePairKey(s1, s2, c);
}

export function begin(): void {
  vseed(4);

  let capacities: StaticArray<i32> = [-3, 0, 1, 16, 17, 31, 32, 33, 100, 1000];
  for (let i = 0; i < capacities.length; ++i) {
    let set = b3CreateSet(capacities[i]);
    vtag("cap");
    vi32(capacities[i]);
    vu32(set.capacity);
    vend();
    b3DestroySet(set);
  }

  let pairs: StaticArray<i32> = [
    0,
    1,
    0,
    1,
    0,
    0,
    5,
    5,
    3,
    4194303,
    0,
    1048575,
    -1,
    7,
    -1,
    100,
    4194304,
    2
  ];
  for (let i = 0; i < 6; ++i) {
    vtag("pair");
    vu64(b3ShapePairKey(pairs[3 * i], pairs[3 * i + 1], pairs[3 * i + 2]));
    vend();
  }
}

// The host calls these with a frame boundary between phases, so the set survives collections
let set = new b3HashSet();
let roundIndex = 0;

export function beginRound(round: i32): void {
  roundIndex = round;
  set = b3CreateSet(vrandint(0, 64));
  keyCount = 0;
  emitSet("create", set);
}

export function phase(): void {
  let adds = vrandint(0, 400);
  for (let k = 0; k < adds; ++k) {
    let key = makeKey();
    if (keyCount > 0 && vrand32() % 8 == 0) {
      key = keys[vrandint(0, keyCount)];
    }
    let found = b3AddKey(set, key);
    if (keyCount < 1 << 16) {
      keys[keyCount++] = key;
    }
    vtag("add");
    vu64(key);
    vu32(b3KeyHash(key));
    vbool(found);
    vend();
  }
  emitSet("added", set);

  let removes = keyCount > 0 ? vrandint(0, adds / 2 + 2) : 0;
  for (let k = 0; k < removes; ++k) {
    let key = vrand32() % 10 == 0 ? makeKey() : keys[vrandint(0, keyCount)];
    let found = b3RemoveKey(set, key);
    vtag("remove");
    vu64(key);
    vbool(found);
    vend();
  }
  emitSet("removed", set);

  let queries = vrandint(0, 100);
  for (let k = 0; k < queries; ++k) {
    let key = vrand32() % 3 == 0 ||
      keyCount == 0 ? makeKey() : keys[vrandint(0, keyCount)];
    vtag("has");
    vu64(key);
    vbool(b3ContainsKey(set, key));
    vbool(b3ContainsHashedKey(set, key, b3KeyHash(key)));
    vend();
  }
}

export function endRound(): void {
  if (roundIndex % 2 == 1) {
    b3ClearSet(set);
    emitSet("clear", set);
    for (let k = 0; k < 50; ++k) {
      let key = makeKey();
      vtag("add");
      vu64(key);
      vbool(b3AddKey(set, key));
      vend();
    }
    emitSet("refill", set);
  }

  b3DestroySet(set);
}
