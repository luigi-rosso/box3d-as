// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/id_pool.c

import { B3_ASSERT } from "./core";

export class b3IdPool {
  freeArray: Array<i32> = new Array<i32>();
  nextIndex: i32 = 0;
  // C's b3Array capacity, kept so b3GetIdBytes reports what C reports
  freeCapacity: i32 = 0;
}

export function b3CreateIdPool(): b3IdPool {
  let pool = new b3IdPool();
  pool.freeCapacity = 32;
  return pool;
}

export function b3DestroyIdPool(pool: b3IdPool): void {
  pool.freeArray = new Array<i32>();
  pool.nextIndex = 0;
  pool.freeCapacity = 0;
}

export function b3AllocId(pool: b3IdPool): i32 {
  let count = pool.freeArray.length;
  if (count > 0) {
    let id = pool.freeArray.pop();
    return id;
  }

  let id = pool.nextIndex;
  pool.nextIndex += 1;
  return id;
}

export function b3FreeId(pool: b3IdPool, id: i32): void {
  B3_ASSERT(pool.nextIndex > 0);
  B3_ASSERT(0 <= id && id < pool.nextIndex);

  // todo does not work with assertion above
  // should probably be `id == pool->nextIndex - 1`
  if (id == pool.nextIndex) {
    pool.nextIndex -= 1;
    return;
  }

  if (pool.freeArray.length >= pool.freeCapacity) {
    pool.freeCapacity = pool.freeCapacity == 0 ? 8 : 2 * pool.freeCapacity;
  }
  pool.freeArray.push(id);
}

export function b3ValidateFreeId(pool: b3IdPool, id: i32): void {}

@inline export function b3GetIdCount(pool: b3IdPool): i32 {
  return pool.nextIndex - pool.freeArray.length;
}

@inline export function b3GetIdCapacity(pool: b3IdPool): i32 {
  return pool.nextIndex;
}

@inline export function b3GetIdBytes(pool: b3IdPool): i32 {
  return pool.freeCapacity * sizeof<i32>();
}
