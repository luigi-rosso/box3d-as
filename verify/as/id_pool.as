// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/id_pool.as, mirrors verify/cref/id_pool.c

import { vtag, vi32, vend, vseed, vrand32, vrandint } from "./common";
import {
  b3IdPool,
  b3CreateIdPool,
  b3DestroyIdPool,
  b3AllocId,
  b3FreeId,
  b3GetIdCount,
  b3GetIdCapacity,
  b3GetIdBytes,
} from "../../lib/id_pool";

export { frameBoundary } from "./common";

function emitPool(pool: b3IdPool): void {
  vtag("s");
  vi32(b3GetIdCount(pool));
  vi32(b3GetIdCapacity(pool));
  vi32(b3GetIdBytes(pool));
  vi32(pool.freeArray.length);
  vend();

  vtag("free");
  for (let i = 0; i < pool.freeArray.length; ++i) {
    vi32(pool.freeArray[i]);
  }
  vend();
}

export function run(): void {
  vseed(2);
  let live = new Array<i32>();
  let pool = b3CreateIdPool();
  emitPool(pool);

  for (let phase = 0; phase < 60; ++phase) {
    let ops = vrandint(1, 300);
    for (let k = 0; k < ops; ++k) {
      let r = vrand32() % 5;
      if (r < 3 || live.length == 0) {
        let id = b3AllocId(pool);
        live.push(id);
        vtag("a");
        vi32(id);
        vend();
      } else {
        let index = vrandint(0, live.length);
        let id = live[index];
        live[index] = live[live.length - 1];
        live.pop();
        b3FreeId(pool, id);
        vtag("f");
        vi32(id);
        vend();
      }
    }

    if (phase % 7 == 3) {
      // The id == nextIndex branch
      b3FreeId(pool, pool.nextIndex);
      vtag("n");
      vi32(pool.nextIndex);
      vend();
    }

    emitPool(pool);
  }

  b3DestroyIdPool(pool);
  emitPool(pool);

  for (let k = 0; k < 40; ++k) {
    let id = b3AllocId(pool);
    vtag("a");
    vi32(id);
    vend();
    if (k % 3 == 0) {
      b3FreeId(pool, id / 2);
    }
  }
  emitPool(pool);
}
