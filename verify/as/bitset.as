// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/bitset.as, mirrors verify/cref/bitset.c

import { vtag, vi32, vu32, vu64, vbool, vend, vseed, vrand32, vrandint } from "./common";
import {
  b3BitSet,
  b3CreateBitSet,
  b3DestroyBitSet,
  b3SetBitCountAndClear,
  b3InPlaceUnion,
  b3GrowBitSet,
  b3CountSetBits,
  b3SetBit,
  b3SetBitGrow,
  b3ClearBit,
  b3GetBit,
  b3GetBitSetBytes,
} from "../../lib/bitset";

export { frameBoundary } from "./common";

function emitSet(tag: string, set: b3BitSet): void {
  vtag(tag);
  vu32(set.blockCapacity);
  vu32(set.blockCount);
  vi32(b3GetBitSetBytes(set));
  vi32(b3CountSetBits(set));
  for (let i: u32 = 0; i < set.blockCapacity; ++i) {
    vu64(set.bits[<i32>i]);
  }
  vend();
}

export function run(): void {
  vseed(3);

  for (let round = 0; round < 40; ++round) {
    let capacity = vrand32() % 700;
    let a = b3CreateBitSet(capacity);
    let b = b3CreateBitSet(vrand32() % 300);
    emitSet("create", a);

    let bitCount = vrand32() % 900;
    b3SetBitCountAndClear(a, bitCount);
    b3SetBitCountAndClear(b, bitCount);
    emitSet("count", a);

    for (let phase = 0; phase < 8; ++phase) {
      let ops = vrandint(1, 120);
      for (let k = 0; k < ops; ++k) {
        let op = vrand32() % 6;
        let limit: u32 = a.blockCount * 64 + 200;
        let bit = vrand32() % limit;
        if (op == 0 && a.blockCount > 0) {
          let inside = vrand32() % (a.blockCount * 64);
          b3SetBit(a, inside);
        } else if (op == 1) {
          b3SetBitGrow(a, bit);
        } else if (op == 2) {
          b3ClearBit(a, bit);
        } else if (op == 3) {
          vtag("get");
          vu32(bit);
          vbool(b3GetBit(a, bit));
          vend();
        } else if (op == 4) {
          b3SetBitGrow(b, vrand32() % (a.blockCount * 64 + 64));
        } else {
          b3ClearBit(b, bit);
        }
      }
      emitSet("a", a);
      emitSet("b", b);

      if (phase == 4) {
        b3GrowBitSet(a, a.blockCount + 1 + vrand32() % 5);
        emitSet("grow", a);
      }

      if (b.blockCount < a.blockCount) {
        b3GrowBitSet(b, a.blockCount);
      }
      if (a.blockCount < b.blockCount) {
        b3GrowBitSet(a, b.blockCount);
      }
      b3InPlaceUnion(a, b);
      emitSet("union", a);
    }

    b3SetBitCountAndClear(a, vrand32() % 2000);
    emitSet("recount", a);

    b3DestroyBitSet(a);
    b3DestroyBitSet(b);
    emitSet("destroy", a);
  }
}
