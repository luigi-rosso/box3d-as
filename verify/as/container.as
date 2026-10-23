// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/container.as and lib/algorithm.as, mirrors verify/cref/container.c

import { vtag, vi32, vf32, vend, vseed, vrand32, vrandint, vrandf } from "./common";
import { b3Array_AddIndex, b3Array_RemoveSwap, b3Array_Append, b3Array_Resize } from "../../lib/container";
import { b3SwapElements } from "../../lib/algorithm";

export { frameBoundary } from "./common";

struct Pair {
  a: f32 = 0;
  b: i32 = 0;
  constructor(a: f32, b: i32) {
    this.a = a;
    this.b = b;
  }
}

function emitArrays(ints: Array<i32>, pairs: Array<Pair>): void {
  vtag("ints");
  vi32(ints.length);
  for (let i = 0; i < ints.length; ++i) {
    vi32(ints[i]);
  }
  vend();
  vtag("pairs");
  vi32(pairs.length);
  for (let i = 0; i < pairs.length; ++i) {
    vf32(pairs[i].a);
    vi32(pairs[i].b);
  }
  vend();
}

export function run(): void {
  vseed(6);
  let ints = new Array<i32>();
  let pairs = new Array<Pair>();
  let src = new StaticArray<i32>(64);

  for (let phase = 0; phase < 200; ++phase) {
    let ops = vrandint(1, 20);
    for (let k = 0; k < ops; ++k) {
      let op = vrand32() % 6;
      if (op == 0) {
        let index = b3Array_AddIndex(ints);
        ints[index] = vrandint(-1000, 1000);
        let p = b3Array_AddIndex(pairs);
        let a = vrandf(-1.0, 1.0);
        pairs[p] = Pair(a, index);
        vtag("add");
        vi32(index);
        vi32(p);
        vend();
      } else if (op == 1 && ints.length > 0) {
        let moved = b3Array_RemoveSwap(ints, vrandint(0, ints.length));
        vtag("rs");
        vi32(moved);
        vend();
      } else if (op == 2 && pairs.length > 0) {
        let moved = b3Array_RemoveSwap(pairs, vrandint(0, pairs.length));
        vtag("rsp");
        vi32(moved);
        vend();
      } else if (op == 3) {
        let n = vrandint(0, 8);
        for (let i = 0; i < n; ++i) {
          src[i] = vrandint(0, 50);
        }
        b3Array_Append(ints, src, n);
      } else if (op == 4) {
        let oldCount = pairs.length;
        let n = vrandint(0, oldCount + 4);
        b3Array_Resize(pairs, n);
        for (let i = oldCount; i < n; ++i) {
          pairs[i] = Pair(<f32>i, -i);
        }
      } else if (pairs.length > 1) {
        let i = vrandint(0, pairs.length);
        let j = vrandint(0, pairs.length);
        b3SwapElements(pairs, i, j);
      }
    }
    emitArrays(ints, pairs);
  }
}
