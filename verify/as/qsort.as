// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/qsort.as, mirrors verify/cref/qsort.c

import {
  vtag,
  vi32,
  vf32,
  vu64,
  vend,
  vseed,
  vrand32,
  vrand64,
  vrandint,
  vrandf
} from "./common";
import { QSORT, Q_LOOP } from "../../lib/qsort";

export { frameBoundary } from "./common";

struct Item {
  key: f32 = 0;
  index: i32 = 0;
}

const items = new StaticArray<Item>(2048);
const pairKeys = new StaticArray<u64>(2048);
let lessCount = 0;

// Ties and signed zeros compare equal, so the output order exposes the exact algorithm.
function sortItemsClosure(n: i32): void {
  lessCount = 0;
  QSORT(n, (i: i32, j: i32): bool => {
    ++lessCount;
    return items[i].key < items[j].key;
  }, (i: i32, j: i32): void => {
    let tmp = items[i];
    items[i] = items[j];
    items[j] = tmp;
  });
}

class ItemSorter {
  @inline less(i: i32, j: i32): bool {
    ++lessCount;
    return items[i].key < items[j].key;
  }
  @inline swap(i: i32, j: i32): void {
    let tmp = items[i];
    items[i] = items[j];
    items[j] = tmp;
  }
}

function sortItemsSorter(n: i32): void {
  lessCount = 0;
  Q_LOOP<ItemSorter>(n, new ItemSorter());
}

function sortKeys(n: i32): void {
  QSORT(
    n,
    (i: i32, j: i32): bool => pairKeys[i] < pairKeys[j],
    (i: i32, j: i32): void => {
      let tmp_ = pairKeys[i];
      pairKeys[i] = pairKeys[j];
      pairKeys[j] = tmp_;
    }
  );
}

function emitItems(tag: string, n: i32): void {
  vtag(tag);
  vi32(n);
  vi32(lessCount);
  for (let i = 0; i < n; ++i) {
    vi32(items[i].index);
    vf32(items[i].key);
  }
  vend();
}

export function run(): void {
  vseed(5);

  let sizes: StaticArray<i32> = [
    0,
    1,
    2,
    3,
    4,
    5,
    15,
    16,
    17,
    18,
    31,
    32,
    33,
    100,
    257,
    1000,
    2048
  ];
  let copy = new StaticArray<Item>(2048);
  for (let shape = 0; shape < 6; ++shape) {
    for (let s = 0; s < sizes.length; ++s) {
      let n = sizes[s];
      let range = vrandint(1, 2 * n + 2);
      for (let i = 0; i < n; ++i) {
        let key: f32;
        if (shape == 0) {
          key = <f32>vrandint(0, range);
        } else if (shape == 1) {
          key = <f32>i;
        } else if (shape == 2) {
          key = <f32>(n - i);
        } else if (shape == 3) {
          key = 7.0;
        } else if (shape == 4) {
          key = (vrand32() & 1) != 0 ? 0.0 : -0.0;
        } else {
          key = vrandf(-1.0, 1.0);
        }
        items[i].key = key;
        items[i].index = i;
      }

      for (let i = 0; i < n; ++i) {
        copy[i] = items[i];
      }
      sortItemsClosure(n);
      emitItems("closure", n);
      for (let i = 0; i < n; ++i) {
        items[i] = copy[i];
      }
      sortItemsSorter(n);
      emitItems("sorter", n);
    }
  }

  for (let round = 0; round < 50; ++round) {
    let n = vrandint(0, 600);
    for (let i = 0; i < n; ++i) {
      pairKeys[i] = vrand32() % 4 == 0 &&
        i > 0 ? pairKeys[vrandint(0, i)] : vrand64() >> <u64>(vrand32() % 64);
    }
    sortKeys(n);
    vtag("keys");
    vi32(n);
    for (let i = 0; i < n; ++i) {
      vu64(pairKeys[i]);
    }
    vend();
  }
}
