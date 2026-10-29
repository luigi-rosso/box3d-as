// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/algorithm.h

// B3_SWAP( x, y ) has no function form: a ref cannot bind a number local.
// Write it inline at each call site, which copies exactly for structs too:
//   let t = x;
//   x = y;
//   y = t;
// Swapping two elements of one array works generically through b3SwapElements.

@inline export function b3SwapElements<A>(a: A, i: i32, j: i32): void {
  let t = a[i];
  a[i] = a[j];
  a[j] = t;
}
