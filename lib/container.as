// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/container.h

// b3Array(T) is the builtin Array<T>. These are the helpers that are more than
// a builtin member. They are generic over the array type, so one helper serves
// arrays of numbers and of structs.
// Growing through length zero fills, where C leaves new elements uninitialized.

import { B3_ASSERT, B3_NULL_INDEX } from "./core";

// Add a zeroed element and return its index. b3Array_Emplace returns this index too.
@inline export function b3Array_AddIndex<A>(a: A): i32 {
  let index = a.length;
  a.length = index + 1;
  return index;
}

export { b3Array_AddIndex as b3Array_Emplace };

@inline export function b3Array_Resize<A>(a: A, n: i32): void {
  a.length = n;
}

// Append the first n values of src.
export function b3Array_Append<A, S>(a: A, src: S, n: i32): void {
  let count = a.length;
  a.length = count + n;
  for (let i = 0; i < n; ++i) {
    a[count + i] = src[i];
  }
}

// Remove and element by swapping with the last element. If the index is the last element it returns
// B3_NULL_INDEX, otherwise it returns the index of the last element (which is now out of bounds).
export function b3Array_RemoveSwap<A>(a: A, index: i32): i32 {
  B3_ASSERT(0 <= index && index < a.length);

  let count = a.length - 1;
  if (index != count) {
    a[index] = a[count];
    a.length = count;
    return count;
  }

  a.length = count;
  return B3_NULL_INDEX;
}
