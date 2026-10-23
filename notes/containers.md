# containers

Containers, sorting, bit tricks, the scalar SIMD lanes and the shared parity
harness.

## Ported

| C | AnimaScript | Notes |
|---|---|---|
| `src/id_pool.h/.c` | `lib/id_pool.as` | `b3IdPool` is a class. `freeCapacity` mirrors C's `b3Array` capacity (reserve 32, then doubling from 8) so `b3GetIdBytes` reports what C reports. `b3ValidateFreeId` is the non validation no-op. |
| `src/bitset.h/.c` | `lib/bitset.as` | `b3BitSet` is a class over `StaticArray<u64>` sized `blockCapacity`. `b3CountSetBits` moves here from `table.c`. |
| `src/table.h/.c` | `lib/table.as` | `b3HashSet` is a class over `StaticArray<b3SetItem>`, slot exact: same hash, power of two capacity, growth at half load, linear probing, backward shift removal. `B3_SHAPE_MASK`/`B3_CHILD_MASK` are exported consts. The debug probe counter is skipped. |
| `src/ctz.h` | `lib/ctz.as` | `ctz`/`clz`/`popcnt` builtins. `b3CTZ32(0)` and `b3CLZ32(0)` return 32 where C is undefined. |
| `src/qsort.h` | `lib/qsort.as` | The exact Tourbin quicksort. `QSORT(n, LESS, SWAP)` takes closures; `Q_LOOP<S>(n, sorter)` takes a class with `less(i, j)` and `swap(i, j)` methods for direct calls at a hot site. Indices are `i32` (the C `unsigned long` never wraps). The 32 frame stack is a module static. |
| `src/container.h` | `lib/container.as` | See below. |
| `src/algorithm.h` | `lib/algorithm.as` | `B3_SWAP` has no function form (see rasc findings); write it inline. `b3SwapElements<A>(a, i, j)` swaps two elements of any array. |
| `src/simd.h/.c`, `simd_scalar.h`, `simd_wide.h` | `lib/simd.as` | See below. |

### container.h

`b3Array(T)` is `Array<T>`. The helpers that are more than a member are generic
over the array type, which rasc accepts for arrays of structs even though a
struct cannot itself be a generic argument, so no per type helpers are needed:

- `b3Array_AddIndex(a)` and `b3Array_Emplace(a)` grow by one zeroed element and
  return its index (Emplace returns an index, not a pointer).
- `b3Array_Resize(a, n)`, `b3Array_Append(a, src, n)`, `b3Array_RemoveSwap(a, i)`
  (returns the moved index or `B3_NULL_INDEX`, as C).
- Members: Push `push`, Pop `pop`, Get `a[i]`, Clear `a.length = 0`, count
  `length`. Reserve and CreateN have no effect on results; note
  `new Array<T>(n)` makes n elements, it does not reserve. `b3Array_ByteCount`
  and `b3Array_MemZero` are not ported; MemZero only matters for elements past
  the count, and growing through `length` zero fills anyway.

### simd

Every function and macro of `simd_scalar.h`, `simd.h` (the `b3V32` path and the
W4 wide vec3) and `simd.c` has a counterpart with the same name, in the same
order. AVX2, SSE2, NEON and all W8 code are skipped. Signature changes, all
forced by pointers:

- `b3LoadW4(data, index)` / `b3StoreW4(data, index, a)` take a
  `StaticArray<f32>` plus the index C's pointer points at; `b3LoadVW4` /
  `b3StoreVW4` take three (array, index) pairs.
- `b3LoadV(v: b3Vec3)`: every C call site is `b3LoadV( &p.x )`.
- `b3SoftMaskW4(indexA, indexB)` takes `FixedArray<i32, 4>`.
- `b3TransposeW4(r0, r1, r2, r3, c0, c1, c2, c3)` writes through `ref`
  params; passing the same `pad` local twice works as in C.
- `b3OverlapNode(av, node)` takes the node by value; `b3UnionPairV(nodes, pair)`
  takes the node array (generic, any array of `b3TreeNode`) and the index;
  `b3StoreAABBV(ref aabb, value, condition)`.
- `b3GetV` replaces the `b3128` union with a ternary.
- `B3_TRANSPOSE3` is inlined in `b3IntersectRayTriangle`.
- `b3_simdWidth` is a plain global; `b3GetSIMDWidth` is always 4.
  `b3IsAVX2Available` lives in `lib/core.as` (the math task put it there), so
  simd.as does not define it again.

`simd_wide.h` names: `b3FloatW`, `b3Vec3W` and every `b3...W` function are
export aliases of the W4 names (`export { b3AddW4 as b3AddW }`), so the `.inl`
ports import the W names and read like C, constructors included
(`b3Vec3W(X, Y, Z)`). `B3_SIMD_WIDTH` is 4 and `B3_WIDE_ALIGNMENT` 16.

For the `.inl` porters: C feeds `b3LoadW` from structs cast to `float*`
(`b3LoadW( (float*)state )`). In AS build those rows from the fields with
`b3SetW`, or keep the data in `StaticArray<f32>`. The loads and stores are
bounds checked; switch them to `unchecked()` once the wide solver passes.

## Arena and block allocators: not ported

Per CONVENTIONS, scratch memory becomes plain arrays, and nothing in Box3D
reads scratch memory before writing it, so dropping the allocators changes no
results. No AS version of either is provided.

- `b3Arena` / `b3Bump` (per task context arena, passed by value so it rewinds on
  return): drop the `b3Arena arena` parameter and make each `b3Bump` a local
  `new StaticArray<T>(n)`. Pointer arrays become index arrays
  (`b3LocalManifold**` to `StaticArray<i32>`). Sites: `contact.c` 483, 491, 620,
  738; `contact.h` 181, 185; `mesh_contact.c` 77, 106, 137, 476, 484, 501, 530,
  545, 547, 549, 573, 576, 585, 586, 980, 981, 1067, 1119, 1139.
  `b3CreateArena` (`physics_world.c` 177) and `b3ArenaSync` (930) vanish.
- `b3Stack` / `b3StackAlloc` / `b3StackFree` (`world->stack`): local arrays; the
  frees vanish. Sites: `broad_phase.c` 664, 677, 719, 774, 776;
  `physics_world.c` 254, 850, 897, 1170 (`b3GrowStack`); `solver.c` 1474, 1572,
  1574, 1575, 1593, 1648, 1654, 1667, 1669, 1790 to 1796, 1983 to 1994, 2244;
  `body.c` 976, 1024; `island.c` 410 to 417, 487, 505 to 507, 528, 534 to 536,
  559, 641 to 648.
- `b3BlockAllocator`: only manifold storage uses it (`b3AllocateManifolds` /
  `b3FreeManifolds` in `physics_world.h` 325 to 361, destroy at
  `physics_world.c` 529, byte count at 2464). C zeroes the block, so
  `new StaticArray<b3Manifold>(count)` is exact; the allocator array, its mutex
  and the free call vanish.
- Counters: `b3World_GetCounters` fills `stackUsed` (`physics_world.c` 2644)
  and `arenaCapacity` (2661 to 2676) from these; they read 0 in the port, and
  `b3GetWorldByteCount` cannot match C.
- Under `--runtime frame` the scratch arrays die at the next frame boundary.
  If a profile shows the per contact arrays in `mesh_contact.c` matter, a
  per world scratch pool can replace them later without changing results.

Not mine but found on the way: `sensor.c` 238 sorts with libc `qsort` and a
comparator that never returns 0. Ties (same `shapeId`, possibly different
`generation`) order by the libc algorithm, so an AS sort matches only while
duplicates are identical. The sensor porter should know.

## Harness

```sh
verify/run.sh               # every module, stops at the first failure
verify/run.sh table simd    # only these
KEEP_GOING=1 verify/run.sh  # report every module
RASC_FLAGS= verify/run.sh   # unoptimized codegen (default is -O)
```

Each step: build `verify/cref/<m>.c` against `libbox3d.a`
(`clang -O2 -ffp-contract=off -DBOX3D_DISABLE_SIMD -DNDEBUG -std=gnu17`),
write `verify/vectors/<m>.txt`, compile `verify/as/<m>.as` with
`--runtime frame -O` to `verify/out/<m>.wasm`, run `node verify/<m>.mjs`.
Outputs are gitignored. `BOX3D`, `BOX3D_BUILD`, `RASC`, `AS_STD`, `CC` override
the paths.

Adding a module `foo`: drop in three files, nothing to register.

1. `verify/cref/foo.c`: `#include "common.h"` and the box3d headers, print lines
   with `vtag("name")`, `vi32`, `vu32`, `vbool`, `vf32`, `vu64`, `vf64`, end
   each with `vend()`. Draw inputs with `vseed`, `vrand32`, `vrand64`,
   `vrandint(lo, hi)`, `vrandf(lo, hi)`, `vspecialf()` (signed zeros,
   denormals, infinities, NaN payloads, huge, raw bits). One draw per statement:
   C does not order argument or initializer evaluation.
2. `verify/as/foo.as`: import the same helpers from `./common` and the code
   from `../../lib/...`, print the same lines, `export { frameBoundary } from
   "./common";`.
3. `verify/foo.mjs`:
   ```js
   import { runModule } from './lib.mjs';
   await runModule('foo', (e, ctx) => e.run());
   ```
   `ctx.frameBoundary()` collects between calls; `ctx.sink.lines` holds what
   the driver printed so far.

Optional `verify/cref/foo.flags` / `verify/as/foo.flags` add compiler arguments
(one line). The comparator prints the first ten mismatching lines with two
lines of context and each differing token, f32 bits decoded, then a total. A
trap reports the last line printed and the next expected C line, so the
failing op is the one right after it. `verify/lib.mjs` also exports
`loadWasm`, `readVectors`, `compareLines`, `f32ToBits`, `bitsToF32`,
`f64ToBits`, `bitsToF64`, `hex32`, `hex64` for comparators that need more.

### Runtime

`--runtime frame`. Probed: stepping a world that keeps 2000 records in a global
while allocating 1000 scratch arrays per step stays at 25 pages with a frame
boundary per step and reaches 9777 pages without; the stub hits the 1 GB
ceiling (it never frees, and `__mark`/`__resetTo` would free the world too).
A minor collection is `__frameMinor()`, legal only with no managed locals live,
so `verify/as/common.as` exports `frameBoundary` (gated on `ASC_RUNTIME == 3`,
a no-op under stub) and the host calls it between driver calls. Keep long
lived state in module globals and step it one export call at a time:
`verify/table.mjs` does this, collecting between every phase, so the hash set
is moved by the collector mid run and still compares exactly. Closures are
only suite tested under the GC runtimes, another reason for frame.

### NaN payloads

With both operands NaN, which payload comes out depends on operand order, and
clang at `-O2` commutes multiplies. 26 of 3000 `vec3w` lines differ that way
against an `-O2` driver (AS keeps source order, `-O0` C matches all 86030
lines). So `cref/simd.flags` builds that driver at `-O0` for a source exact
check, and `NAN_EQUAL=1` with an `-O2` reference checks everything else.
`runModule(..., { nanEqual: true })` lets any NaN match any NaN and counts
those lines in the summary; physics modules may need it where NaNs meet.

## Checks

All pass bit exact against the real `lib/`, with `-O` and without:

| module | lines | values |
|---|---|---|
| bitset | 4307 | 48451 |
| container | 1360 | 130192 |
| ctz | 40009 | 258727 |
| id_pool | 8153 | 16532 |
| qsort | 254 | 100316 |
| simd | 86030 | 1882350 |
| table | 156985 | 624165 |

- id_pool: 60 phases of random alloc/free plus the `id == nextIndex` branch,
  destroy and reuse; count, capacity, bytes and the free list after each phase.
- bitset: create, set count, set/grow/clear/get, grow, union, recount,
  destroy; every block up to capacity, so stale blocks past the count match too.
- table: create capacities, pair keys, 6 rounds of 12 add/remove/contains
  phases with pair keys and random keys, clear and refill; every occupied slot
  (index, key, hash) after each phase.
- qsort: 6 key shapes (many ties, sorted, reversed, all equal, signed zeros,
  random) at sizes around the insertion threshold up to 2048, compared by
  original index order and comparison count, through both `QSORT` and
  `Q_LOOP`; u64 pair keys with duplicates.
- container: AddIndex, RemoveSwap, Append, Resize, element swap on int and
  struct arrays.
- simd: every lane op over 3000 draws of specials and raw bits, masks,
  embed/min index, transposes with aliased outputs, AABB helpers, the `b3V32`
  ops, wide vec3 ops, loads and stores, 12000 bounds/ray/triangle tests, SIMD
  width.

## Wasm SIMD later

Each lane op maps to v128 behind `if (ASC_FEATURE_SIMD)`, with `b3FloatW4`
becoming one `v128`. Nothing is written yet. Exact replacements:

- Zero, Splat, Set, Neg, Add, Sub, Mul, Div, Sqrt: `f32x4` ops directly
  (`f32x4.neg` flips only the sign, like `-x`). MulAdd stays `add(a, mul(b, c))`;
  `relaxed_madd` fuses and rounds differently.
- Load/Store: `v128.load`/`v128.store` at the element address (unsafe).
- Abs: `x < 0 ? -x : x` keeps `-0` and leaves a negative NaN alone;
  `f32x4.abs` clears the sign. Use `bitselect(neg(a), a, lt(a, 0))`.
- Min/Max: scalar is `a <= b ? a : b` / `a >= b ? a : b`. `f32x4.min/max`
  differ on NaN and signed zero; `pmin/pmax` differ when a NaN is involved. Use
  `bitselect(a, b, le(a, b))` / `bitselect(a, b, ge(a, b))`. SymClamp is two of
  these with `neg(b)`.
- The `b3V32` min and max use strict compares and map exactly to
  `f32x4.pmin(b, a)` / `f32x4.pmax(b, a)`.
- Masks: the scalar path stores 1.0 or 0.0 and tests `!= 0`, so NaN is true and
  `-0` is false. The comparisons produce all ones lanes in v128. Either convert
  back (`and(mask, splat(1.0))`) wherever a mask is stored or used as a number,
  or prove the wide code only feeds masks to And/Or/AndNot/Blend/AnyTrue/
  AllZero/MoveMask. To stay exact for any input: AllZero
  `all_true(eq(a, 0))`, AnyTrue `any_true(ne(m, 0))`, MoveMask
  `i32x4.bitmask(ne(m, 0))`, Blend `bitselect(b, a, ne(mask, 0))`. A raw
  `v128.any_true` would treat `-0` as true.
- SoftMask: `i32x4.eq` with zero on both index vectors, `or`, then and with
  `splat(1.0)`.
- EmbedIndex: integer `andnot` and `or` with `(base, base+1, base+2, base+3)`;
  exact.
- MinIndex: the reduction is sequential (`m = y < m ? y : m`, x then y, z, w);
  `pmin(m, y)` matches one step, but a tree reduction changes the order for
  ties and NaN. Extract lanes or chain shuffles in that order.
- Transpose and Dot3V splats: shuffles, exact. Dot keeps `(x + y) + z`.
- NaN payloads of `f32x4` arithmetic are nondeterministic by the wasm spec;
  check them on V8 and WAMR before relying on parity there.

## rasc findings

No compiler or std changes.

1. `ref` to a primitive local or a primitive class field is refused, and the
   message names exactly those as valid places:
   ```as
   class O { a: i32 = 1; b: i32 = 2; }
   function swapI(x: ref i32, y: ref i32): void { let t = x; x = y; y = t; }
   export function f(): i32 {
     let o = new O();
     swapI(o.a, o.b); // error: ref needs a place (a local, field, or element); this expression is a temporary
     let p = 1;
     let q = 2;
     swapI(p, q);     // same error
     return o.a + p;
   }
   ```
   A struct leaf works (`swapI(s.a, s.b)` with `s` a struct local or a struct
   field of an object). Either the docs should say only struct leaves, or the
   message should not list locals and fields. This is why `B3_SWAP` stays
   inline.
2. A constant expression of float literals is `f64` even inside an `f32`
   expression: `let t: f32 = <f32>x * (1.0 / 16777216.0);` fails with "cannot
   implicitly convert f64 to f32". Cast the constant: `<f32>(1.0 / 16777216.0)`.
   asc behaves the same; listed because CONVENTIONS says a literal in an f32
   expression is already f32, which holds for one literal only.
3. A compile only type checks functions reachable from the entry, so a module
   can compile while an unused function in it does not type check; `rasc check
   <file>` reports it. Run `rasc check lib/<file>.as` on every lib file, not
   just the driver compile.
4. A `type` alias of a struct is not callable as its constructor
   (`b3Vec3W(...)` is "unknown function"); `export { b3Vec3W4 as b3Vec3W }` is,
   and also serves as the type. An export alias is visible to importers only,
   not inside its own module, as in ES modules.
5. Useful, not a bug: a function generic over the array type (`function
   f<A>(a: A)`) accepts `Array<V>` and `StaticArray<V>` for a struct `V`, with
   element reads, writes, `length` assignment and struct temporaries, although
   `f<V>` itself is refused. That covers the `b3Array_*` macros generically.
