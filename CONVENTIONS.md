# Porting conventions

Box3D (C17, github.com/erincatto/box3d) ported to AnimaScript, compiled by rasc.
Upstream pin: `f36d061ba26998c87cb7dc593a4065aaa03fb7eb` (v0.1.0 plus fixes),
checked out at `~/Projects/box3d`. The parity reference is the scalar build at
`~/Projects/box3d/build/scalar` (`BOX3D_DISABLE_SIMD=ON`, `-ffp-contract=off`,
`BOX3D_VALIDATE=OFF`, Release).

The goal is a faithful port: same algorithms, same operation order, same float
results bit for bit. A difference is either float drift we can explain or a bug.

## Toolchain

```sh
RASC=~/Projects/rive-master/packages/rasc/animascript/out/release/rasc
AS_STD=~/Projects/rive-master/packages/rasc/animascript/std
$RASC entry.as --path "$AS_STD" --lib "$AS_STD/assembly" --runtime stub -O -o out.wasm
```

- `--runtime stub` or `--runtime frame`; `minimal` and `incremental` reject
  arrays of structs.
- `rasc check <file> --runtime stub --lib "$AS_STD/assembly"` reports
  diagnostics without output.
- Never edit the compiler or std. When rasc refuses something it should
  accept, or miscompiles, write a minimal repro into your notes file under
  "rasc findings" and work around it.

## Files

- One `.as` per C source, same basename, under `lib/`. A private header
  merges into the `.as` of the same name (`shape.h` + `shape.c` ->
  `lib/shape.as`). Headers without a `.c` get their own file
  (`math_internal.h` -> `lib/math_internal.as`).
- Public headers: `include/box3d/math_functions.h` + `src/math_functions.c`
  -> `lib/math_functions.as`; `types.h` + `types.c` -> `lib/types.as`;
  `collision.h` -> `lib/collision.as`; `constants.h` -> `lib/constants.as`;
  `base.h` + `core.h` + `core.c` -> `lib/core.as`; `box3d.h` is implemented
  across the module files as in C and re-exported from `lib/box3d.as`.
- Keep the order of functions in each file as in C, so the two diff side by
  side.
- Start each file with:
  ```
  // SPDX-FileCopyrightText: 2026 Erin Catto
  // SPDX-License-Identifier: MIT
  // AnimaScript port of src/<file>.c
  ```
- Keep upstream comments where the code they describe survives. Our own
  comments only when a constraint is not visible in the code, one line.
- Each porting task owns a set of files. Edit only those, never commit;
  write deviations, skipped functions and rasc findings to
  `notes/<task>.md`.

## Names

C identifiers stay verbatim: `b3Dot`, `b3MakeBoxHull`, `b3World_Step`,
`b3_dynamicBody`, `B3_LINEAR_SLOP`, struct field names. Static C functions
stay unexported.

## Types

| C | AnimaScript |
|---|---|
| `float` | `f32` |
| `double` | `f64` |
| `int` | `i32` |
| `uint32_t` / `uint16_t` / `uint8_t` / `uint64_t` / `int64_t` | `u32` / `u16` / `u8` / `u64` / `i64` |
| `bool` | `bool` |
| `void* userData` | `u64` |
| `const char* name` | `string` |
| `typedef enum b3X { b3_a, b3_b }` | `export type b3X = i32;` plus `export const b3_a: b3X = 0;` for each enumerator, so call sites read as in C |
| `#define B3_X 4` | `export const B3_X: i32 = 4;` |
| `#define` that reads a runtime global (`B3_LINEAR_SLOP` uses `b3_lengthUnitsPerMeter`) | `@inline export function` with the same name, so it is evaluated at each use |
| fixed array member `b3Vec3 v[3]` | `FixedArray<b3Vec3, 3>` |
| `b3Array(T)` | builtin `Array<T>`; `.count` is `.length` |

Float literals: AnimaScript types a bare literal local as `f64`. Annotate every
float local, `let h: f32 = 0.5;`, and cast integer to float explicitly,
`<f32>i`. A literal inside an `f32` expression (`h * 0.5`) is already `f32`.
An `f64` leaking into float math changes results, so the parity checks will
find it, but avoid it up front. C float to int casts truncate; wasm traps on
NaN or out of range, so where C could see such a value use the saturating
helper from `lib/core.as`.

`sqrtf` is `Mathf.sqrt` (one instruction, exact). `floorf` is `Mathf.floor`.
`fabsf` is `Mathf.abs`. `remainderf` is `Mathf.rem`. Box3D has its own
`b3ComputeCosSin` and `b3Atan2`; port them verbatim and never use
`Mathf.sin`/`cos`/`atan2`, which round differently from libm.

## Structs and classes

- A C struct whose members are all plain data (numbers, other plain structs,
  fixed arrays) becomes a rasc `struct`: `b3Vec3`, `b3Quat`, `b3Transform`,
  `b3AABB`, ids, `b3Manifold`, `b3ManifoldPoint`, `b3Body`, `b3BodySim`,
  `b3BodyState`, `b3ContactSim`, `b3JointSim`, joint defs, shape defs.
  Structs copy on assignment, never allocate, and return by multi-value.
- A C struct that holds a pointer, a `b3Array`, or that C only ever handles
  through a pointer it owns, becomes a reference `class`: `b3World`,
  `b3Shape`, `b3Contact`, `b3Island`, `b3SolverSet`, `b3GraphColor`,
  `b3Sensor`, `b3BroadPhase`, `b3DynamicTree`, `b3HullData`, `b3MeshData`,
  `b3HeightFieldData`, `b3CompoundData`.
- Unions become one field per member.
- Structs cannot be generic arguments of our own classes, so there is no
  container class: use `Array<T>` / `StaticArray<T>` directly. Functions
  generic over the array type do work, so `lib/container.as` has
  `b3Array_RemoveSwap`, `b3Array_AddIndex`, `b3Array_Emplace` (returns an
  index), `b3Array_Resize` and `b3Array_Append` for any `Array<V>`.
- rasc refuses `ref` to a primitive local or field, so `B3_SWAP` is written
  inline (`let t = x; x = y; y = t;`); `b3SwapElements(a, i, j)` swaps array
  elements.
- Arena, stack and block allocators are not ported: each `b3Bump` or
  `b3StackAlloc` becomes a local `StaticArray<T>` and its free disappears,
  `b3Arena` parameters are dropped. `notes/containers.md` lists every site.
- Struct methods are allowed but keep C's free function style
  (`b3Add(a, b)`) for diffability.

## Pointers

- `const T*` input that C only reads: pass `T` (a copy) for small structs,
  `ref T` when the struct is large or C writes through it.
- `T* out` parameters: `ref T`, or return the value when C's function only
  fills one output.
- `T* p = array + i` then reads and writes through `p`: when the array is a
  `StaticArray<T>` or `FixedArray`, `ref p = array[i]`. When it is a
  growable `Array<T>`, rasc refuses a `ref` (push may move it); write through
  the place directly (`arr[i].v.x = ...`) or copy out, mutate, store back.
- A function that returns a pointer into an array returns the index instead,
  and the caller binds the element.
- Pointers stored inside other structs (`b3ContactSim**`) become indices.
- `memcpy` / `memset` of struct arrays become loops or `Array` copies.
- Hull, mesh, height field and compound data are offset addressed blobs in
  C (`b3GetHullPoints(hull)`). Port them as classes with one
  `StaticArray<T>` per section, field names kept, and accessor functions with
  the C names returning the array.

## Callbacks

A C callback plus `void* context` becomes a function typed parameter; the
closure carries the context. Keep the callback's C name as the type alias.

## Asserts and validation

`B3_ASSERT(x)` stays as a call to `B3_ASSERT` from `lib/core.as`, compiled out
unless built with `-D B3_ENABLE_ASSERT`. `B3_VALIDATE` code is skipped.

## Skipped

Debug draw, recording, replay, world snapshots, the internal scheduler,
profiling timers (keep `b3Profile` fields; times read 0) and the SSE2, AVX2
and NEON backends. `simd_scalar.h` is ported: with `BOX3D_DISABLE_SIMD` the
contact solver and SAT still run their 4 lane structure through it, and the
port keeps that structure so it matches bit for bit. Wasm SIMD later replaces
the scalar lane type behind `ASC_FEATURE_SIMD`.

## Verification

Each module gets checks against the native scalar build:

- `verify/cref/<module>.c`: links `~/Projects/box3d/build/scalar/src/libbox3d.a`
  (include `~/Projects/box3d/src` for internals), built with
  `clang -O2 -ffp-contract=off -DBOX3D_DISABLE_SIMD -DNDEBUG`. It prints
  vectors through `verify/cref/common.h`: `vtag`, `vi32`, `vu32`, `vbool`,
  `vf32` (`%08x` bits), `vu64` and `vf64` (`%016llx`), `vend` ends a line.
- `verify/as/<module>.as`: exports driver functions over `lib/` that print the
  same lines through `verify/as/common.as`, which has the same functions.
  Floats cross to the host as bits, so NaN payloads survive.
- `verify/<module>.mjs`: usually three lines, `runModule(name, drive)` from
  `verify/lib.mjs`. It loads the wasm, lets `drive` call the exports, then
  compares line by line with the C vectors, exactly, and prints the first
  mismatches with context and decoded floats, then a total.
- `verify/run.sh [module...]` builds and runs each module and stops at the
  first failure (`KEEP_GOING=1` to continue). Modules are found by file name;
  optional `cref/<m>.flags` and `as/<m>.flags` add compiler arguments.
  `RASC_FLAGS` replaces the default `-O`; run once with `RASC_FLAGS=` too.

Inputs: a fixed set plus a seeded sweep from `vseed`/`vrand32`/`vrandint`/
`vrandf`/`vspecialf` (splitmix64, identical on both sides). C leaves the
evaluation order of arguments and initializers open, so in C drivers draw each
random value in its own statement. A check passes only when the bits match.

Runtime: drivers compile with `--runtime frame`. The stub never frees, and its
`__mark`/`__resetTo` rewind would free a world that outlives the rewind. Under
frame, a minor collection runs only at a frame boundary, where only globals are
roots, so a long run keeps its world in module globals, steps it from the host
one export call at a time, and calls `ctx.frameBoundary()` between calls. Every
driver re-exports `frameBoundary` from `verify/as/common.as`. Never collect from
inside a driver call.

NaN payloads: when both operands of an operation are NaN, clang may commute it
and return the other payload, so C at `-O2` is not source exact there. Pass
`{ nanEqual: true }` to `runModule` to let any NaN match any NaN; the summary
counts those lines. Leave it off unless a mismatch shows only NaN tokens.
