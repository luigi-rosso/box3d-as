# rasc and runtime findings

Small issues hit while porting. None of these were fixed in passing; each has
a minimal repro.

## Structs cannot cross lanes

A struct is plain data, but a job argument or result holding one is refused:

```as
export struct Pt { x: f32 = 0; }
export class Batch { pts: StaticArray<Pt> = new StaticArray<Pt>(0); }
@job export function sum(b: Batch): f32 { return b.pts.length > 0 ? b.pts[0].x : 0; }
// E3001: @job sum: b.pts[] (Pt) cannot be sent to another lane
```

Workaround: pack job batches into typed arrays.

## Generic classes over structs

```as
struct V { x: f32 = 0; }
class Arr<T> { data: StaticArray<T> = new StaticArray<T>(0); }
let a = new Arr<V>(); // error: value classes cannot be generic arguments yet
```

Builtin `Array<V>` and `StaticArray<V>` work, so C's `b3Array(T)` maps to
`Array<T>` with per type helpers.

## `--runtime minimal` and `incremental` reject arrays of structs

`new Array<V>()` with a struct `V` fails with "Array needs a runtime; use
--runtime stub" under `--runtime minimal` and `--runtime incremental`;
`stub` and `frame` accept it. The message names the wrong cause.

## `performance.now()` traps in rive-cli

`performance.now()` compiles, then traps at run time:
`failed to call unlinked import function (env, performance.now)`. The host
clock is `clock()` from `rive/os`. Either the std should route
`performance.now` to the host clock or refuse it at compile time.

## `ref` to a primitive local or field is refused, though the message lists them

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

A struct leaf works. Either the docs should say only struct leaves, or the
message should not name locals and fields.

## A type alias of a struct is not callable as its constructor

`export type b3Vec3W = b3Vec3W4;` then `b3Vec3W(x, y, z)` is "unknown
function". `export { b3Vec3W4 as b3Vec3W }` works as both type and
constructor.

## Doc mismatch: struct ternaries

`value-classes.html` lists ternaries on struct values as compile errors;
rasc accepts them and returns correct results.

## Module constants are never folded under `-O`

```as
export const A: u32 = 1 << 31;
const E = 7;
@inline const F: u32 = 0x80000000;
export function f(x: u32): u32 { return (x & A) | <u32>E | (x & F); }
// --wat -O: every constant is a global.get of an immutable global
```

asc inlines constant globals. The release bake's binaryen pass removes the
cost; plain `rasc -O` does not.

## `-O` never inlines struct constructors or struct returning functions

Every `b3Vec3(...)` and helper returning a struct stays a call, `@inline` or
not. The release bake's binaryen pass inlines them; plain `rasc -O` does not.

## A function that folds to nothing stays a call

Its arguments are still evaluated, so `B3_ASSERT(expensive())` costs in a
release build; the port guards hot asserts with a constant.

## `FixedArray` is capped at 255 elements

C's 512 entry stack arrays cannot be locals; the port uses module statics.
