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

## A float literal in a nested ternary types as f64

```as
let t: f32 = m == 0 ? 0.0 : m == 1 ? 1.0 : x; // cannot implicitly convert f64 to f32
let u: f32 = m == 0 ? 0.0 : x;                // fine
```

Workaround: `<f32>0.0`. The same loss happens to a single ternary nested in
an f32 expression: `let x: f32 = a + (c ? 0.0 : 20.0);` fails while
`let x: f32 = c ? 5.0 : 15.0;` compiles.

## Whole struct store through a call result is refused

`getM(s)[0] = m` is refused, while a leaf store `getM(s)[0].x = 1` through the
same call compiles. Binding the array to a local first works.

## `StaticArray.fromArray` traps on an array of structs

```as
struct P { x: f32 = 0; y: f32 = 0; }
export function f(n: i32): f32 {
	let a = new Array<P>();
	for (let i = 0; i < n; ++i) {
		let p = P();
		p.x = <f32>i;
		a.push(p);
	}
	let s = StaticArray.fromArray<P>(a);
	return s[n - 1].x; // traps "unreachable" inside fromArray, with -O and without, --runtime frame
}
```

`fromArray` of an array of class references works. Workaround: a `new StaticArray<P>(a.length)` and
an element copy loop (verify/scenes/scenes.as, the village compound defs).

## `shift()` is refused on arrays of structs

`E3001: 'shift' is not supported on value class arrays yet`; `push`, `pop`
and `length` work.

## WAMR's fast interpreter traps on a v128 local past the first 256 frame cells

Found by the solver pass (notes/perf_solver.md). The SIMD artifact of the bench runs under AOT but traps on
the interpreter tier at the first step: `wasm call trapped in host_obj_advance: invalid local type`. In WAMR
2.4.5 the fast interpreter loader rewrites `local.set` / `local.tee` into the `*_FAST_V128` opcodes only for
frame offsets below 256 (`core/iwasm/interpreter/wasm_loader.c` near 14074); past that it keeps the generic
`WASM_OP_SET_LOCAL`, whose handler in `wasm_interp_fast.c` (near 5677) has i32, f32, i64 and f64 cases and
no v128, so it sets the exception. Any function with more than about 60 live v128 locals hits it. Repro:
a rive-cli wasm project whose `init` calls

```as
function many(x: f32, count: i32): f32 {
	if (ASC_FEATURE_SIMD) {
		let b = f32x4.splat(x);
		let v0 = f32x4.add(b, f32x4.splat(0.0));
		// ... v1 to v79 the same with constants 1 to 79
		for (let k = 0; k < count; ++k) {
			v0 = f32x4.mul(v0, v1);
			// ... vi = f32x4.mul(vi, v(i + 1) % 80) for every i, so all 80 stay live
		}
		let s = f32x4.splat(0.0);
		s = f32x4.add(s, v0); // ... and every other vi
		return f32x4.extract_lane(s, 0);
	}
	return x;
}
```

prints its result with `--aot` and traps with `invalid local type` without `RIVE_WAMRC`. The fix belongs in
WAMR (a v128 case in the generic set/tee handlers, as the 64 bit one). Until then a project that may run
on the interpreter needs `wasmSimd: off`; `bench/as/run.sh` runs its interpreter scenes on that artifact.

## `@inline` does not reach binaryen

rasc emits no `metadata.code.inline` hint, so the release bake's binaryen inlines only by its own size rule
(at most 100 nodes and no calls, `flexibleInlineMaxSize = 100` in `wasm_opt_binaryen.cpp`). An `@inline`
helper that rasc -O does not inline itself (struct returning ones, see above) stays a call when it is
bigger, `b3CrossW4` among them. Helpers that return one scalar or one v128 are inlined by rasc already.

## `dataStart` is refused on value class arrays

`arr.dataStart` on an `Array<V>` of structs is "not supported on value class arrays yet", although the
array keeps the usual header. Raw code reads it as `load<usize>(changetype<usize>(arr) +
offsetof<Array<i32>>("dataStart"))`.

## `isFloat<T>()` folds per instance but is not documented

The docs list `isDefined`, `isBoolean`, `isArray`, `isFunction` and `isNullable` as folding per
monomorphized instance. `isFloat<T>()` folds too and its dead branch never compiles, which is what lets
one generic kernel take `T = v128` under `--enable-simd` and `T = f32` without it (lib/simd.as, the `L`
helpers). There is no `isVector<T>()`. A constant condition in a ternary still compiles both arms, so the
two instantiations have to be picked with an `if`.
