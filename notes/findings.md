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
