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

Fixed on the `parallel-calls` toolchain (a4fa1239d4, from #15091): `-O` folds all three to immediates.

## `-O` never inlines struct constructors or struct returning functions

Every `b3Vec3(...)` and helper returning a struct stays a call, `@inline` or
not. The release bake's binaryen pass inlines them; plain `rasc -O` does not.

Fixed by #15091: struct constructors and struct helpers splice under `-O`.

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
on the interpreter needs `wasmSimd: off`, or functions that keep their v128 locals below cell 256. The solver
kernels now do (notes/perf_solver.md, round 2), and `bench/as/tools/v128cells.py <module.wasm>` lists each
function's last v128 offset in a bake; `bench/as/run.sh` runs it after a SIMD session.

Fixed by #15091 (WAMR patch 0030): the 80 local repro prints the same on the interpreter and on AOT with the `parallel-calls` CLI, so bench/as/run.sh no longer runs the cell check and the kernels no longer reload values to stay under it (notes/perf_4.md).

## `@inline` does not reach binaryen

rasc emits no `metadata.code.inline` hint, so the release bake's binaryen inlines only by its own size rule
(at most 100 nodes and no calls, `flexibleInlineMaxSize = 100` in `wasm_opt_binaryen.cpp`). An `@inline`
helper that rasc -O does not inline itself (struct returning ones, see above) stays a call when it is
bigger, `b3CrossW4` among them. Helpers that return one scalar or one v128 are inlined by rasc already.

Fixed by #15091: `@inline` and `@noinline` reach binaryen as code annotations. `b3BarycentricCoordsTri` is a helper again and the release bake inlines it at its five sites.

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

## WAMR AOT reloads the memory base and bound after every store in a function that makes a call

Found by the third perf pass (notes/perf_3.md). WAMR 2.4.5 decides per function whether linear memory can move
under it: `create_memory_info` in `core/iwasm/compilation/aot_llvm.c` (near 1220) sets
`mem_space_unchanged = (!has_op_memory_grow && !has_op_func_call) || !possible_memory_grow`. Our module grows
memory (the frame collector), so every function that contains a `call` or `call_indirect` loads
`mem_base` and `mem_bound_check_*` from the module instance at each access instead of once at entry
(`aot_emit_memory.c` near 115), and LLVM can only reuse them until the next store, which may alias the
instance. In the arm64 code that is `ldr xB, [x19, #0x1b8]` (bound) and `ldr xM, [x19, #0x178]` (base)
after every store. Counted in a release bake (`wamrc --format=object`, the ladder's flags): b3ShapeDistance
had 1125 such loads in 6121 instructions while it called three helpers; with the helpers written into it
(no calls left) it has 2 loads in 4023 instructions, and the query scene's shape casts ran 27% faster. The
iteration kernels and prepare, which make no calls, load both once. Repro: any function that stores to memory
in a loop and also calls a function, compiled by wamrc with `--bounds-checks=1`; remove the call and the
reloads go. The fix belongs in WAMR (treat the base and bound as invariant between calls, since only a call
or memory.grow can move them). Until then hot store heavy code should live in functions without calls.

Fixed by #15091 (WAMR patch 0037): in a `parallel-calls` bake `wamrc --format=object` shows 0 to 3 base or bound reloads per function (only after calls) in the solver kernels, `b3ShapeDistance`, the joint solvers, `b3UpdateConvexContact` and `b3ComputeMeshManifoldsAt`.

## There is no way to keep a function out of line in the release bake

Binaryen (version 133, `src/passes/Inlining.cpp`, `worthFullInlining`) inlines every function with one
direct caller regardless of size (`oneCallerInlineMaxSize` is unlimited), and rasc emits no
`metadata.code.inline` hint that binaryen would honor as never inline. Together with the WAMR finding above
this means a call free leaf cannot be split out of a caller that makes calls: the leaf is inlined back and
inherits the reloads. A module constant referencing the function (`const f = leaf;`) is dropped as unused
and does not help. What works is a second direct call site (lib/mesh_contact.as calls b3PrefilterTriangles
once for meshes and once for height fields), which is a hack. A `@noinline` decorator, or `@inline(never)`
emitted as the code annotation, would solve it.

Fixed by #15091: `@noinline`. `b3PrefilterTriangles` has one call site with it, and the joint type functions use it to stay out of `b3SolveJoint` as in C.

## `@inline` is not honored for functions that take a struct parameter

The docs promise "guaranteed inlining under -O regardless of size". With a value class parameter rasc -O
keeps the call:

```as
struct V { x: f32 = 0; y: f32 = 0; z: f32 = 0; }
@inline function pickS(ax: f32, bx: f32, k: i32): f32 {
	if (k == 0) {
		return bx - ax;
	}
	return bx + ax;
}
@inline function pickV(a: V, b: V): f32 {
	return b.x - a.x;
}
export function f(x: f32, y: f32): f32 {
	let a: V; a.x = x; let b: V; b.y = y;
	return pickS(x, y, 0) + pickS(x, y, 1) + pickV(a, b);
}
// rasc f.as --runtime stub -O --wat: pickS is inlined, pickV stays `call $pickV`
```

The release bake's binaryen then inlines it when it is at most 100 nodes, so small helpers are fine, but a
larger `@inline` helper with struct parameters stays a call (b3BarycentricCoordsTri, which lib/distance.as now
writes out at its five sites).

Fixed by #15091: the repro inlines `pickV`.

## The host clock reads the environment on every call

`rtNowImpl` in `packages/runtime/src/wasm/wasm_scripting_vm.cpp` (near 1756) calls
`getenv("RIVE_WASM_FIXED_DATE")` on every `clock()`; the date variant next to it caches the lookup in a static.
b3Profile reads the clock about 40 times per step, as C does with mach time. In the mover scene (a 2 µs step)
the CPU profile has `__findenv_locked` / `getenv` / `steady_clock::now` / `wasm_runtime_invoke_native_quick`
under `rtNow` at about 17% of all samples, against 26% for all AOT script code, which is why the smallest
scenes run 4 to 7 times native per step. A static flag, as `rtDateNowImpl` has, would fix it.

Fixed by #15091.

## Jobs and workers have no clock

Found by the threading pass (notes/threading.md). Lane code that reaches `clock()` from `rive/os` fails the build,
although the error lists timers among what a job may use:

```as
import { clock } from "rive/os";
@job export function timed(n: i32): f64 {
	let t = clock();
	return clock() - t;
}
// E3004: jobs module: job code reaches rive_rt_v1.now, which a job cannot use; it sees only its arguments, shared
// memory, pure std code and console logs and timers (through timed -> clock -> RtNative.now)
```

So a lane cannot time its own stages. lib/lane_common.as times only the main lane's partition, through a clock
function the main lane installs and the lanes see as null; per lane stage times are not measurable.

Fixed by #15091 for jobs and workers, and on `parallel-calls` for parallel code: lib/parallel_step.as now fills the solver stage times of b3Profile on part 0, as C times them on its main thread.

## A contended region lock parks its waiters

`SharedRegion.exclusive` takes `SharedRegion::lock`, which is `std::timed_mutex::try_lock_until`
(runtime/src/wasm/wasm_shared.cpp). Contended, it waits on a condition variable, so every handoff pays a thread
wake. bench/threads/micro (`locks`), N workers each running 20000 times
`region.exclusive((view) => { view.readInto(0, out); })` with a 64 byte `out`, Apple M5 Max, load 11 to 13:

| workers | one region for all, us per lock | a region each, us per lock |
|---|---|---|
| 1 | 0.31 | 0.28 |
| 3 | 2.23 | 0.46 |
| 7 | 35.25 | 0.48 |

The first exchange protocol of lib/lanes.as had every neighbor read one region per lane and took 37 us per color
stage at 4 lanes and 155 us at 8; one region per sending and receiving pair, whose lock never sees two threads,
brought it to about 7 us. A short spin before parking, as `lockWord` does, would fix the general case.

## A parked worker wakes tens to hundreds of microseconds after a notify

bench/threads/micro (`signals`, `naps`): the main lane notifies a `Signal` a worker awaits with `next`, after the
worker has parked for 2 ms, and spins until the worker acks through a region word. 200 rounds, M5 Max:

| load | signal wake, mean / max us | `Worker.sleep(0.0002)` poll, mean / max us | port round trip us |
|---|---|---|---|
| 8 | 75 / 3337 | 57 / 423 | 20.6 |
| 13 | 45 to 798 / 268 to 14944 | 79 to 533 / 259 to 13505 | |
| 40 | | | 447 |

A wake costs more than one of the solver's colored stages at 4 lanes (about 11 us of work), so the task layer
wakes its workers at the start of each hook, where the wake hides behind the main lane's serialization, and keeps
them spinning while a command is in flight.

## A trap in a lane names no function

A lane that traps reports only `worker b3LaneWorker failed: unreachable` (or `wasm call trapped in
host_obj_advance: unreachable` on the main lane), in release and debug bakes alike, so an index out of range in
lane code has no location. Debugging the solver replica took a mode that runs it on the main lane
(`replicaOnMain`, `REPLICA=1` in bench/threads) and console lines.

## A variable index into a FixedArray local compiles to a jump table

```as
export function f(p: i32, n: i32): i32 {
	let a: FixedArray<i32, 4>;
	for (let i = 0; i < 4; ++i) {
		a[i] = p + i;
	}
	let s = 0;
	for (let i = 0; i < n; ++i) {
		s += a[i & 3];
	}
	return s;
}
// rasc -O --release, and still after the -O2 binaryen pass: each access is an index check and a br_table over
// four locals
```

A fixed array local lives in wasm locals, so an index that is not a constant becomes a `br_table` and a range check
per read and per write, where C keeps the array in registers after unrolling or on the stack. Box3D's wide prepare and
store impulses kept their per lane contacts, manifolds, sims and counts in such arrays: 70 to 120 jump tables per call
and the largest share of their extra ops over C. Writing the lanes into the constraint (as C does) and reading them back
with raw loads, or unrolling the lane loop through an `@inline` helper, removed them (notes/perf_4.md, change 2). Either
rasc should place a fixed array indexed by a variable in the shadow stack, or the docs should say to keep such indices
constant.

## binaryen's offset folding needs low memory unused, and rasc's static data starts at 8

binaryen folds `(load (i32.add x (i32.const C)))` into `(load offset=C x)` only under `--low-memory-unused`
(`OptimizeAddedConstants`, for C < 1024), which emscripten passes and rasc's post pass (`postOptimizeInProcess`,
`kWasmOptFlags`) does not. Passing it to a rasc module as it is laid out today miscompiles: static data starts at
`kDataBase = 8` (compiler.h), and the frame runtime's raw allocator indexes its size class table at static address 424
with a negative offset that binaryen's reassociation produces (`(-clz(x) << 2) + 424`). With the flag the 424 moves
into the store's offset, the address no longer wraps and the store traps: `memory access out of bounds` in
`rememberAdd` on the first steps of large_pyramid.

Prototyped outside the rive worktree: a copy of rasc with `kDataBase = 1024` (the lane reset then clears the first
1024 bytes with a `memory.fill` instead of one i64 store) and `--low-memory-unused` added to the external wasm-opt run.
The Box3D web bake then runs bit exact on every scene, and the kernels lose most of their address adds (solve 190 to 32
`i32.add`, prepare 236 to 69; loads with a constant offset 4 to 154 in solve). Measured:

- V8 (node 26.3, 3 interleaved rounds): no change, every stage within 2%. TurboFan's arm64 code absorbs the adds.
- WAMR AOT, static (`wamrc --format=object`, the ladder's flags, software bounds): 10 to 20% fewer instructions in the
  kernels and 19% in `b3ShapeDistance`, mostly bounds checks: with the offsets in the immediate LLVM merges the checks of
  accesses off one base (`b3ShapeDistance` 735 to 393 compares and branches, warm start 253 to 166).
- WAMR AOT at run time: not measured, since rive-cli bakes through its in-process post pass and the prototype cannot
  reach it without a CLI change.

So the change is worth making for WAMR with software bounds checks: start static data at 1024 (one constant) and set
`lowMemoryUnused` in both post pass paths. Under hardware bounds most of the gain (the merged checks) goes away.

## binaryen does not simplify a shuffle of shuffles, and V8 lowers a non canonical one to lane moves

The port's first transposes took each column as `shuffle(shuffle(r0, r1, 0, 4, 0, 4), shuffle(r2, r3, 0, 4, 0, 4), 0, 1,
4, 5)`. binaryen keeps all three shuffles, and V8's arm64 backend has no single instruction for `0, 4, 0, 4`, so it
moves lanes one by one: 296 lane moves in the solve kernel against C's 50. LLVM (WAMR AOT) picks good instructions for
the same code. Writing every shuffle as one of the zip patterns (`0, 4, 1, 5` and `2, 6, 3, 7`), as SSE2's unpacks
compile, fixed it in source (notes/perf_4.md, change 1). A shuffle simplification in rasc or binaryen would make the
pattern a non issue.

## A StaticArray literal cannot hold structs

Found writing `examples/` on rive master b0b4628806:

```as
let points: StaticArray<b3Vec3> = [b3Vec3(0.0, 0.0, 0.0), b3Vec3(1.0, 0.0, 0.0)];
// E3001: StaticArray literals cannot hold structs; use Array or index assignments
```

The message names the workaround: `new StaticArray<b3Vec3>(n)` and one assignment per element. The C API takes point
and vertex arrays everywhere (hulls, meshes, proxies), so this is the first thing a user of the port writes.

## Atomics are only on the parallel-calls toolchain

Not a bug, a dependency: rive master's rasc has no `atomic.*` builtins (std declares them, codegen does not know them:
`unknown function 'or' in namespace`); they arrive with #15105. The parallel backend's finalize marks tree ancestors
with `atomic.or`, so `b3DynamicTree_MarkProxyMoved` takes the mode type argument and only its `b3InParallel`
instantiation uses the atomic, which keeps main building on master.
