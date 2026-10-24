# perf 3: prepare, continuous, mesh and compound collide, and what WAMR leaves

Scope: all of `lib/`. Commits 0b9c992 (start) to 3e5eaf2. Every commit kept `KEEP_GOING=1 verify/run.sh` (29
modules) and `verify/traj.sh --large` (13 scenes) bit exact with `RASC_FLAGS=-O` and with
`RASC_FLAGS="-O --enable-simd"`. The bench state hash matches the native recording on every tier (SIMD and
scalar artifacts, AOT and interpreter) for pyramid, large_pyramid, many_bodies, chains and plank_pile, and the
query hashes match native in village and queries. No run grows the heap. The v128 cell check stays below 240
(prepare 225, solve 238, push 237, the new mesh prefilter 197), so the SIMD artifact still runs on the
interpreter.

## Results

**Wall clock**, `cmp.py` session in the scratchpad: 4 rounds (SIMD artifact) and 3 rounds (scalar artifact), each
round running native, base (0b9c992) and final (e880a9b, before the last commit) one after the other per scene,
min of means over rounds, ms per step (the b3Profile step; the queries scene's queries are outside it). AOT -O3,
release bakes, one worker, Apple M5 Max, one minute load 11 to 21. Back to back native runs in the same session
differed by up to 25%, so read differences under about 20% as noise; the cycle table below is the better guide.

| scene | native SIMD | base SIMD | final SIMD | final x native SIMD | native scalar | base scalar artifact | final scalar artifact | final x native scalar |
|---|---|---|---|---|---|---|---|---|
| large_pyramid | 4.953 | 7.497 | 5.275 | 1.07 | 8.155 | 11.74 | 13.66 | 1.67 |
| many_bodies | 8.712 | 11.07 | 11.53 | 1.32 | 16.15 | 22.59 | 22.76 | 1.41 |
| pyramid | 0.0344 | 0.0689 | 0.0613 | 1.78 | 0.0583 | 0.1397 | 0.126 | 2.16 |
| chains | 0.0443 | 0.1095 | 0.0902 | 2.04 | 0.0613 | 0.1094 | 0.1072 | 1.75 |
| plank_pile | 0.982 | 1.151 | 1.018 | 1.04 | 1.023 | 1.283 | 1.056 | 1.03 |
| mesh_drop | 6.132 | 9.993 | 10.17 | 1.66 | 7.623 | 14.14 | 12.35 | 1.62 |
| village | 2.857 | 4.29 | 4.075 | 1.43 | 3.294 | 6.027 | 6.186 | 1.88 |
| queries | 0.015 | 0.035 | 0.0371 | 2.47 | 0.0217 | 0.0424 | 0.0403 | 1.86 |
| capsule_mesh | 0.1206 | 0.1912 | 0.1861 | 1.54 | 0.1625 | 0.2871 | 0.253 | 1.56 |
| heightfield | 0.0776 | 0.118 | 0.1092 | 1.41 | 0.1189 | 0.1811 | 0.1938 | 1.63 |
| mover | 0.0005 | 0.0035 | 0.0035 | 7.00 | 0.0008 | 0.004 | 0.0041 | 5.12 |
| card_house | 0.0108 | 0.0201 | 0.0184 | 1.70 | 0.0173 | 0.0284 | 0.0321 | 1.86 |
| bullets | 0.0055 | 0.0186 | 0.0176 | 3.20 | 0.008 | 0.0228 | 0.0212 | 2.65 |

Interpreter, pyramid, 3 rounds: SIMD artifact 1.022 (base) and 1.049 ms (final); scalar artifact 2.21 and
1.96 ms.

In a quieter hour (load 9 to 13, 3 rounds, the tree at 76e6cfe, before changes 4 to 10) the same comparison
gave large_pyramid 4.70 against native SIMD 3.55 (1.32x), many_bodies 9.01
against 6.70 (1.35x), mesh_drop 6.85 against 4.74 (1.44x), village 2.96 against 1.99 (1.49x).

**Cycles**, the load resistant view: whole run CPU Profiler traces of base and final, each function group's
cycles divided by the cycles of the relax and push kernels in the same trace (which this pass changed only
by the alignment padding), final over base:

| group | large_pyramid | many_bodies | mesh_drop | village |
|---|---|---|---|---|
| prepare | 0.78 | 0.73 | 0.72 | 0.74 |
| store impulses | 1.21 | 1.05 | 0.86 | 0.99 |
| solver kernels (all) | 0.93 | 0.91 | 0.90 | 0.92 |
| integrate and dispatch | 0.80 | 0.89 | 1.02 | 0.99 |
| finalize and continuous | 1.05 | 0.89 | 0.71 | 0.77 |
| collide | 1.12 | 0.94 | 0.80 | 0.74 |
| broad phase and pairs | 1.09 | 1.19 | 0.98 | 0.91 |
| host calls and runtime | 1.04 | 0.90 | 0.87 | 0.84 |
| whole step | 0.97 | 0.92 | 0.82 | 0.83 |

Groups this pass did not touch (store impulses and collide in the pyramids, broad phase) moved by up to 20%
between traces, which bounds the method's noise. Against native (shares of each trace, same method), the
largest remaining excess groups are host calls (6 to 8% of the AnimaScript step, 0.1 to 0.8% of native's),
finalize and continuous (6% of the step against 3% native on the pyramids) and integrate and dispatch.

## How it was measured

The machine was shared all session: one minute load 8 to 65, other agents running multi threaded benches, four
niced CPU bound leftovers in the scratchpad (`coplanar`, from an earlier session) and a spinning `BTLEServer`.
Back to back runs of the same native binary differed by 25%, so three kinds of evidence are used:

- Interleaved wall clock A/B of two snapshots of the tree (a scratchpad `cmp.py`: each round runs every scene
  for every label in turn, native included, min of means over rounds). Good for changes of 15% or more.
- Whole run CPU Profiler traces (cycle sampled, `xctrace --template "CPU Profiler"`, attached during a spin
  before the first step so every step is covered), symbolized with `bench/as/tools/aotsym.py`. A function's
  cycles divided by the cycles of solver kernels the change does not touch (solve, push, warm start) cancel
  most of the load; prepare, untouched in those runs, moved by 2 to 5% between runs, which is the noise of
  that method.
- Static counts on the release bake through `wamrc --format=object` with the ladder's flags: instructions,
  and the memory base and bound reloads (`ldr x, [x19, #0x178]` / `[x19, #0x1b8]`) the WAMR finding below
  is about.

## Changes, in commit order

### 1. Prepare gathers rows with v128 loads (f205ae2)

C's SSE2 and NEON prepare loads each lane's inverse inertia, manifold header (normal, twist, friction and
rolling impulse) and point (anchors, separation, normal impulse) as rows of four floats and transposes them.
The port gathered every float with its own load and lane insert, each with a software bounds check. The SIMD
kernel now does what C does through `b3GatherXL` ... `b3GatherWL` (lib/contact_solver_wide.as), which load
four v128 rows and transpose with shuffles; the scalar artifact's f32 kernel reads the same addresses one
float at a time as before. `b3Contact` keeps friction, restitution and rolling resistance next to each other
(a port detail; C has a union between them) so they are one row too, and the tangent velocity is gathered
the same way. The loads per wide constraint roughly halved.

Interleaved A/B, 3 rounds: large_pyramid prepare 0.659 to 0.485 ms (-26%), many_bodies 1.599 to 1.227 ms
(-23%), with the rest of the step unchanged. A CPU profile put prepare at 18.4% of large_pyramid's script
samples before and 12.5% after.

Reloading the two tangents after the point loops instead of keeping them live (dad7419) keeps prepare's last
v128 local at frame cell 225; the gathers alone had pushed it to 245.

### 2. Continuous collision through addresses (dad7419)

`b3SolveContinuous` allocated a `b3ContinuousContext` (with a 252 byte sim copy inside) and a capturing
closure per fast body, copied the fast body's sim out of the set and back, and the query callback copied a
whole sim per candidate shape through `b3GetBodySim`. It now reads and writes the fast body's sim through its
address as C's pointer does (`FSIM_` offsets), reuses one module context, passes a plain tree callback, and
builds the candidate's sweep from the sim's address (`b3GetBodySimAddress`, `b3MakeRelativeSweepAt` in
body.as). mesh_drop's transforms stage (finalize plus continuous) went from 3.16 to 2.35 ms (-26%) in an
interleaved A/B.

### 3. Body rows and the mesh prefilter on v128 (76e6cfe)

`b3StoreRow` (lib/simd.as) writes four floats as one v128 store under SIMD and four f32 stores without; lanes
only move bits, so both artifacts write the same memory, and a lane that covers the next field rewrites its
current value. Finalize writes the center with the sim flags, the rotation, the zeroed delta position with the
new state flags, the identity delta rotation, the position with the first rotation lane, the move event's
transform, the zeroed force with the first torque lane, center0, rotation0 and the world inverse inertia as
rows; the integrations write the velocities and the whole state the same way. Transforms -4 to -8%, integrate
velocities -10%, integrate positions -9 to -11% (large_pyramid and many_bodies, 3 rounds).

The mesh contact prefilter transforms four triangles at a time and culls them against the hull bounds. Its
`b3FloatW4` structs are four scalars in the port; under SIMD it now runs on v128 lanes with the exact forms of
notes/containers.md (min and max as compare plus bitselect, the move mask as `i32x4.bitmask` of the compare
masks). The lanes would have put v128 locals past frame cell 256 in `b3ComputeMeshManifolds`, which traps
WAMR's interpreter, so the prefilter is a function of its own, `b3PrefilterTriangles`; it is called once for
meshes and once for height fields, because the release bake inlines any function with one caller (finding
below). mesh_drop collide -7%, capsule_mesh -22%, heightfield -4%.

### 4. Mesh manifolds copy with memory.copy (f5a4129)

The copy of the old manifolds and the zeroing of reused ones were per field struct copies (67 floats each);
they are one `memory.copy` and one `memory.fill`, as C's memcpy and memset.

### 5. Compound child and manifold point rows (42aacc2)

C copies the whole parent shape into the temporary child shape. The port copied every field, eight of them
references through the write barrier. It now copies what the child paths read: id, generation, flags,
materials and their count, plus the child type and geometry it already set. Manifold points are written as
rows (anchors with the separation and normal impulse; the impulse, normal velocity, base separation and
feature id), and the flip, compound offset and center of mass adjustments rewrite the anchor rows.
`b3ComputeConvexManifold` matches each point's impulse right after storing it, where C stores every point
first; a point's match reads only its own feature id and the old points, so the result is the same. village
collide -5%.

### 6. GJK without calls (dc29dd5)

`b3ShapeDistance` called `b3BarycentricCoordsTri` (five sites, a four float multi value return),
`b3ComputeWitnessPoints` and `b3GetMetric`. Any call makes WAMR reload the memory base and bound after every
store (finding below), and the simplex lives in memory, so the function had 1125 such loads in 6121
instructions. The two small helpers are `@inline` now and the barycentric coordinates are written out at their
five sites; the function makes no calls and compiles to 4023 instructions with 2 reloads. Queries scene,
interleaved A/B, 4 rounds: shape casts 0.114 to 0.084 ms (-27%), closest rays -15%, overlaps -17%.

### 7. Found edges without zeroing (e39e5ea)

`let foundEdges: b3FoundEdges` in `b3ComputeMeshManifolds` zeroed 64 u64 keys and 64 i32 keys in linear
memory on every call (the structs are passed by `ref`, so they live in the shadow stack), each store followed
by the base and bound reload: 330 instructions per call, 17.6% of the function's samples in one profile. C's
stack arrays are not initialized and only the counts are reset; the sets are module statics now.
`b3ComputeMeshManifolds` went from 13383 to 12360 instructions and -17% in cycles relative to the solver
kernels (mesh_drop, two whole run profiles).

### 8. No allocation in time of impact, shape cast and contact creation (4fa83ac)

`b3TimeOfImpact` allocated a `b3DistanceInput` (three objects) per call and a `b3SeparationFunction` per outer
iteration; `b3ShapeCast` a distance input per call. They reuse module objects now (neither nests).
`b3CreateContact` allocated a new `b3Contact` (with its mesh contact and cache array) for every contact, where
C resets the slot in place; it now resets the slot's object (`b3ResetContact`), and destroying a mesh contact
empties its triangle cache instead of allocating a new one. Creating a compound contact no longer builds the
child to read its type (`b3GetCompoundChildType`). Relative to the solver kernels, mesh_drop's time of impact
-10% and continuous callback -20% in cycles.

### 9. Wide structs on 16 bytes (e880a9b)

`b3ContactConstraintW4` was 420 bytes and `b3ContactManifoldW4` put its points 4 bytes off a 16 byte boundary,
so most v128 accesses of the iteration kernels were misaligned and a quarter of them crossed a cache line.
Padding (C's SIMD builds align these types to 16) makes every wide field 16 byte aligned; the allocator already
aligns payloads to 16. Kernel cycles relative to the collide functions: solve -5%, prepare -6%, warm start
-10%, push +1%; within the noise of the method, kept because it is C's layout.

### 10. Compound mesh children built once (3e5eaf2)

A contact update against a compound's mesh child allocated a `b3CompoundMesh`, a `subarray` view of the
material indices and a `b3Mesh`. The compound now builds the mesh objects and views once
(`b3BuildCompoundChildMeshes`, at creation, and lazily for hand built compound data);
`b3GetCompoundChildInto` hands them out. village: `b3GetCompoundChildInto` -40% in cycles.

## What did not help

- **The v128 store impulses** (each wide row loaded once for the four lanes, lanes extracted at the stores):
  +30 to +40% in cycles relative to the kernels on large_pyramid and many_bodies. LLVM extracts every lane up
  front and spills them. Reverted twice (once in a loaded A/B, once in the cycle method).
- **A call free function for the integrations** (`b3IntegrateBodiesTask`, two call sites to keep it out of
  `b3ExecuteBlock`): no change in cycles. After the row stores the integrations make few stores, so the
  reloads cost little there.
- **A call free function for contact recycling** in the collide task: no measurable change; the recycle path
  waits on the contact, its sims, AABBs and manifold, not on instructions.
- **Rasc `@inline` for the barycentric coordinates** (a scalar returning form with an element index): rasc
  does not inline `@inline` functions with struct parameters (finding below), so it stayed 20 calls.

## Target 1: contact and manifold layout

The port's contacts are objects in `world.contacts` (one 4 byte reference per id) where C has an array of
structs, and each contact's manifolds are their own `StaticArray`, where C uses its block allocator. Measured
before restructuring anything:

- Store impulses run twice in a row (the second pass hits warm caches): the second pass cost 55 to 56% of the
  first in AnimaScript and 52 to 66% in native C built the same way (`STORE_TWICE` in a scratch build of the C
  sources), so both are memory bound to the same degree.
- Compacting every contact's manifold array in contact id order at step 50 of many_bodies (new arrays, which
  the collector then promotes together) changed store impulses by -3% and prepare by -5% over the next 20
  steps, within the noise.
- Under the frame collector, contacts created together are promoted together in Cheney order, so a settled
  scene's contacts already sit in id order, close to C's array.

So the layout is not where prepare and store impulses lost to C; prepare's instructions were (change 1), and
store impulses' remaining 1.5 to 2x is instruction overhead, mostly WAMR's (below). The full restructure
(manifolds in a pool with offsets, solver fields in flat arrays) was not done.

## Findings (in notes/findings.md)

- WAMR AOT reloads the memory base and bound after every store in any function that makes a call
  (`mem_space_unchanged` in aot_llvm.c). Hot store heavy code should live in call free functions.
- Binaryen inlines every function with one caller and rasc has no never inline; a second call site is the
  only way to keep a call free helper out of line.
- rasc -O ignores `@inline` on functions with struct parameters.
- The host clock (`rtNowImpl`) calls getenv on every `clock()`: about 17% of the mover scene's samples.

## What is left, and why

Measured on the final tree unless noted:

- **Software bounds checks.** The tree at 76e6cfe on rive-cli's hardware bounds build (`out/release_hw`, guard pages,
  `--bounds-checks=0`) ran large_pyramid 6%, many_bodies 6%, mesh_drop 8% and village 6% faster in one
  interleaved session, and prepare 10%, relax 7%, warm start 5%. Each v128 load and store in the kernels is an
  add, a compare and a branch (169 compares in the 1572 instructions of `b3SolveContactsL<v128>`, 122 in the
  900 of warm start). WAMR has no check hoisting across accesses that share a base.
- **Memory base and bound reloads in functions with calls** (finding). In the final bake they are 8.7% of
  `b3ComputeMeshManifolds`' instructions (1077 of 12360), 9% of `b3UpdateConvexContact`, 7.6% of the collide
  closure, 8% of the finalize closure, 9.4% of `b3World_Step`. These functions call user callbacks, the GJK,
  SAT and clipping routines, so they cannot be made call free; where a function could be (the GJK) it got 27%
  faster.
- **Code generation in the kernels.** Under hardware bounds the kernels are still 1.1 to 1.2x native SIMD:
  prepare keeps the bound in a stack slot and reloads it before every check, solve has about 50 v128 spill
  instructions, and every call between AOT functions goes through WAMR's entry stub and an exception flag test
  after it (`ldrb w8, [x19, #0x68]; cbnz`). Native NEON code has none of these.
- **Missing SIMD lane stores in rasc.** `v128.store64_lane`, `v128.store32_lane`, `v128.load_lane` and
  `i64x2.extract_lane` are "unknown simd builtin" in rasc. With them, scatters of lane values (store impulses,
  the hit event velocities) could write two floats per store; without them a lane needs an extract and a scalar
  store, and the v128 store impulses attempt lost to the scalar one.
- **The host clock.** `rtNowImpl` calls getenv per `clock()` (finding): 1.1 to 2.3% of the four big scenes'
  cycles in `__findenv_locked` and 3 to 6% in host calls overall, and most of the mover and bullets scenes'
  excess (7x and 3.2x native SIMD for 2 and 15 µs steps).
- Not done, inside our source: joints (chains is 2x native; `b3PrepareJoint` has 948 reloads in 8118
  instructions and calls the per type prepare functions), the overflow color kernels inlined into
  `b3SolverTask` (one caller each, so they inherit its reloads; plank_pile is at native), the per step closures
  and contexts the task call sites allocate (left alone for the threading work), and a slimmer reset of the mesh
  time of impact context (0.5% of mesh_drop).
