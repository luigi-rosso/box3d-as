# tree: the dynamic AABB tree

`src/dynamic_tree.h` + `src/dynamic_tree.c` -> `lib/dynamic_tree.as`, functions
in C order. Types (`b3TreeNode`, `b3TreeProxy`, `b3DynamicTree`,
`b3TreeStats`, the callback aliases, `b3DynamicTree_GetUserData`,
`b3DynamicTree_GetAABB`) were already in `lib/collision.as`; the SIMD helpers
(`b3OverlapNode`, `b3UnionPairV`, `b3StoreAABBV`, `b3UnionV`,
`b3TestBoundsRayOverlap`) come from `lib/simd.as`.

## Ported

Everything except Save/Load:

- header: `B3_TREE_STACK_SIZE`, `B3_MOVED_NODE`, `B3_LEAF_NODE`,
  `B3_NODE_INDEX_MASK`, `B3_EMPTY_NODE`, `B3_ROOT_NODE`, `b3IsLeaf`,
  `b3IsNodeMoved`, `b3IsEmptyNode`, `b3GetLeftChild`, `b3GetProxyId`,
  `b3GetRootPair`, `b3MakeEmptyNode`, `b3TreeView`, `b3MakeTreeView`,
  `b3HasTreeMoved`, `b3NeedsRebuild`
- create/destroy, the sibling pair allocator and free list, the proxy pool and
  its growth, node pool growth (drops the spare as C does)
- insertion: SAH sibling search, the rotations, removal, move (no rotations),
  enlarge, set/get category bits
- queries: AABB query, closest query, ray cast, box cast, through the tree and
  through a `b3TreeView` with or without proxies
- rebuild, full and partial: gather of build leaves, median split partition,
  bump allocated DFS build, retained subtree copy, array swap
- moved flags: `MarkProxyMoved`, `MarkProxyMovedSerial`, `ClearMoved`,
  `GatherMovedProxies`, `Refit` (sweep and the unordered subtree refit)
- height, area ratio, root bounds, proxy count, byte count, `Validate`,
  `ValidateNoMoved`

## Skipped

- `b3DynamicTree_Save`, `b3DynamicTree_Load` and `b3OpenTreeFile` (file IO).
- The SAH partition (`b3PartitionSAH`, `b3TreeBin`, `b3TreePlane`). C pins
  `B3_TREE_HEURISTIC` to 0 with an unconditional `#define`, so only the median
  split is compiled and that is what the port has. `leafBoxes` and
  `binIndices` stay empty arrays, as C leaves them NULL, while
  `b3DynamicTree_GetByteCount` still counts them by `rebuildCapacity`, as C
  does.

## Signatures that differ from C

```as
// node helpers take the node by value; pass nodes[i] or a ref binding
b3IsLeaf(node: b3TreeNode): bool            // also IsNodeMoved, IsEmptyNode, GetLeftChild, GetProxyId
b3GetRootPair(nodes: StaticArray<b3TreeNode>): i32

b3DynamicTree_Create(proxyCapacity: i32): b3DynamicTree     // a new class instance
b3DynamicTree_Destroy(tree): void                           // resets every field, the memset

b3DynamicTree_Query(tree, aabb, maskBits: u64, requireAllBits: bool, callback: b3TreeQueryCallbackFcn): b3TreeStats
b3DynamicTree_QueryClosest(tree, point, maskBits, requireAllBits, callback: b3TreeQueryClosestCallbackFcn,
                           minDistanceSqr: ref f32): b3TreeStats
b3DynamicTree_RayCast(tree, input: b3RayCastInput, maskBits, requireAllBits, callback: b3TreeRayCastCallbackFcn): b3TreeStats
b3DynamicTree_BoxCast(tree, input: b3BoxCastInput, maskBits, requireAllBits, callback: b3TreeBoxCastCallbackFcn): b3TreeStats
b3TreeView_Query / _QueryClosest / _RayCast / _BoxCast(view: b3TreeView, ...same...)

b3DynamicTree_GatherMovedProxies<A>(tree, proxyIds: A): i32   // any i32 array with room
```

- Callbacks drop `void* context` and the closure carries it (CONVENTIONS).
  Ray and box casts hand the callback a copy of the clipped input struct.
- `minDistanceSqr` is in and out. rasc binds a `ref f32` to a struct leaf or a
  FixedArray element but not to a plain local, so callers write
  `let m: FixedArray<f32, 1>; m[0] = start; b3DynamicTree_QueryClosest(..., m[0]);`.
- `b3TreeView` is a class `{ nodes: StaticArray<b3TreeNode>; proxies: StaticArray<b3TreeProxy>? }`,
  built with `new b3TreeView(nodes, proxiesOrNull)`. A null `proxies` takes C's
  `hasProxies == false` path: no category test, and user data is the leaf's
  `shapeIndex`. The tree functions pass the arrays to the shared traversal
  directly, so a tree query does not allocate a view.
- `b3PartitionMid` takes the leaf arrays plus a base index where C takes
  offset pointers.

## Port details

- The C union `height`/`shapeIndex` is two fields in our `b3TreeNode`. Every
  node in dynamic_tree.c is built whole by `b3MakeLeafNode`,
  `b3MakeInternalNode` or `b3MakeEmptyNode`, so the port keeps the unused
  member at zero: a leaf has `height == 0`, an internal node
  `shapeIndex == 0`. C's union value is `isLeaf ? shapeIndex : height`. The
  validator asserts this and the parity driver counts violations (0).
  Anyone writing nodes elsewhere (compound, mesh) must keep the same rule.
- Stacks: C keeps 512 entry stacks on the C stack. A FixedArray local is capped
  at 255 elements and 1024 leaves, so they are heap arrays. The outermost
  query or cast uses a module static stack and a query nested inside a callback
  allocates its own (a depth counter), so callbacks that query again work, as
  the parity driver checks. A callback that traps leaves the counter raised and
  every later query allocates. The rebuild, copy, moved flag and refit walks
  never call out and share three module static stacks. Those are per module
  instance, so two lanes need two instances.
- `B3_GROW_ZERO` becomes a new zeroed `StaticArray` plus a copy; a NULL
  pointer is a length 0 array (`swapNodes.length == 0` means no spare).
  The rebuild arrays come from `new StaticArray` (zeroed) where C uses
  uninitialized `b3Alloc`, so they match C only up to the last leaf count,
  which is everything C itself reads.
- `MarkProxyMoved`: the atomic load and fetch-or become one check and a plain
  or, which is the same thing single threaded.
- `GetByteCount` uses `sizeof( b3DynamicTree ) == 112` and the C element sizes
  (node 32, proxy 24, AABB 24, vec3 12) as constants, so it reports C's bytes.
- Validation: C builds it out (`B3_ENABLE_VALIDATION` 0), and core.as has the
  constant false. To exercise our validator anyway, dynamic_tree.as adds
  `-D B3_VALIDATE_TREE`, which turns on `Validate` and `ValidateNoMoved` (with
  `-D B3_ENABLE_ASSERT`). `verify/as/tree.flags` sets both, so every rebuild
  in the parity run validates the whole tree.
- Hot asserts are `if (B3_ASSERT_ENABLED) B3_ASSERT(...)`.

## Verification

`verify/run.sh tree`: **546056 lines, 4364118 values, 0 mismatches**. The
same result with `RASC_FLAGS=` (no `-O`) and, compiled by hand, without
`tree.flags` (no asserts or validation) at `-O` and without it. `rasc check
--strict` is clean on `lib/dynamic_tree.as` with and without the defines.

The driver (`verify/cref/tree.c`, `verify/as/tree.as`, `verify/tree.mjs`)
keeps the tree in a module global and runs a frame collection between
phases, so the collector moves the tree's arrays mid run.

- Scripted: capacities -5 to 100; the empty tree (every query, rebuilds, refit,
  clear, gather); a single proxy at the root through rebuilds, mark, enlarge,
  refit, clear and destroy; a row of six with removals, pair reuse, partial and
  full rebuild; pool growth from capacity 1 to 60 proxies with rebuilds between
  (the spare is dropped on node growth), then shrink and regrow; 33 identical
  boxes (every cost and partition ties); a compound style build.
- 12 seeded rounds of 10 phases over 6 box shapes (uniform, integer grid with
  many equal boxes, degenerate flat and point boxes, a line, far coordinates
  near 1e5, identical boxes), up to 852 live proxies and height 47. Random
  create (public and internal, marked or not), destroy, move (both),
  enlarge, mark, serial mark, set category, refit, rebuild (full and partial),
  clear, gather and queries, with a solver style refit and partial rebuild
  every third phase and a full rebuild at the end.
- Per operation: an FNV hash over every live node, parent and proxy plus the
  counts (28618 ops). After each phase and scripted step: the whole tree
  (241 dumps), every node and parent up to capacity (151188 lines), the spare
  (135124), every proxy (74834), the rebuild leaf indices, nodes and centers
  of the last rebuild (16590), stats (height, area ratio, root bounds, count,
  bytes, moved, needs rebuild).
- Queries (4639) in callback order with their stats: AABB queries with mask
  and require-all filters and early stop (11049 hits), closest queries with
  the running minimum per callback and the final minimum (8380), ray casts
  with clipping, skip, terminate and a real AABB ray cast policy, axis
  aligned and zero rays (1836 hits with input and fraction), box casts (3874).
  A quarter of each run nested queries inside the callback (90653 nested AABB
  hits, 8933 nested closest) to cover reentrancy. Paths are split between the
  tree API, a view without proxies and a view with proxies.

One driver rule came out of the validator: `ClearMoved` right after
`MarkProxyMoved` with no `Refit` leaves ancestors that no longer contain
their children and no longer carry the moved mark, and the next partial
rebuild keeps them. C produces the same tree (the parity matched with
validation off), so this is a contract of the API, not a port issue: the
solver always refits first. The driver's clear op refits before clearing.

## rasc findings

### Module constants are never folded under -O

```as
export const A: u32 = 1 << 31;
const E = 7;
@inline const F: u32 = 0x80000000;
export function f(x: u32): u32 { return (x & A) | <u32>E | (x & F); }
// --wat -O: every constant is a global.get of an immutable global, typed,
// untyped, exported or @inline alike; imported ones too
```

asc inlines constant globals. Here every `B3_LEAF_NODE` test in the tree
traversal loads a global. Engines usually treat immutable globals as
constants, so the cost is likely small, but a wasm-opt pass or constant
propagation in rasc would remove it. Not worked around.

### Already known, seen again

- Struct returning helpers (`b3LoadAABBV`, `b3AABB_Union`, `b3MakeEmptyNode`)
  stay calls under `-O`.
- `StaticArray<T> | null` is refused ("union types are not supported"); the
  nullable spelling `StaticArray<T>?` works.
- `ref i32` binds a `FixedArray<i32, 1>` element, not a local; the validator's
  leaf count and the closest query's minimum use that.
- `FixedArray` is capped at 255 elements, so C's 512 entry stack arrays cannot
  be locals.
