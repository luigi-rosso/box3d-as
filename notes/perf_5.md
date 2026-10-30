# Fifth perf pass: the large scene gap

2026-10-30, rive master dd800e6a83 (rive-cli release, rasc, and a wamrc with every patch through 0059), the SIMD
artifact on the hardware bounds AOT tier, against native Box3D at f36d061 with NEON.

Counts come from the script thread, not the wall clock. A local rive-cli change (not on master) makes `clock()` read
`thread_selfcounts`, so the bench's step and stage times read as thousands of retired instructions or cycles, and
the native bench links a timer that does the same. Instruction counts repeat to 0.1% at any load; cycles still
depend on core type and contention, so each table says what load it ran under.

## The gap on a quiet machine

Load 4 to 10, median of three interleaved rounds, ours over native:

| stage | large_pyramid instructions | large_pyramid cycles | pyramid instructions | pyramid cycles |
|---|---|---|---|---|
| step | 1.07 | 1.04 | 1.11 | 1.12 |
| collide | 0.97 | 1.04 | 0.98 | 0.85 |
| pairs | 1.06 | 0.75 | 1.13 | 1.32 |
| prepare | 1.25 | 1.11 | 1.26 | 1.25 |
| warm start | 1.13 | 1.02 | 1.19 | 1.19 |
| solve | 1.06 | 1.06 | 1.09 | 1.13 |
| relax | 1.02 | 1.05 | 1.05 | 1.09 |
| store impulses | 1.50 | 1.29 | 1.45 | 1.31 |
| integrate positions | 1.13 | 1.22 | 1.14 | 1.18 |
| transforms | 1.12 | 1.07 | 1.13 | 1.19 |

The 2x collide of earlier sessions was the machine's load (100 to 180), not the port.

## Under contention collide degrades twice as fast as C's

Six processes copying 64 MB buffers stream through the shared caches. large_pyramid, cycles per step:

| | native | ours |
|---|---|---|
| step | 29.6M (1.9x quiet) | 34.6M (2.2x quiet) |
| collide | 5.6M (2.4x quiet) | 10.3M (4.2x quiet) |

Collide is 4.7M of the 5.0M difference; the solver stages slow down alike on both sides, store impulses a little more
on ours (3.2M against 2.5M).

- **Not the layout.** A probe over every live contact: ours are contiguous at a 224 byte stride (C: 216), and one
  pass over the contacts, their manifolds and both shapes touches 147k cache lines and 600 pages (C: 130k and 555).
- **Not the front end.** Instruments' CPU Counters during the steps: ours 2.7% delivery, 75.1% processing, 20.9%
  useful; native 1.9%, 71.4%, 23.8%. Both wait on memory.
- **Serialized misses.** C finds a contact or shape from its index; we load its address from an array slot first, and
  the loop body is too long for the core to start the next contact's loads early. Loading the next contact's lines
  ahead confirmed it.

## Change: collide loads the next contact first

`b3CollideTask` loads four lines of the next contact, three of each of its shapes, four of its manifold and the first
line of each awake body sim before it updates the current contact, and the first line of the contact after that. The
loads change nothing the step computes; their sum lands in `b3TaskContext.collideAhead` so the compiler keeps them.

- Instructions per step: large_pyramid and many_bodies +1.9%, village +1.7%, pyramid +1.6%, mesh_drop +0.4%,
  chains +0.1%.
- Quiet, large_pyramid, three rounds: collide 3.62 to 3.78M cycles against 3.89 to 4.70M, the step 1 to 7% faster.
- Contended: collide 17% faster in the first sessions (12.1M to 10.1M), the step 3%.
- pyramid's minimum step is unchanged (169.5k cycles).

Gates: `verify/run.sh` 29 of 29 and `verify/traj.sh --large` 13 of 13 on both artifacts, every bench hash
unchanged.

## Tried and dropped

- **The same for prepare and store impulses.** Faster under contention, but slower quiet: prepare up to 13% and
  store impulses 20%. Their loops are short enough that the extra loads cost more than the overlap saves.
- **A struct array copy of what collide reads from shapes.** The layout probe shows nothing to gain.

## What is left

- **Store impulses, integrate positions and prepare.** Mostly wasm32 addressing: each access adds the memory base to
  a 32 bit offset, and offsets advance in 32 bits because wasm's adds may wrap, so a manifold point in store impulses
  takes ten instructions where C's post incremented pointers take six.
- **The frame GC.** At every frame boundary the frame runtime visits all 160 of the bench module's globals, about 1.5%
  of a pyramid frame. That is outside the step and the same for any script with that many globals.
