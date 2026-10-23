# parity: end to end trajectory parity against native Box3D

The whole port, driven through `lib/box3d.as` only, replays the nine native scenes of
`notes/scenes.md` and writes the same version 1 trajectory files as `verify/scenes/traj.c`. Each
recording is compared word for word with the scalar 1 worker reference in `verify/scenes/out/`.

| file | what |
|---|---|
| `verify/scenes/scenes.as` | mirror of `verify/scenes/scenes.c`, statement for statement |
| `verify/scenes/traj.as` | wasm driver: `create(sceneIndex, workerCount)`, `step(i)`, record buffer exports |
| `verify/traj.mjs` | host: runs one scene, writes `verify/scenes/out/as/<scene>.bin`, diffs with `dump.mjs` |
| `verify/traj.sh` | builds, runs, diffs, prints the scene table |

```sh
verify/traj.sh                  # every scene but large_pyramid and many_bodies (about 30 s)
verify/traj.sh --large          # all nine (large_pyramid about 90 s, many_bodies about 50 s more)
verify/traj.sh chains mover     # a subset
RASC_FLAGS= verify/traj.sh      # without -O; WASM=path builds the driver elsewhere
NATIVE=1 verify/traj.sh         # re-record the native references first
node verify/scenes/dump.mjs diff verify/scenes/out/chains_scalar_w1.bin verify/scenes/out/as/chains.bin
```

## How the driver works

- Built with `--runtime frame -O`. The scene lives in a module global; `traj.mjs` calls `step(i)`,
  copies the record out of wasm memory through `recordPointer()` / `recordLength()` (a
  `Uint32Array`, valid until the next collection), then calls `frameBoundary()`. A minor collection
  runs between every two steps, so the world is moved by the collector throughout.
- The host writes the 16 word header exactly as `traj.c` does (worker count 1, `Math.fround(1/60)`,
  4 sub-steps, 56 counters, 112 mover words for `mover`).
- `diff` from `dump.mjs` compares bodies, the mover block and counters 0..50. Counters 51..55
  (taskCount, stackUsed, arenaCapacity, byteCount) are environment counters. In practice the files
  differ only in stackUsed, arenaCapacity and byteCount low, which read 0 in the port
  (`notes/world.md`); taskCount matches.

### Mirror details

- Constants are written as in C. Pure literal expressions use explicit casts
  (`<f32>1.0 / <f32>60.0`); a literal in an f32 expression or passed to an f32 parameter is f32.
- `b3GetLengthAndNormalize` takes its length through a `FixedArray<f32, 1>` element (rasc refuses a
  `ref` to a local).
- The mover's plane callback has no context argument, so the mover being solved sits in a module
  global, set right before `b3World_CollideMover`.
- `b3CreateRock` and `b3CreateWaveMesh` return nullable references and are unwrapped with `!`.
- `SceneBone` is a class; it never reaches the library.

## Results

Latest run 2026-10-23 13:27, HEAD `10e0ce3` plus the porters' uncommitted lib, fingerprint
`bac76e39fa6a` (`cat lib/*.as | shasum`, newest files solver.as, island.as, core.as at 13:15).

| scene | steps compared | result | first divergence |
|---|---|---|---|
| pyramid | 300 | pass | none |
| card_house | 300 | pass | none |
| chains | 300 | pass | none |
| capsule_mesh | 300 | pass | none |
| heightfield | 400 | pass | none |
| bullets | 150 | pass | none |
| mover | 300 | pass | none |
| large_pyramid | 200 | pass | none |
| many_bodies | 100 | pass | none |

Pass means every body word (valid, awake, type, id, position, rotation, velocities), every mover
word (state and all 8 plane slots) and counters 0..50 (contact, island, awake counts, tree heights,
SAT calls and cache hits, TOI iterations, events, the 24 graph color counts, manifold buckets) are
bit identical at every step. A raw word compare of `chains` shows 900 differing words out of
440716, all of them stackUsed, arenaCapacity and byteCount low (3 per step).

Also checked on the seven default scenes:

- without `-O` (`RASC_FLAGS=`): all pass;
- with `-O -D B3_ENABLE_ASSERT`: all pass, no assert fires (an assert traps through
  `b3InternalAssert`).

Negative control: a copy of `scenes.as` with card_house friction 0.7 changed to 0.70001 fails at
step 0 (body 1 `w.x`, `3b84b432` vs `3b84b430`), so the compare catches a one ulp scale change.

Wall time under node, `-O`, no binaryen pass, machine load about 25: pyramid 2.0 ms per step,
large_pyramid 195 ms, many_bodies 375 ms (native scalar 1 worker: 10.5 and 14.7 ms). Under load
40 the same run was several times slower, so treat these as rough.

## Divergences

None. Every scene is bit exact against the scalar native build, so no probe programs were needed.

## Re-runs

Lib files were still changing while this ran; each row is against the lib as it was then.

| time | lib fingerprint | scenes | result |
|---|---|---|---|
| 13:02 | `a79e3b12a909` | default seven, `-O` | all pass |
| 13:03 | `a79e3b12a909` | default seven, no `-O`; with `-D B3_ENABLE_ASSERT` | all pass |
| 13:05 | `a79e3b12a909` | large_pyramid, many_bodies | both pass |
| 13:27 | `bac76e39fa6a` (body, core, island, solver changed) | all nine | all pass |

Re-run with `verify/traj.sh --large` after lib changes and add a row.

## Harness findings

- `process.exit` in node 26.3.1 hung for 20 minutes once, after the run had finished and written its
  file: the main thread was joining a V8 Maglev compile worker stuck allocating. `traj.mjs` sets
  `process.exitCode` and lets the event loop drain instead. Other drivers call `process.exit`
  through `runModule` in `verify/lib.mjs` and could hit the same hang on long runs.
- The driver imports `rive_rt_v1.now`, which the harness stubs to 0 (profiling timers read 0 as
  in `notes/world.md`).
- Editing `traj.sh` while it runs breaks the running copy, since bash reads scripts as it goes.
