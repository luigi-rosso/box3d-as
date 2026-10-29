# web: native Box3D compiled to wasm against the port, on V8

What this measures: how the port does on the web against native Box3D on the web. Box3D's own C, compiled to
WebAssembly with emscripten, and the port's release bake run the same scenes on the same JS engine, single
threaded and threaded, with native ARM64 Box3D beside them. Everything here is in `bench/web/`.

Session: 2026-10-26, Apple M5 Max (18 cores, 6 + 12), macOS 26.5.2. node v26.3.1 with V8 14.6.202.34-node.20, and
headless Chrome 155.0.8059.86 (playwright-core 1.64.1, Chrome channel, cross origin isolated). The machine was
shared: the 1 minute load was 4 to 8 in the single threaded sessions and 4 to 15 in the threaded ones. Every
session interleaves its implementations and takes the min over rounds.

## Parallel calls on master (2026-10-28)

Parallel calls landed on rive master as 4a861ff147 (#15105), with the WebKit, team size and isolation fixes the
branch sessions below asked for. Everything here is built from that master: rive-cli and rasc, librive's web scripting
harness (`out/wasm_threads_release`, as below), and the JS runtime's `webgl2_threads` and `webgl2_advanced` flavors
(`packages/runtime_wasm/wasm/build_flavor.sh`, `-m` being the threads flag), packed as `@rive-app/webgl2-threads`
with `js/build.sh -s -r webgl2,webgl2-threads`. C's pthreads build is emsdk 4.0.23's (the tables below used 6.0.13).
Headless Chrome 156, Firefox 157 and WebKit 27.2, Apple M5 Max, load 7 to 8.

One change to the harness page: master sizes a page's team with `Module.riveScripting.parallelParts` (the JS runtime's
`RuntimeLoader.setParallelParts`), one part per core up to 8 by default, and no longer from the job pool, so
`lanes.html` takes `parts` and `parallel.mjs` passes the part count there. With `jobThreads` alone every call ran on 8.

ms per step in Chrome, median from step 1, min over 3 rounds interleaved with C's pthreads build at the same worker
count, a fresh page per run; 108 of 108 runs matched native on their final hash (`parallel.mjs`, log
`parallel_bench_chromium_20261029T030440.jsonl`):

| scene | parts | C pthreads | AS parallel calls | AS / C | C speed up | AS speed up |
|---|---|---|---|---|---|---|
| large_pyramid | 1 | 3.700 | 3.805 | 1.03 | 1.00x | 1.00x |
| large_pyramid | 2 | 1.970 | 2.050 | 1.04 | 1.88x | 1.86x |
| large_pyramid | 4 | 1.115 | 1.250 | 1.12 | 3.32x | 3.04x |
| large_pyramid | 8 | 0.975 | 1.100 | 1.13 | 3.79x | 3.46x |
| many_bodies | 1 | 7.895 | 8.385 | 1.06 | 1.00x | 1.00x |
| many_bodies | 2 | 3.930 | 4.240 | 1.08 | 2.01x | 1.98x |
| many_bodies | 4 | 2.150 | 2.450 | 1.14 | 3.67x | 3.42x |
| many_bodies | 8 | 1.595 | 1.825 | 1.14 | 4.95x | 4.59x |
| mesh_drop | 1 | 4.815 | 5.715 | 1.19 | 1.00x | 1.00x |
| mesh_drop | 2 | 2.585 | 3.200 | 1.24 | 1.86x | 1.79x |
| mesh_drop | 4 | 1.505 | 1.880 | 1.25 | 3.20x | 3.04x |
| mesh_drop | 8 | 1.230 | 1.560 | 1.27 | 3.91x | 3.66x |
| village | 1 | 2.825 | 3.160 | 1.12 | 1.00x | 1.00x |
| village | 2 | 1.720 | 2.010 | 1.17 | 1.64x | 1.57x |
| village | 4 | 1.125 | 1.360 | 1.21 | 2.51x | 2.32x |
| village | 8 | 1.090 | 1.375 | 1.26 | 2.59x | 2.30x |

The default, 8 parts on this machine, ran within 5% of 8.

- **At C's speed on the solver heavy scenes**: 1.03 to 1.14x C's pthreads build at every count on large_pyramid and
  many_bodies (1.06 to 1.16x on the branch), 3.5 and 4.6x at 8 parts. mesh_drop and village keep their 1.2 to 1.3x,
  the port's own parallel step, as natively.
- **Parity, every step against the native recording** (`MODE=parity`, HASHLOG bakes, 1, 2, 4, 8 parts and the
  default): Chrome 20 of 20, Firefox 20 of 20, WebKit 20 of 20. On a page without isolation the threads build now
  fails at once: "this build runs threads, which need shared memory, and the page is not cross origin isolated; load
  the build without threads".
- **WebKit no longer hangs.** Every run finished, where the branch hung in 9 of 24. It scales to 4 parts and loses
  past them (large_pyramid 3.88, 2.32, 1.44, 2.26 ms at 1, 2, 4, 8; mesh_drop 5.78, 3.44, 2.46, 2.62), consistent
  with team threads that wake from WebKit's 2 ms timed waits. Firefox scales to 8 (large_pyramid 4.12 to 1.26 ms).

The whole demo on a local copy of the hosted page, on this runtime and on the published 2.48.0 ones, is in
notes/demo.md (The Threads scene): every scene loads in Chrome, Firefox and WebKit, every backend and count steps,
lane rows line up since #15189, and the 2.48.0 runtimes run the master bake with Parallel off.

## Results

### Single threaded

ms per step on V8 in node: each run's median over the steps after the first 10%, min over 5 interleaved rounds (one
process per run, 240 runs, load 3.9 to 8.5). Each run first runs the scene once untimed in the same instance so V8
has tiered every hot function up (below). The queries scene's step includes its queries.

| scene | native NEON | native scalar | C wasm SIMD | C wasm scalar | AS SIMD | AS scalar | AS / C SIMD | AS / C scalar |
|---|---|---|---|---|---|---|---|---|
| large_pyramid | 3.34 | 5.66 | 3.72 | 9.81 | 4.40 | 8.41 | 1.18 | 0.86 |
| many_bodies | 6.68 | 11.30 | 7.64 | 19.59 | 9.74 | 17.91 | 1.27 | 0.91 |
| mesh_drop | 5.35 | 8.00 | 5.85 | 10.67 | 6.99 | 11.64 | 1.19 | 1.09 |
| village | 2.55 | 3.21 | 2.93 | 4.69 | 3.76 | 4.87 | 1.29 | 1.04 |
| plank_pile | 0.661 | 0.706 | 0.724 | 0.789 | 0.757 | 0.823 | 1.04 | 1.04 |
| chains | 0.0169 | 0.0171 | 0.0232 | 0.0228 | 0.0259 | 0.0255 | 1.12 | 1.12 |
| pyramid | 0.0290 | 0.0515 | 0.0354 | 0.0913 | 0.0428 | 0.0769 | 1.21 | 0.84 |
| queries | 0.113 | 0.114 | 0.136 | 0.142 | 0.182 | 0.181 | 1.34 | 1.27 |

Over native NEON, C wasm SIMD costs 1.09 to 1.37x and the port's SIMD artifact 1.15 to 1.61x. The mean per step over
the same steps gives the same ratios within 0.05 (AS / C SIMD 1.05 to 1.32).

Headless Chrome, 3 rounds, mean per step after the first 10% (Chrome's timer is 5 µs even on an isolated page, too
coarse for the small scenes' medians):

| scene | C wasm SIMD | C wasm scalar | AS SIMD | AS scalar | AS / C SIMD | AS / C scalar |
|---|---|---|---|---|---|---|
| large_pyramid | 3.78 | 9.59 | 4.07 | 8.65 | 1.08 | 0.90 |
| many_bodies | 8.40 | 20.31 | 9.04 | 18.55 | 1.08 | 0.91 |
| mesh_drop | 7.74 | 10.78 | 9.10 | 12.35 | 1.18 | 1.15 |
| village | 2.56 | 3.74 | 3.21 | 4.07 | 1.25 | 1.09 |
| plank_pile | 0.807 | 0.797 | 0.756 | 0.833 | 0.94 | 1.05 |
| chains | 0.0503 | 0.0547 | 0.0559 | 0.0583 | 1.11 | 1.07 |
| pyramid | 0.0388 | 0.0960 | 0.0411 | 0.0801 | 1.06 | 0.83 |
| queries | 0.146 | 0.153 | 0.189 | 0.195 | 1.29 | 1.27 |

Chrome agrees with node: the same ratios within the noise, a little kinder to the port on the two pyramid
scenes (Chrome 155 carries a newer V8 than node 26).

Query time per call kind, ms per step, same node session:

| scene, kind | native NEON | C wasm SIMD | AS SIMD | AS / C |
|---|---|---|---|---|
| queries, ray closest | 0.0334 | 0.0387 | 0.0540 | 1.39 |
| queries, ray all | 0.0096 | 0.0121 | 0.0185 | 1.53 |
| queries, shape cast | 0.0475 | 0.0532 | 0.0679 | 1.28 |
| queries, overlap | 0.0065 | 0.0089 | 0.0119 | 1.34 |
| queries, mover | 0.0033 | 0.0045 | 0.0068 | 1.50 |
| village, shape cast | 0.168 | 0.191 | 0.273 | 1.43 |

### V8 against WAMR AOT, the same module

The port's release bake on rive-cli's WAMR AOT tier (bench/as, `--aot`, the SIMD artifact, rive master's CLI and
`~/Projects/tools/wamrc`), interleaved with V8 and native in one session (3 rounds, load 5 to 8). Median from step
1 on, as bench/as times it:

| scene | native NEON | C wasm SIMD on V8 | AS on V8 | AS on WAMR AOT | AS V8 / native | AS AOT / native |
|---|---|---|---|---|---|---|
| large_pyramid | 3.38 | 3.90 | 4.58 | 4.69 | 1.36 | 1.39 |
| many_bodies | 7.34 | 7.92 | 10.39 | 10.08 | 1.42 | 1.37 |
| mesh_drop | 4.43 | 5.00 | 5.70 | 5.49 | 1.29 | 1.24 |
| village | 2.53 | 2.93 | 3.62 | 3.65 | 1.43 | 1.44 |
| plank_pile | 0.665 | 0.732 | 0.760 | 0.699 | 1.14 | 1.05 |
| chains | 0.0186 | 0.0284 | 0.0293 | 0.0343 | 1.58 | 1.84 |
| pyramid | 0.0290 | 0.0354 | 0.0426 | 0.0438 | 1.47 | 1.51 |
| queries | 0.113 | 0.136 | 0.178 | 0.180 | 1.58 | 1.59 |

V8 and WAMR AOT run our module within 10% of each other on every scene, and V8 is ahead on the small, call heavy
ones (chains). notes/perf_3.md's AOT ratios (1.07x to 1.32x on the big scenes) came from a busier hour, where
native lost more than AOT; side by side today they are 1.24 to 1.44x on both engines.

### Threaded

ms per step, the mean from step 1 on (the threaded bench's own metric), min over 3 rounds. Native Box3D uses its
own scheduler; the C wasm build is the same with pthreads (`-pthread`, shared memory, 8 pooled workers). The port
runs on rive's threaded web runtime (librive's web scripting harness built with `--web_threads`), lanes on
workers (TASKS=2, the solver) and the best lane configuration of three (solve or both on workers, solve on jobs).
Parallel calls did not run in a browser at the time, so their row is native WAMR AOT (their browser runs came later,
in the next section), from the rive
`parallel-calls` branch at a4fa1239d4, in a separate session with its own native column (3 rounds, load 5 to 15,
every final hash matching native). The same session on the branch's previous head, 28be2e0b62, at load 5 to 15,
gave the same within 10% (large_pyramid 4.42, 2.33, 1.37, 1.18).

large_pyramid:

| impl | host | 1 | 2 | 4 | 8 |
|---|---|---|---|---|---|
| native NEON | native | 3.42 | 1.78 | 1.01 | 0.99 |
| C wasm pthreads | node | 3.79 | 1.98 | 1.15 | 1.03 |
| C wasm pthreads | Chrome | 3.81 | 1.99 | 1.15 | 1.08 |
| AS serial | node / Chrome | 4.63 / 4.64 | | | |
| AS lanes, solve on workers | node | 4.78 | 4.71 | 4.43 | 4.92 |
| AS lanes, solve on workers | Chrome | 4.65 | 4.81 | 4.81 | 5.34 |
| AS lanes, best config | node | 4.77 | 4.71 | 4.43 | 4.83 |
| native NEON (parallel session) | native | 3.39 | 1.78 | 1.04 | 1.03 |
| AS parallel calls, AOT | native | 4.51 | 2.46 | 1.50 | 1.28 |

many_bodies:

| impl | host | 1 | 2 | 4 | 8 |
|---|---|---|---|---|---|
| native NEON | native | 7.22 | 3.40 | 2.02 | 1.55 |
| C wasm pthreads | node | 7.88 | 3.92 | 2.22 | 1.73 |
| C wasm pthreads | Chrome | 8.08 | 4.28 | 2.26 | 1.72 |
| AS serial | node / Chrome | 9.75 / 10.11 | | | |
| AS lanes, solve on workers | node | 9.96 | 9.57 | 8.27 | 9.37 |
| AS lanes, solve on workers | Chrome | 9.81 | 10.69 | 8.92 | 9.97 |
| AS lanes, best config | node | 9.94 | 9.57 | 8.27 | 7.52 (jobs) |
| native NEON (parallel session) | native | 7.10 | 3.43 | 2.05 | 1.71 |
| AS parallel calls, AOT | native | 9.82 | 5.04 | 2.86 | 2.06 |

mesh_drop (120 steps):

| impl | host | 1 | 2 | 4 | 8 |
|---|---|---|---|---|---|
| native NEON | native | 6.52 | 3.44 | 1.95 | 1.52 |
| C wasm pthreads | node | 7.21 | 3.72 | 2.18 | 1.69 |
| C wasm pthreads | Chrome | 7.48 | 3.95 | 2.19 | 1.63 |
| AS serial | node / Chrome | 8.72 / 8.77 | | | |
| AS lanes, solve on workers | node | 8.67 | 8.30 | 8.17 | 7.83 |
| AS lanes, solve on workers | Chrome | 8.80 | 8.15 | 8.24 | 8.30 |
| AS lanes, best config | node | 8.60 | 8.30 | 7.57 | 7.45 (both) |
| native NEON (parallel session) | native | 6.56 | 3.42 | 1.93 | 1.56 |
| AS parallel calls, AOT | native | 8.38 | 4.77 | 2.81 | 2.09 |

village (120 steps):

| impl | host | 1 | 2 | 4 | 8 |
|---|---|---|---|---|---|
| native NEON | native | 2.13 | 1.34 | 0.93 | 0.97 |
| C wasm pthreads | node | 2.48 | 1.54 | 1.04 | 1.00 |
| C wasm pthreads | Chrome | 2.45 | 1.54 | 1.06 | 0.98 |
| AS serial | node / Chrome | 3.02 / 3.22 | | | |
| AS lanes, solve on workers | node | 3.05 | 3.34 | 3.52 | 4.44 |
| AS lanes, solve on workers | Chrome | 3.16 | 3.72 | 3.84 | 4.82 |
| AS lanes, best config | node | 3.05 | 3.34 | 3.52 | 4.44 |
| native NEON (parallel session) | native | 2.14 | 1.34 | 0.92 | 0.92 |
| AS parallel calls, AOT | native | 2.77 | 1.81 | 1.24 | 1.36 |

What the threaded tables say:

- **C Box3D threads as well on the web as natively.** The pthreads build scales 1.9x at 2 workers, 3.3 to 3.6x at
  4 and 3.5 to 4.7x at 8 (village 2.5x), the same as native's 1.9, 3.4 to 3.6 and 3.5 to 4.7x, and stays 1.04 to
  1.16x native at every count, in node and in Chrome alike.
- **Lanes do not scale on the web, as they did not natively** (notes/threading.md): at best 1.1 to 1.4x of serial,
  and village gets slower with every lane. At 8 workers C wasm is 4.6x (large_pyramid), 4.4x (many_bodies), 4.4x
  (mesh_drop) and 4.4x (village) faster than the port's best lanes on the same page.
- **Parallel calls close that.** Natively they scale like C (4.51, 2.46, 1.50 and 1.28 ms on large_pyramid,
  3.5 to 4.9x at 8 parts except village) and sit 1.2 to 1.5x native at every count. In Chrome they now do the same
  against C's pthreads build, at 1.06 to 1.38x (next section).

### Parallel calls in the browser

The port's parallel calls backend (`lib/parallel/`, bench/threads BACKEND=3) on rive `parallel-calls` at b9685d0884
(#15105), which runs parallel calls on page threads. Hosted as the branch's docs ("In a browser", threading.md) and
web tests (test.sh, browser_audio.mjs) do it: librive's threads build (`--web_threads`, release) on a cross origin
isolated page, the job pool sized before the file loads (`_harness_job_threads(n - 1)` for n parts, as the tests'
`--job-threads`; `_harness_parallel_safe()` for 1 part), `riveScripting.prepare()` awaited before the scripts
start, so the team is ready for the first call, and a frame per event loop turn. The page runs one part on its own
thread and one on each team thread, so n parts is n threads. `bench/web/parallel.mjs` drives it.

Session 2026-10-26 19:51 to 20:16 local, headless Chrome 156.0.8078.48, 5 rounds, each round every scene's AS runs
at 1, 2, 4, 8 parts and the job pool's default interleaved with C's pthreads build at 1, 2, 4 and 8 workers, a
fresh page per run. ms per step, median from step 1, min over rounds. The machine was shared and on battery: load
22 to 86 (one 5 minute stretch ran single threaded code 5 to 9x slow, native included), so the min over 5 rounds
matters more than usual. 178 of 180 runs finished, every final hash matched native, and every run below is from a
round that did; two runs (one C, one AS) never answered the driver under load 60 (below).

| scene | parts | C pthreads | AS parallel calls | AS / C | C speed up | AS speed up |
|---|---|---|---|---|---|---|
| large_pyramid | 1 | 3.935 | 4.165 | 1.06 | 1.00x | 1.00x |
| large_pyramid | 2 | 2.165 | 2.345 | 1.08 | 1.82x | 1.78x |
| large_pyramid | 4 | 1.245 | 1.395 | 1.12 | 3.16x | 2.99x |
| large_pyramid | 8 | 1.050 | 1.215 | 1.16 | 3.75x | 3.43x |
| large_pyramid | default (7) | | 1.290 | | | 3.23x |
| many_bodies | 1 | 9.725 | 10.565 | 1.09 | 1.00x | 1.00x |
| many_bodies | 2 | 4.465 | 5.010 | 1.12 | 2.18x | 2.11x |
| many_bodies | 4 | 2.400 | 2.715 | 1.13 | 4.05x | 3.89x |
| many_bodies | 8 | 1.795 | 1.995 | 1.11 | 5.42x | 5.30x |
| many_bodies | default (7) | | 2.245 | | | 4.71x |
| mesh_drop | 1 | 5.350 | 5.975 | 1.12 | 1.00x | 1.00x |
| mesh_drop | 2 | 2.720 | 3.510 | 1.29 | 1.97x | 1.70x |
| mesh_drop | 4 | 1.645 | 2.235 | 1.36 | 3.25x | 2.67x |
| mesh_drop | 8 | 1.200 | 1.650 | 1.38 | 4.46x | 3.62x |
| mesh_drop | default (7) | | 1.820 | | | 3.28x |
| village | 1 | 2.975 | 3.365 | 1.13 | 1.00x | 1.00x |
| village | 2 | 1.860 | 2.270 | 1.22 | 1.60x | 1.48x |
| village | 4 | 1.205 | 1.510 | 1.25 | 2.47x | 2.23x |
| village | 8 | 1.120 | 1.500 | 1.34 | 2.66x | 2.24x |
| village | default (7) | | 1.400 | | | 2.40x |

- **Parallel calls scale in Chrome like C's pthreads:** 3.4 to 5.3x at 8 parts on the solver heavy scenes (C 3.75
  to 5.4x), at 1.06 to 1.16x C's time at every count on large_pyramid and many_bodies. The port went from 4.4 to
  4.6x slower than C at 8 threads (lanes) to 1.1 to 1.4x.
- **mesh_drop and village lose ground with parts** (1.12 to 1.38x and 1.13 to 1.34x C). Natively the same scenes do
  the same against native Box3D (mesh_drop 1.29 to 1.37x, village 1.30 to 1.40x, notes/threading.md), so this is
  the port's parallel step (the contacts and bodies it leaves to the main lane, `parDeferredContacts` and
  `parDeferredBodies`), not the web.
- **The web is no slower than native here.** At 8 parts Chrome runs large_pyramid in 1.22 ms against 1.28 on native
  WAMR AOT (4.51, 2.46, 1.50, 1.28 at a4fa1239d4, an earlier session), since V8 runs the port's code faster than
  WAMR does single threaded too.
- **The default part count is 7 on the web.** The job pool takes `min(cores - 1, 6)` threads and a browser does not
  say which cores are fast, so the M5 Max (6 performance cores) gets 7 parts where native gets 6. 7 parts was
  slower than 8 on three scenes and faster on village; neither is the 6 native picks.
- **Firefox 157** (Playwright's build, the parity session at load 5 to 6, one run each, not interleaved): 4.86,
  2.82, 1.62 and 1.94 ms on large_pyramid at 1, 2, 4 and 8 parts, many_bodies 11.24 to 2.74, mesh_drop 7.34 to
  2.46, village 4.52 to 2.36. It scales to 4 parts as Chrome does, and 8 parts is slower than 4 on large_pyramid
  and village.
- **WebKit 27.2** (Playwright's build): every 1 part run finished and matched, as fast as Chrome at low load (4.60
  ms large_pyramid), but 9 of 24 runs at 2 or more parts never finished, over two sessions (load 5 to 10 in the
  second). The second session's stuck page did not answer at all ("page thread unresponsive"): its thread was
  still in the call's join, which spins with no bound until every team thread reports its part. The runs that
  finished got slower past 2 parts (large_pyramid 4.60, 3.14, 9.98, 8.14 ms). That reads as team threads that wake
  late or never in WebKit, which parks them with `Atomics.waitAsync`.

What a call and a sync cost, rive's web stage benchmark (`fixture/parallel_bench/main.as` from the branch, baked as a
release bake), in Chrome at 8 parts and natively on AOT with `--job-threads=7`, the same session at load 10 to 15,
µs, 2 rounds each (the lower):

| | 1 part | 2 | 4 | 8 |
|---|---|---|---|---|
| empty call, Chrome | 0.10 | 1.07 | 1.59 | 2.11 |
| empty call, native AOT | 0.06 | 0.66 | 0.77 | 2.01 |
| sync, Chrome | | 0.21 | 0.26 | 0.62 |
| sync, native AOT | | 0.16 | | 0.56 |

- A sync costs what it does natively, as the docs say. A call costs 1.6 to 2x native at 2 and 4 parts (the page
  thread notifies the team and polls each member's line) and the same at 8.
- With 5 to 10 µs of work per stage, 8 parts ran at 41 to 54% of perfect scaling on the web and 48 to 58% natively,
  under the docs' 60 to 85%: 8 parts is more than this machine's 6 performance cores, so two parts land on
  efficiency cores and every sync waits for them. A step of the port has a few hundred syncs, at under a
  microsecond each, so neither cost shows in the table above.

**Parity.** Every step's state hash against the native scalar recording (`MODE=parity`, HASHLOG bakes), at 1, 2, 4
and 8 parts and the default: 20 of 20 runs in Chrome before the backend moved to `lib/parallel/` and 19 of 20 after
(the 20th stalled in the harness's frame loop, below), 20 of 20 in Firefox, and 21 of 30 in WebKit, every run that
finished; 14 more Chrome runs while chasing the stall all matched too.

**Without cross origin isolation** a page cannot share memory, so it loads librive's build without threads
(`out/wasm_release`), as test.sh's `--no-isolation` lanes do:
- `Parallel.size()` is 1, the team is empty (`parallelState()` is `{"team":0,"waits":"none"}`), and every call runs
  its parts on the page's thread. Asking for 7 job threads changes nothing.
- Nothing traps, and every step matches native, at the default and with 7 job threads asked for, in Chrome, Firefox
  and WebKit (8 of 8 runs each). The memory is unshared.
- The step times are serial (large_pyramid 4.9 to 5.9 ms in Chrome), read through a coarser clock: such a page's
  `performance.now()` has 100 µs steps in Chrome and 1 ms in Firefox.
- The threads build on such a page never finishes loading, in every browser: emscripten waits on
  "loading-workers" forever after the browser refuses to send its memory to a worker (Chrome: "SharedArrayBuffer
  transfer requires self.crossOriginIsolated"; Firefox: "The WebAssembly.Memory object cannot be serialized";
  WebKit: "DataCloneError"). So the page must pick the build from `self.crossOriginIsolated` before it loads one;
  nothing does that for it.

## Parity

The per step state hash (FNV-1a over every body's position, rotation and velocities, as verify/scenes/hash.mjs
reads a recording, a destroyed body as zero words) is checked against the native scalar recording
(verify/scenes/out, recorded at 1 worker) on every step of every run:

- `bench/web/verify.sh`: all 13 scenes on native NEON, native scalar, C wasm SIMD, C wasm scalar, AS SIMD and AS
  scalar, 78 of 78 pass on every step; the pthreads builds (SIMD and scalar) at 1, 2, 4 and 8 workers, 104 of 104.
- Every bench run checks it again: 240 node and 96 Chrome single threaded runs, and every threaded run (252 in
  node, 204 in Chrome; the lanes and parallel calls bakes check the final step's hash), all pass.
- The query result hashes of village and queries match native on every build.

Nothing from V8's NaN canonicalization or emscripten's libm showed: Box3D calls no transcendentals, and its SSE2
path on wasm SIMD (emscripten's `xmmintrin.h`) gives the same bits as NEON and as scalar code.

## Analysis

### Where the port is slower than C on V8

b3Profile stages, mean ms per step, node, the run with the lowest median (large_pyramid; the other big scenes
follow the same pattern):

| stage | native NEON | C wasm SIMD | AS SIMD | AS / C | C wasm scalar | AS scalar | AS / C |
|---|---|---|---|---|---|---|---|
| step | 3.347 | 3.736 | 4.455 | 1.19 | 10.047 | 8.432 | 0.84 |
| pairs | 0.183 | 0.190 | 0.237 | 1.25 | 0.200 | 0.241 | 1.20 |
| collide | 0.453 | 0.605 | 0.608 | 1.01 | 0.875 | 0.687 | 0.78 |
| prepare | 0.359 | 0.347 | 0.484 | 1.39 | 1.189 | 1.265 | 1.06 |
| integrate velocities | 0.055 | 0.061 | 0.075 | 1.23 | 0.066 | 0.067 | 1.01 |
| warm start | 0.300 | 0.301 | 0.358 | 1.19 | 0.940 | 0.659 | 0.70 |
| solve | 0.643 | 0.684 | 0.901 | 1.32 | 2.518 | 1.962 | 0.78 |
| integrate positions | 0.089 | 0.104 | 0.116 | 1.11 | 0.097 | 0.097 | 1.00 |
| relax | 0.919 | 1.035 | 1.194 | 1.15 | 3.574 | 2.866 | 0.80 |
| store impulses | 0.156 | 0.169 | 0.244 | 1.44 | 0.350 | 0.370 | 1.06 |
| transforms | 0.181 | 0.225 | 0.217 | 0.97 | 0.220 | 0.196 | 0.89 |

The same ratios on the other scenes (AS / C SIMD): prepare 1.50, 1.50, 1.61 and store impulses 1.54, 1.45, 1.77
(many_bodies, mesh_drop, village) are the largest; solve 1.31, 1.23, 1.39; relax 1.16, 1.05, 1.28; collide 1.19,
1.26, 1.36; pairs 1.36 and 1.37 on mesh_drop and village; transforms (finalize and continuous) 1.02 to 1.13.

V8 profiles of large_pyramid (`node --cpu-prof`, 100 µs, both passes of the run). The C build for this profile
keeps names and skips binaryen's link time inlining (`--profiling-funcs -O1` at link; clang still compiles at
-O3), which otherwise folds every solver kernel into `b3ExecuteBlock`; it runs at the same speed (3.90 against
3.93 ms). Self time, ms, over the same 400 steps:

| function, C | C ms | function, AS | AS ms | AS / C |
|---|---|---|---|---|
| `b3SolveContacts_ConvexW4` (solve and relax) | 431 | `b3SolveContactsL<v128>` | 514 | 1.19 |
| `b3PushContacts_ConvexW4` (biased iterations) | 284 | `b3PushContactsL<v128>` | 385 | 1.35 |
| `b3PrepareContacts_ConvexW4` | 190 | `b3PrepareConstraintL<v128>` | 268 | 1.41 |
| `b3WarmStartContacts_ConvexW4` | 133 | `b3WarmStartContactsL<v128>` | 154 | 1.16 |
| `b3StoreImpulses_ConvexW4` | 103 | `b3StoreImpulsesW` | 157 | 1.52 |
| `b3ExecuteBlock` (integrations, dispatch) | 72 | `b3ExecuteBlock` | 94 | 1.31 |
| `b3CollideTask` | 332 | collide task closure (`<lambda>_1050`) | 390 | 1.18 |
| `b3FinalizeBodiesTask` | 63 | finalize closure (`<lambda>_1011`) | 65 | 1.03 |
| whole profile | 1993 | | 2474 | 1.24 |

Both modules spend the same shares: the five contact kernels are 57% of C's samples and 60% of the port's. There is
no runtime overhead left to blame: no allocator, collector or host call shows (`__alloc` 0.2%, nothing from the
frame collector, which runs between steps). The gap is the code inside the same kernels. The wasm the two
compilers hand V8, per kernel (binaryen `--func-metrics` on the shipped modules):

| function | ops | locals | loads | stores | lane loads | shuffles | arithmetic | if | global.get |
|---|---|---|---|---|---|---|---|---|---|
| C solve | 2076 | 66 | 152 | 24 | 0 | 90 | 543 | 0 | 0 |
| AS solve | 2512 | 75 | 174 | 24 | 0 | 126 | 698 | 19 | 3 |
| C push | 1352 | 71 | 104 | 17 | 0 | 90 | 282 | 0 | 0 |
| AS push | 1601 | 74 | 108 | 17 | 0 | 126 | 364 | 15 | 1 |
| C prepare | 2687 | 97 | 135 | 88 | 34 | 82 | 598 | 0 | 0 |
| AS prepare | 3374 | 89 | 159 | 79 | 0 | 132 | 736 | 35 | 7 |
| C warm start | 1094 | 55 | 86 | 16 | 0 | 60 | 217 | 0 | 0 |
| AS warm start | 1359 | 45 | 97 | 16 | 0 | 84 | 321 | 19 | 1 |
| C store impulses | 451 | 34 | 44 | 19 | 0 | 0 | 85 | 0 | 0 |
| AS store impulses | 581 | 28 | 44 | 11 | 0 | 0 | 115 | 22 | 0 |

The port's kernels carry 20 to 26% more operations than C's for the same math, and the extra ops line up with the
stage gaps:

- **Branches C does not have.** 15 to 35 `if`s per kernel: rasc's checks on array indexes and empty lanes. C's
  kernels have none. V8 needs no bounds checks of its own on either (guard regions), so these are the whole
  difference in control flow.
- **Address arithmetic.** The float math is the same op for op (solve: 231 `f32x4.mul` and 134 against 136
  `f32x4.add` on both sides), but the port's solve has 178 `i32.add` and 218 `i32.const` against C's 56 and 76, and
  only 4 of its loads carry a constant offset against C's 139 (prepare: 242 against 65 adds, 4 against 241
  offsets). The port adds each field's offset to its base explicitly; C folds it into the load's offset immediate.
  V8 cannot fold an `i32.add` into the access itself, because the add wraps at 32 bits and the offset does not, so
  each one costs an instruction. binaryen folds such adds only under `--low-memory-unused`, which emscripten passes
  and rasc's post pass does not, so that flag (if the bake keeps the first 1 KB of memory unused) or folding in
  rasc's own codegen would likely remove most of them.
- **More shuffles.** 126 against 90 `i8x16.shuffle` in solve and push, 132 against 82 in prepare, plus more
  `v128.const` and `v128.bitselect`: the port's row gathers and transposes, and its lane blends.
- **No lane loads in prepare and store impulses.** C's prepare uses 34 `v128.load32_lane`; rasc has no lane loads
  or stores (notes/perf_3.md), so the port replaces them with lane replaces and extra shuffles, and store impulses
  extracts each lane and stores it as a scalar behind an index check. These are the two stages with the largest
  gap (1.4 to 1.8x).
- **Collide and pairs** (1.0 to 1.37x) pay for objects where C has structs: the contact, its manifolds and the shapes
  are separate heap objects reached through references and checked indexes, and the pair callbacks are closures.
  The queries (1.28 to 1.53x) are the same story with a closure call per result.

### Why the scalar artifact beats C's scalar build

C's `BOX3D_DISABLE_SIMD` build runs `simd_scalar.h`'s four lane structs as straight scalar code: wasm has no
autovectorization without `-msimd128`, so the solve kernel becomes 6762 ops with 223 locals (native clang
autovectorizes the same code into NEON, which is why native scalar is only 1.7x native SIMD while wasm scalar is
2.6x wasm SIMD). The port's scalar artifact runs its kernels one lane at a time in a loop over the four lanes
(`b3SolveContactsL<f32>`, 1955 ops and 69 locals) and skips empty lanes, so V8 keeps one lane's state in
registers. The iteration kernels come out 0.70 to 0.82x C's, the whole step 0.84 to 0.91x on the solver heavy
scenes. prepare (1.06 to 1.46x) and store impulses (1.06 to 1.53x) stay slower for the reasons above.

### V8 against WAMR AOT

The same module runs within 10% on V8 and on WAMR AOT, though the two lose time in different places:

- WAMR pays for software bounds checks (6 to 8%), memory base reloads after stores in functions that call (8 to
  10% of the collide paths), a stub on every AOT to AOT call, and v128 spills (notes/perf_3.md). V8 has none of
  the first three on 64 bit hosts: it uses guard regions and calls wasm functions directly.
- V8's optimizing compiler gives back about as much: its register allocation and instruction selection for wasm
  SIMD are weaker than LLVM's, which compiles WAMR's AOT code at -O3. The evidence is C: its wasm build on V8 runs
  1.09 to 1.17x native on the big scenes, the cost of V8 code generation alone, and in this session it is 1.17x
  to 1.30x ahead of the port on V8.
- So on the web the port is where it is on WAMR: 1.2 to 1.45x native on the big scenes (1.05 to 1.15x on
  plank_pile, 1.5 to 1.6x on the small and query scenes). The extra ops above are the part both engines share,
  and the place to win on the web: lane loads and stores in rasc, cheaper gathers, fewer index checks in the
  kernels (unchecked access in `unsafe` code), field offsets folded into loads.

### Threads on the web

- **C pthreads.** Box3D's scheduler runs unchanged on emscripten pthreads: workers are browser workers over one
  shared memory, and Box3D's spin and wait loops work in node and on an isolated Chrome page. It scales like native.
- **Lanes.** The port's lanes copy the solver's data to each lane and back and sync 84 colored stages per step
  (notes/threading.md). The web adds nothing on top: the lane tables match the native ones (solve on workers 1.0 to
  1.2x, both on workers losing). On the web the workers also need the page to yield before their threads start, so
  the bake must idle a few frames after creating them (`IDLEFRAMES=30` in bench/threads; without it the first
  step's `b3LaneAwait` spins on workers that have not started and gives up).
- **Parallel calls run in a browser since #15105 reached b9685d0884** and scale like C's pthreads (above). At
  a4fa1239d4 they did not: the web host never linked `rive_parallel_v1.run` (`RIVE_WASM_PARALLEL` was gated on
  `!__EMSCRIPTEN__`), and the std called it even for a one part call, so a BACKEND=3 bake trapped on its first web
  step ("failed to call unlinked import function rive_parallel_v1.run"). b9685d0884 runs a one part call inline
  in the std, loads the module's memory as an import (shared when the page is isolated), and runs the parts on a
  team of pool threads, each with its own member instance over that memory.
- **The page drives frames off a message channel, not a timer.** A chain of `setTimeout(0)` frames stalled in
  headless Chrome under load: a 1 part run stopped advancing frames at step 96 of 120 for minutes while the page
  still answered. `lanes.html` now yields through a `MessageChannel`, and none of the 98 AS runs since stalled. One of
  them finished its steps yet its result never reached the driver, as one C run's never did, both at load 60;
  those are the two missing runs in the table. A real
  page drives frames with `requestAnimationFrame` and does not hit this.

## How it was measured

### Builds

- **emscripten** 6.0.13 (`emsdk install latest` into the session scratchpad; the global ~/emsdk stays 3.1.61).
- **C Box3D**, `bench/web/c/CMakeLists.txt`: Box3D at f36d061 as a subdirectory, so its top level pthread flags
  and `BOX3D_VALIDATE` (a top level only option, so it stays undefined and off) do not apply. Release (`-O3
  -DNDEBUG`), `-ffp-contract=off` (Box3D's own), samples and tests off. SIMD: Box3D's CMake adds `-msimd128 -msse2`
  and core.h maps wasm onto the SSE2 path. Scalar: `BOX3D_DISABLE_SIMD=ON`, no `-msimd128`. Threads:
  `WEB_THREADS=ON` adds `-pthread` to every object and `-pthread -sPTHREAD_POOL_SIZE=8` to the link. The driver,
  `bench/web/c/driver.c` over verify/scenes/scenes.c, exports `create`, `step`, `stateHash`, `profileField`,
  `queryMs` and `queryHash`. Link: `-sMODULARIZE -sEXPORT_ES6 -sALLOW_MEMORY_GROWTH -sINITIAL_MEMORY=64MB
  -sSTACK_SIZE=4MB`.
- **The port**, `bench/web/as/web.as`, the same exports over verify/scenes/scenes.as: `rasc -O2 --dangerously-fast
  --no-throw-messages --runtime frame` (and `--enable-simd`), rive master's rasc. That is a release bake as
  rive-cli makes one for a `dangerouslyFast` project: optimized, no fuel, throws stripped, the frame collector run
  between steps by the host. `-O2` is rasc's binaryen post pass, `wasm-opt -O3 -fimfs=100 -ifwl` with the module's
  features, the same options as rive-cli's in-process `postOptimizeInProcess` (O3, flexible inline size 100,
  inlining of functions with loops). The wasm-opt is binaryen version_133 built from rive-cli's own dependency
  (`packages/rive-cli/dependencies/WebAssembly_binaryen_version_133`, cmake Release, `ninja wasm-opt`).
- **Native**: `bench/web/build_native.sh` links the same driver against `~/Projects/box3d/build/release` (NEON) and
  `build/scalar`, `-O3 -ffp-contract=off`.
- **rive's threaded web runtime**: the web scripting harness of the `parallel-calls` worktree (master plus
  parallel calls; the lanes runs used its 28be2e0b62 build, whose lanes code is master's), built into its own out
  dir with emsdk 4.0.23:
  `RIVE_OUT=out/wasm_threads_release build_rive.sh wasm release --with_rive_scripting --scripting_vm=wasm
  --with_rive_layout --with_rive_tools --web_threads -- web_scripting_harness web_scripting_page` in
  `packages/runtime/tests/web_scripting`. `bench/web/lanes.mjs` drives the node harness, `lanes.html` the page one.
  The bakes are bench/threads through `bench/web/lanes_bake.sh` (`rive --unsigned-riv --release` on a dev build of
  rive-cli, `--once` before master dropped it, the options blob holding SCENE, STEPS, LANES, TASKS, BACKEND,
  IDLEFRAMES).
- **Parallel calls in a browser**: rive `parallel-calls` at b9685d0884, the web scripting harness built in
  `packages/runtime/tests/web_scripting` with emsdk 4.0.23 into two out dirs, the threads build for isolated pages
  and the plain one for pages without isolation:
  `RIVE_OUT=out/wasm_threads_release build_rive.sh wasm release --with_rive_scripting --scripting_vm=wasm
  --with_rive_layout --with_rive_tools --web_threads -- web_scripting_harness web_scripting_page`, and the same
  without `--web_threads` into `out/wasm_release`. The bakes are bench/threads through `lanes_bake.sh` with that
  branch's rive-cli (`SCENE STEPS LANES=1 BACKEND=3 TASKS=3`, plus `HASHLOG=1` for the parity bakes), which brings
  in `lib/parallel/`. C's pthreads build is `c_simd_mt` from build.sh with `THREADS=1`.
- **Parallel calls natively**: a detached checkout of the box3d-as `parallel` branch (ef1e329), its
  bench/threads/run.sh with the `parallel-calls` rive-cli and that branch's own wamrc
  (`~/Projects/rive-parallel-calls/wamrc-build/wamrc`). With `~/Projects/tools/wamrc` that CLI's AOT runs
  segfault on the first step.

### Runs

- `node bench/web/run.mjs <impl> <scene> <steps> [--workers=n] [--warmup=n] [--prewarm=0]`: one process per run.
  It creates the scene, runs it once untimed, creates it again and times every step with `performance.now()`
  around `step()`, reading the b3Profile fields and the state hash after each step (outside the timing), and the
  port's frame collector between steps. It prints one JSON line with min, median and mean over the steps from the
  warmup on, the stage means, the query times and hashes, and the first step whose hash differs from the
  recording.
- `bench/web/bench.mjs` (single threaded rounds and tables), `browser.mjs` (the same in headless Chrome, and the
  lanes bakes with `RIVS`), `threads.mjs` (threaded rounds and tables), `aot.mjs` (WAMR AOT against V8),
  `verify.sh` (parity sweep), `cpuprof.mjs` (self time from a `.cpuprofile`).
- **Tier up**: without the untimed first pass, V8's dynamic tiering still ran parts of C scalar on Liftoff 30
  steps into pyramid (median 0.169 against 0.091 ms; `--no-liftoff` gives 0.092 without the pass), and the small
  scenes' 300 steps of 30 µs do not give the tier up time to finish. With the pass every build reads the same with
  and without `--no-liftoff`. The threaded AS runs run each bake once, as a page would, from step 1 on; their big
  scenes take tens of ms per step to tier up, so the first steps barely move the means.

Logs (not committed, bench/web/out): `bench_20261026T192617.jsonl` (single threaded, node),
`chrome_20261026T193156.jsonl` (single threaded, Chrome), `aot_20261026T202455.jsonl` (AOT against V8),
`threads_*.jsonl` and `chrome_*.jsonl` (threaded), `verify_w1.log` and `verify_mt.log` (parity),
`prof/*.cpuprofile` (profiles).

Logs for parallel calls in the browser: `parallel_bench_chromium_20261027T025141.jsonl` (the table),
`parallel_parity_{chromium,firefox,webkit}_*.jsonl`, `parallel_stages_chromium_20261027T031652.jsonl`.

Commands, from the repo root:

```sh
S=<scratchpad>                                     # emsdk and binaryen 133 live there
EMSDK=$S/emsdk WASM_OPT=$S/binaryen133/bin/wasm-opt THREADS=1 PROFILE=1 bench/web/build.sh
bench/web/build_native.sh
bench/web/verify.sh                                # all scenes, all builds, every step's hash
IMPLS="c_simd_mt c_scalar_mt" WORKERS="1 2 4 8" bench/web/verify.sh
ROUNDS=5 node bench/web/bench.mjs                  # single threaded, node
PLAYWRIGHT=$S/pw ROUNDS=3 node bench/web/browser.mjs
ROUNDS=3 node bench/web/aot.mjs                    # WAMR AOT against V8
BAKE=1 ROUNDS=3 HARNESS_NODE=<rive>/packages/runtime/tests/web_scripting/out/wasm_threads_release/web_scripting_harness.mjs \
	node bench/web/threads.mjs
PLAYWRIGHT=$S/pw ROUNDS=3 WARMUP=1 IMPLS=c_simd_mt WORKERS="1 2 4 8" \
	SCENES="large_pyramid:200 many_bodies:100 mesh_drop:120 village:120" node bench/web/browser.mjs
PLAYWRIGHT=$S/pw HARNESS=<rive>/packages/runtime/tests/web_scripting/out/wasm_threads_release \
	RIVS="$(ls bench/web/out/lanes | sed -n 's/\.riv$//p' | grep -v parallel)" node bench/web/browser.mjs
METRIC=mean node bench/web/threads.mjs --table=<threads log>[,<chrome logs>]
W=<parallel-calls>/packages/runtime/tests/web_scripting/out
BAKE=1 MODE=parity PLAYWRIGHT=$S/pw HARNESS=$W/wasm_threads_release HARNESS_PLAIN=$W/wasm_release \
	node bench/web/parallel.mjs                    # parallel calls: every step, every part count, no isolation
ROUNDS=5 PLAYWRIGHT=$S/pw HARNESS=$W/wasm_threads_release node bench/web/parallel.mjs
node bench/web/parallel.mjs --table=<parallel_bench log>
BROWSER=firefox MODE=parity ... node bench/web/parallel.mjs   # or webkit, after npx playwright-core install
MODE=stages STAGE_RIV=<release bake of fixture/parallel_bench> ... node bench/web/parallel.mjs
node --cpu-prof --cpu-prof-interval=100 bench/web/run.mjs c_simd_split large_pyramid 200
node bench/web/cpuprof.mjs <file.cpuprofile>
```

`c_simd_split` is the profiling build (`-DCMAKE_EXE_LINKER_FLAGS="--profiling-funcs -O1"`), made by hand as in
build.sh's `c_build`.
