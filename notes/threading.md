# threading: Box3D's tasks on AnimaScript lanes

Box3D threads by handing closures to `b3EnqueueTaskCallback` and joining them with `b3FinishTaskCallback`:
`b3ParallelFor` (narrow phase, finalize bodies, pair finding, sensors, bullets), the solver's stage workers and the
tree rebuild. AnimaScript lanes share no memory and no closure crosses to one, so this pass maps the tasks whose
inputs and outputs can be serialized onto jobs and workers, keeps every result bit exact with the serial port and
native at any lane count, and measures what it costs.

## Results

Bit exact: verify/threads.sh runs all 13 trajectory scenes with the narrow phase, the solver stages and the tree
rebuild on lanes, at 1 lane, 1 lane through the serialized path, 2, 4 and 8 lanes, on workers and on jobs, and
compares every step's state hash with the native scalar recording: 130 of 130 runs pass, before and after merging
main's third perf pass (chains and plank_pile take the serial solver every step, joints and the overflow color not
being covered, and lanes for the rest). verify/traj.sh, the serial gate, passes all 13 scenes on the merge.

Scaling, whole step, ms per step (bench/threads/run.sh on the merge with main 5f40f4a, session
`bench/threads/out/session_20261024_143701.log`, 3 rounds interleaved with native SIMD at the same worker counts,
AOT -O3, SIMD artifact, Apple M5 Max, 1 minute load 5.5 to 71 from other agents, in parentheses per cell; min over
rounds; x is AS serial over the cell). Configs: `solve` the solver stages on workers, `solve+tree` plus the tree
rebuild job, `all` plus the narrow phase, `-jobs` the same on jobs. The session before the merge,
`session_20261024_134807.log`, has the same shape at a slower single lane baseline.

| scene | impl | 1 lanes | 2 lanes | 4 lanes | 8 lanes |
|---|---|---|---|---|---|
| large_pyramid | AS serial | **4.154** (8) | | | |
| large_pyramid | native simd | 3.240 (8) | 1.721 (8) | 0.992 (8) | 0.894 (8) |
| large_pyramid | AS solve | 4.244 (8, x0.98) | 3.949 (10, x1.05) | 3.543 (11, x1.17) | 4.943 (10, x0.84) |
| large_pyramid | AS solve+tree | 4.241 (10, x0.98) | 3.794 (10, x1.09) | 3.560 (9, x1.17) | 4.933 (9, x0.84) |
| large_pyramid | AS all | 4.312 (9, x0.96) | 5.188 (9, x0.80) | 4.806 (9, x0.86) | 7.204 (9, x0.58) |
| large_pyramid | AS solve-jobs | 4.249 (8, x0.98) | 7.116 (8, x0.58) | 3.462 (9, x1.20) | 3.934 (10, x1.06) |
| large_pyramid | AS all-jobs | 4.287 (9, x0.97) | 11.839 (9, x0.35) | 5.020 (6, x0.83) | 5.985 (8, x0.69) |
| many_bodies | AS serial | **7.862** (6) | | | |
| many_bodies | native simd | 6.121 (6) | 3.237 (6) | 1.827 (8) | 1.438 (6) |
| many_bodies | AS solve | 8.168 (8, x0.96) | 7.497 (7, x1.05) | 6.915 (6, x1.14) | 7.444 (6, x1.06) |
| many_bodies | AS solve+tree | 8.089 (6, x0.97) | 7.554 (8, x1.04) | 6.899 (5, x1.14) | 7.637 (8, x1.03) |
| many_bodies | AS all | 8.301 (8, x0.95) | 11.490 (7, x0.68) | 11.844 (7, x0.66) | 13.442 (7, x0.58) |
| many_bodies | AS solve-jobs | 8.170 (8, x0.96) | 15.047 (8, x0.52) | 13.060 (9, x0.60) | 7.040 (8, x1.12) |
| many_bodies | AS all-jobs | 8.375 (8, x0.94) | 24.720 (7, x0.32) | 21.288 (7, x0.37) | 18.538 (8, x0.42) |
| mesh_drop | AS serial | **6.140** (9) | | | |
| mesh_drop | native simd | 4.566 (10) | 2.456 (10) | 1.436 (10) | 1.135 (10) |
| mesh_drop | AS solve | 6.153 (9, x1.00) | 6.027 (9, x1.02) | 5.764 (9, x1.07) | 6.288 (8, x0.98) |
| mesh_drop | AS solve+tree | 6.001 (8, x1.02) | 5.766 (8, x1.06) | 5.675 (7, x1.08) | 6.272 (10, x0.98) |
| mesh_drop | AS all | 6.269 (11, x0.98) | 5.885 (13, x1.04) | 5.461 (12, x1.12) | 5.844 (12, x1.05) |
| mesh_drop | AS solve-jobs | 6.151 (11, x1.00) | 5.727 (12, x1.07) | 5.478 (11, x1.12) | 5.456 (10, x1.13) |
| mesh_drop | AS all-jobs | 5.846 (10, x1.05) | 6.822 (10, x0.90) | 6.114 (9, x1.00) | 5.075 (9, x1.21) |
| village | AS serial | **2.795** (8) | | | |
| village | native simd | 2.054 (8) | 1.304 (8) | 0.872 (8) | 0.872 (8) |
| village | AS solve | 2.808 (8, x1.00) | 2.973 (10, x0.94) | 3.228 (10, x0.87) | 5.011 (9, x0.56) |
| village | AS solve+tree | 2.716 (9, x1.03) | 2.877 (9, x0.97) | 3.212 (9, x0.87) | 4.954 (11, x0.56) |
| village | AS all | 2.768 (12, x1.01) | 3.528 (12, x0.79) | 3.529 (11, x0.79) | 5.697 (11, x0.49) |
| village | AS solve-jobs | 2.848 (10, x0.98) | 3.065 (10, x0.91) | 3.072 (10, x0.91) | 4.310 (10, x0.65) |
| village | AS all-jobs | 2.758 (10, x1.01) | 3.612 (9, x0.77) | 3.510 (9, x0.80) | 4.749 (8, x0.59) |

The solver stages alone (`constraints`, the b3Solve span the lanes take), mean over the run then the fastest step,
min over rounds:

| scene | impl | 1 lanes | 2 lanes | 4 lanes | 8 lanes |
|---|---|---|---|---|---|
| large_pyramid | AS serial | **3.052** / 2.980 | | | |
| large_pyramid | native simd | 2.454 | 1.290 | 0.732 | 0.664 |
| large_pyramid | AS solve | 3.145 / 3.081 (x0.97) | 2.798 / 2.649 (x1.09) | 2.323 / 2.145 (x1.31) | 3.249 / 2.422 (x0.94) |
| large_pyramid | AS solve+tree | 3.144 / 3.064 (x0.97) | 2.695 / 2.635 (x1.13) | 2.364 / 2.162 (x1.29) | 3.256 / 2.423 (x0.94) |
| large_pyramid | AS all | 3.170 / 3.079 (x0.96) | 2.832 / 2.733 (x1.08) | 2.364 / 2.265 (x1.29) | 3.129 / 2.414 (x0.98) |
| large_pyramid | AS solve-jobs | 3.149 / 3.080 (x0.97) | 5.872 / 5.646 (x0.52) | 2.345 / 2.287 (x1.30) | 2.821 / 2.512 (x1.08) |
| large_pyramid | AS all-jobs | 3.150 / 3.077 (x0.97) | 5.873 / 5.586 (x0.52) | 2.417 / 2.333 (x1.26) | 2.866 / 2.650 (x1.06) |
| many_bodies | AS serial | **6.167** / 6.078 | | | |
| many_bodies | native simd | 4.924 | 2.599 | 1.460 | 1.133 |
| many_bodies | AS solve | 6.433 / 6.275 (x0.96) | 5.662 / 5.454 (x1.09) | 4.744 / 4.497 (x1.30) | 4.785 / 4.059 (x1.29) |
| many_bodies | AS solve+tree | 6.384 / 6.285 (x0.97) | 5.709 / 5.471 (x1.08) | 4.739 / 4.500 (x1.30) | 4.903 / 4.239 (x1.26) |
| many_bodies | AS all | 6.456 / 6.277 (x0.96) | 6.305 / 5.960 (x0.98) | 5.479 / 4.529 (x1.13) | 4.983 / 4.436 (x1.24) |
| many_bodies | AS solve-jobs | 6.432 / 6.300 (x0.96) | 12.418 / 12.033 (x0.50) | 10.444 / 9.714 (x0.59) | 4.857 / 4.516 (x1.27) |
| many_bodies | AS all-jobs | 6.498 / 6.266 (x0.95) | 12.711 / 11.694 (x0.49) | 10.510 / 9.918 (x0.59) | 8.571 / 7.555 (x0.72) |
| mesh_drop | AS serial | **1.844** / 0.054 | | | |
| mesh_drop | native simd | 1.684 | 0.942 | 0.539 | 0.393 |
| mesh_drop | AS solve | 1.834 / 0.054 (x1.01) | 1.499 / 0.044 (x1.23) | 1.245 / 0.038 (x1.48) | 1.272 / 0.089 (x1.45) |
| mesh_drop | AS solve+tree | 1.854 / 0.056 (x0.99) | 1.469 / 0.049 (x1.25) | 1.260 / 0.039 (x1.46) | 1.293 / 0.106 (x1.43) |
| mesh_drop | AS all | 1.932 / 0.056 (x0.95) | 1.516 / 0.046 (x1.22) | 1.308 / 0.041 (x1.41) | 1.424 / 0.101 (x1.30) |
| mesh_drop | AS solve-jobs | 1.833 / 0.055 (x1.01) | 1.426 / 0.066 (x1.29) | 1.149 / 0.060 (x1.60) | 1.042 / 0.081 (x1.77) |
| mesh_drop | AS all-jobs | 1.816 / 0.056 (x1.02) | 1.767 / 0.093 (x1.04) | 1.665 / 0.105 (x1.11) | 1.187 / 0.169 (x1.55) |
| village | AS serial | **0.945** / 0.251 | | | |
| village | native simd | 0.807 | 0.485 | 0.281 | 0.320 |
| village | AS solve | 0.972 / 0.262 (x0.97) | 1.109 / 0.223 (x0.85) | 1.239 / 0.172 (x0.76) | 2.762 / 0.174 (x0.34) |
| village | AS solve+tree | 0.971 / 0.265 (x0.97) | 1.096 / 0.227 (x0.86) | 1.323 / 0.182 (x0.71) | 2.808 / 0.181 (x0.34) |
| village | AS all | 0.972 / 0.266 (x0.97) | 1.136 / 0.225 (x0.83) | 1.277 / 0.174 (x0.74) | 2.836 / 0.173 (x0.33) |
| village | AS solve-jobs | 0.985 / 0.262 (x0.96) | 1.196 / 0.251 (x0.79) | 1.168 / 0.207 (x0.81) | 2.313 / 0.206 (x0.41) |
| village | AS all-jobs | 0.971 / 0.266 (x0.97) | 1.198 / 0.251 (x0.79) | 1.233 / 0.200 (x0.77) | 2.299 / 0.205 (x0.41) |

The narrow phase (`collide`), where `all` puts it on lanes:

| scene | impl | 1 lanes | 2 lanes | 4 lanes | 8 lanes |
|---|---|---|---|---|---|
| large_pyramid | AS serial | **0.599** / 0.520 | | | |
| large_pyramid | native simd | 0.416 | 0.252 | 0.142 | 0.126 |
| large_pyramid | AS all | 0.670 / 0.588 (x0.89) | 1.875 / 1.782 (x0.32) | 1.949 / 1.795 (x0.31) | 3.521 / 2.719 (x0.17) |
| large_pyramid | AS all-jobs | 0.667 / 0.566 (x0.90) | 5.479 / 5.125 (x0.11) | 2.115 / 1.933 (x0.28) | 2.631 / 2.489 (x0.23) |
| many_bodies | AS serial | **1.246** / 1.089 | | | |
| many_bodies | native simd | 0.843 | 0.451 | 0.260 | 0.219 |
| many_bodies | AS all | 1.391 / 1.247 (x0.90) | 4.715 / 4.231 (x0.26) | 5.807 / 4.728 (x0.21) | 7.910 / 6.487 (x0.16) |
| many_bodies | AS all-jobs | 1.422 / 1.207 (x0.88) | 11.544 / 10.519 (x0.11) | 10.310 / 9.763 (x0.12) | 9.478 / 8.821 (x0.13) |
| mesh_drop | AS serial | **1.876** / 0.000 | | | |
| mesh_drop | native simd | 1.263 | 0.710 | 0.396 | 0.315 |
| mesh_drop | AS all | 2.174 / 0.000 (x0.86) | 2.231 / 0.000 (x0.84) | 1.909 / 0.000 (x0.98) | 1.851 / 0.000 (x1.01) |
| mesh_drop | AS all-jobs | 1.944 / 0.000 (x0.97) | 2.917 / 0.000 (x0.64) | 2.261 / 0.000 (x0.83) | 1.674 / 0.000 (x1.12) |
| village | AS serial | **0.820** / 0.000 | | | |
| village | native simd | 0.553 | 0.350 | 0.197 | 0.148 |
| village | AS all | 0.887 / 0.000 (x0.92) | 1.452 / 0.000 (x0.56) | 1.301 / 0.000 (x0.63) | 1.802 / 0.000 (x0.45) |
| village | AS all-jobs | 0.887 / 0.000 (x0.92) | 1.483 / 0.000 (x0.55) | 1.328 / 0.000 (x0.62) | 1.478 / 0.000 (x0.55) |

What the tables say:

- The step barely gets faster: 1.17 to 1.20x on large_pyramid at 4 lanes, 1.14x on many_bodies, 1.12 to 1.13x on
  mesh_drop, and village slower at every lane count. Native SIMD gets 3.6x, 4.3x, 4.0x and 2.4x from 1 to 8
  workers on the same scenes.
- The solver stages scale to about 1.3x where few bodies are shared: large_pyramid 1.31x at 4 lanes, many_bodies
  1.30x at 4 and 8, mesh_drop 1.48x on workers and 1.60 to 1.77x on jobs at 4 and 8. Village, with 15 to 18% of its
  bodies shared, gets slower with every lane (0.34x at 8). Native: 3.4x, 3.4x, 3.1x and 2.9x on the same stage at
  4 workers.
- The narrow phase on lanes loses everywhere: 0.16 to 0.32x on the pyramids, whose 15000 and 28000 contacts are all
  recycled at about 40 ns each, while each contact's state round trip costs the main lane alone about 120 ns; 0.84
  to 1.01x on mesh_drop, whose mesh contacts are expensive; 0.45 to 0.63x on village.
- The tree rebuild job takes 0.30 ms off `pairs` on mesh_drop (0.58 to 0.28) and 0.13 ms on village (0.41 to 0.28)
  for 0.05 to 0.08 ms of main lane marshalling; the pyramids rebuild little.
- Jobs and workers trade places. Jobs start without a parked worker to wake and win the solver on mesh_drop; but a
  job whose replica passes 16 MB is instantiated afresh every run (item 3 below), which makes many_bodies 2 to 3
  times slower than serial at 2 and 4 lanes (first exchange wait 5.3 ms against 0.0003 on workers), and jobs decode
  the shape snapshot every run, which sinks `all-jobs`.

## Design

### What the task callbacks can and cannot carry

The enqueue callback gets `(task, taskContext, userContext, name)`. For a parallel for, `task` is a trampoline
that claims blocks from a counter shared through its closure; for the solver it is `b3SolverTask(worker)` over the
step context. Neither the closure nor the memory it points into can reach a lane, and the callback sees nothing
else, so a generic callback can only run tasks inline. The layer therefore works at three levels:

| task | how it reaches a lane | where |
|---|---|---|
| narrow phase, `b3CollideTask` over the awake contacts | a hook in `b3Collide` in place of its `b3ParallelFor` | `b3SetLaneCollide`, lib/lane_collide.as |
| the solver stages, `b3SolverTask` | a hook in `b3Solve` in place of the worker enqueue loop | `b3SetLaneSolve`, lib/lanes.as |
| the tree rebuild, `b3UpdateTreesTask` | the world's enqueue and finish callbacks, by task name | lib/lane_tree.as |
| everything else | inline in the enqueue callback, as the default callback runs it | |

The tree rebuild is the one task whose closure carries nothing but the world, so the callback recognizes it by
name, starts a job with the two trees and returns a handle, and the finish callback joins it and copies the trees
back. It overlaps the narrow phase and the solver, as in C.

The lib hooks are small: two function typed module globals with setters (`b3SetLaneCollide` in
physics_world.as, `b3SetLaneSolve` in solver.as) checked at the two call sites, `b3CollideTask` and
`b3ExecuteBlock` exported, and the default friction and restitution callbacks exported so a replica can install
them. With no hook set the port runs as before; verify/traj.sh passes all 13 scenes with them in.

### Partitions

A step's work is split into `lanes` partitions. Partition 0 runs on the main lane, in place on the world's own
arrays, so it costs no serialization; partitions 1 to `lanes - 1` run on jobs or workers over a replica built from
a section blob. With one lane everything stays in place, and `replicaOnMain` (`REPLICA=1`) sends partition 0
through the serialized path instead, which measures the marshalling alone and checks it without another lane.

The main lane posts the command first, so a parked worker wakes while the main lane serializes, then packs one
blob per partition and publishes each as soon as it is packed (a ready word per partition in the control region),
so a lane starts on its section while the main lane packs the next.

### The solver

Every stage of `b3SolverTask` runs on the partitions: prepare, integrate velocities, warm start, solve, integrate
positions, relax, restitution and store impulses, with the kernels of contact_solver_wide.as and solver.as
unchanged, called through `b3ExecuteBlock` with blocks over the partition's ranges.

- Assignment: each color's wide constraints, and its mesh groups, split evenly in constraint order, so each
  partition takes a contiguous slice of every color, in C's own 4 lane grouping. Within a color no two
  constraints share a dynamic or kinematic body, so any assignment of a color's constraints to lanes gives the
  same bits as long as each body sees its colors in order; a body's states go through the same operations in the
  same order as serially.
- Bodies: a partition takes every body its constraints touch, and the awake bodies no constraint touches are
  split by index. A body that two partitions touch is shared: both integrate it, from the same inputs, so their
  copies stay identical, and after each color stage the partition whose constraint wrote it sends its linear and
  angular velocity to the others that touch it.
- Exchange: one region per sending and receiving pair, appended to through the step with a table of each
  exchange's offset and count, and an epoch word. The main lane works out, per color, which lanes send which
  lanes anything (`needs`); a lane waits only for those, so a color stage whose shared bodies stay inside one
  partition costs nothing. A pair region's lock has one writer and one reader, which never contend.
- The replica holds a `b3World` with contacts at their ids, awake sims and states at their awake indices, and
  compact constraint arrays with spans and color slices that give each color's slice the lanes the main lane's
  layout gives it.
- Outputs: per contact what the solver writes into its manifolds (friction, twist and rolling impulses, and per
  point the normal and total normal impulses and the normal velocity) and its hit event bit; per body owned by
  the lane (the lowest lane that touches it) the final state.
- Not covered, so those steps fall back to the serial solver: joints and the overflow color (chains and
  plank_pile fall back every step), a world created with more than one worker.

### The narrow phase

`b3CollideTask` runs unchanged on a lane's replica world.

- Shapes and geometry go in a snapshot blob, made once and rebuilt when a checksum over the shapes (ids,
  generations, types, flags and materials) changes. Hulls, meshes, height fields and compounds are deduplicated
  by object and written as their scalar head plus raw sections; a lane decodes a shape on first use. A worker
  keeps what it decoded while the snapshot stays; a job decodes again each run.
- Per step a section holds each contact's ids and state (flags, manifolds, caches, triangle cache, recycling
  poses, mixed material values), the sims of the bodies the contacts name and the bounds and fat bounds of their
  shapes. The answer is the contacts' updated state as a blob, since a mesh contact's manifolds and triangle cache
  may grow.
- The main lane copies the state back and does what the task does with the world's task context: the contact
  state bits, the counters and the mesh contact spec.
- Not covered, so those steps fall back: a custom friction, restitution or pre-solve callback, which cannot cross.

### Backends

- Workers (`b3_laneBackendWorkers`): one `@worker` per extra lane, with its replicas, pooled contact objects and
  decoded shapes kept from step to step. Between commands it spins briefly on the control region, then parks on a
  `Signal`; the main lane notifies at the start of each hook and again when it posts.
- Jobs (`b3_laneBackendJobs`): one `Jobs.run` per extra lane per task per step, building its replica fresh. The
  colored solver stages need every partition running at once, so the job pool needs at least `lanes - 1`
  threads (`--job-threads`).

The tree rebuild is always a job.

## Analysis

### What crosses, per stage

Measured per step at 4 lanes (the lane cost table of the session; bytes are the main lane's sections and the
lanes' answers). Pack and apply are the main lane's serial time; the lanes spend about as much again unpacking and
writing answers, in parallel.

| scene | solve in / out MB | solve pack + apply ms | collide in / out MB | collide pack + apply ms | collide work, serial ms | tree MB | tree pack + apply ms |
|---|---|---|---|---|---|---|---|
| large_pyramid | 4.60 / 1.08 | 0.64 + 0.35 | 9.23 / 4.41 | 1.20 + 0.46 | 0.60 | 1.40 | 0.03 + 0.02 |
| many_bodies | 8.86 / 2.12 | 1.51 + 0.91 | 17.69 / 8.38 | 3.54 + 1.55 | 1.25 | 3.14 | 0.16 + 0.15 |
| mesh_drop | 2.68 / 0.59 | 0.42 + 0.14 | 3.72 / 2.43 | 0.92 + 0.34 | 1.88 | 2.10 | 0.04 + 0.03 |
| village | 2.14 / 0.36 | 0.30 + 0.11 | 3.19 / 1.46 | 0.69 + 0.22 | 0.82 | 1.40 | 0.03 + 0.02 |

The whole round trip, measured alone by sending the main lane's own partition through the serialized path with
one lane (`REPLICA=1`), so that nothing runs in parallel and the difference to the in place run is all
marshalling:

| scene | solver stages in place ms | through the replica ms | pack (incl. blob) | lane unpack | lane answer | apply | MB in / out |
|---|---|---|---|---|---|---|---|
| large_pyramid | 3.42 | 5.76 | 0.69 | 0.76 | 0.43 | 0.60 | 6.01 / 1.45 |
| many_bodies | 7.08 | 14.36 | 1.98 | 2.19 | 1.50 | 1.61 | 11.79 / 2.83 |
| mesh_drop | 1.94 | 3.21 | 0.61 | 0.36 | 0.19 | 0.22 | 3.87 / 0.86 |
| village | 1.06 | 1.70 | 0.26 | 0.23 | 0.11 | 0.12 | 2.61 / 0.50 |

Measured on the merge; the run before it gave the same marshalling within 5%. So the solver's data costs 60 to 100%
of the solver's own work to move once each way, 0.28 to 0.50 ms per MB, of
which the bulk copies are about 0.02 to 0.03 ms per MB (bench/threads/micro: `memory.copy` 22.7, `Blob.fromArray`
34, `readInto` 22.6, region `writeFrom` 29 and `readInto` 24.7 us per MB at 8 MB). The rest is per record work:
walking `world.contacts` to each contact object and its separately allocated manifold array, and on the lane
filling contact objects and sims back in.

### Why the colored stages cannot scale here

A step runs 84 colored stages (7 active colors, warm start, solve and relax over 4 sub-steps), and after each the
lanes that share a body swap its velocity. large_pyramid's solver takes about 3.1 ms serially on the merge, so a
color stage holds about 33 us of work: 8 us per lane at 4 lanes, 4 at 8. One exchange wait costs 4.4 us at 4 lanes
(108 waits, 0.48 ms per step) and 8.1 at 8 (156 waits, 1.26 ms), mostly spent waiting for the slowest neighbor to
arrive, then two lock and copy calls of 0.2 to 0.5 us each. At 4 lanes the sync costs half the work it separates;
at 8, twice as much. Village has 353 waits per step at 8 lanes (1.87 ms) against a 0.95 ms solver; many_bodies 72
and 84 waits (0.50 and 0.67 ms) although only 0.2 and 0.9% of its bodies are shared, since the first wait of a step
also absorbs the lanes' unpacking.

The floor of a lane barrier on this runtime is the host call atomics: 11 to 15 ns per `atomicLoad`, each a
registry lookup that pins and unpins the region's slot, so lanes spinning on one word contend on it: a counting
barrier over one region costs 1.1 us with 2 lanes, 3.5 to 3.9 with 4 and 7.6 to 14.9 with 8 (bench/threads/micro,
`barrier`). Native Box3D syncs the same 84 stages on plain atomics in shared memory, which is why its solver keeps
scaling to 8 workers.

The partitioning keeps exchanges rare where the scene allows: each color is split in constraint order, which
follows creation order, and a lane waits only for lanes that send it something in that color. many_bodies' 196
pyramids share 0.2% of their bodies at 4 lanes and 0.9% at 8, so most colors cost no wait; large_pyramid shares 7%
and 15%, village 15% and 18%.

### The flat stages as jobs of their own

Running prepare, the integrations, store impulses or finalize bodies as separate jobs, as C's parallel for runs
them, would ship each stage's inputs and outputs per stage. At the measured 0.28 to 0.50 ms per MB of round trip:

| stage, large_pyramid | per record in / out | MB per step | marshal ms | stage ms, serial |
|---|---|---|---|---|
| prepare contacts | about 400 / 436 B per contact (the wide constraint goes back for the iterations) | 12.5 | 4.2 to 6.1 | 0.65 to 1.4 |
| integrate velocities | 170 / 24 B per body | 1.0 | 0.33 to 0.48 | 0.12 |
| integrate positions | 56 / 60 B per body | 0.6 | 0.20 to 0.29 | 0.18 |
| store impulses | 436 / 60 B per contact | 7.4 | 2.5 to 3.6 | 0.4 to 0.7 |
| finalize bodies, no continuous | about 450 / 350 B per body plus shape bounds | 4.0 | 1.4 to 2.0 | 0.26 to 0.38 |

Each would lose 2 to 10 times what it saves, which is why the solver here crosses once per step, with every stage
of a partition run on the same lane over the same replica, and only shared velocities crossing between stages.

### The other tasks

- Island splitting: 0 to 0.0007 ms per step in these scenes (the pyramids never sleep), and it rewrites world
  islands that the main lane would then have to rebuild; not worth a lane.
- Sensor overlaps: none of the four scenes has a sensor. The task queries the three trees, which copy cheaply (the
  tree job ships the dynamic and kinematic trees, 1.4 to 3.1 MB, for 0.06 to 0.4 ms), plus each sensor's shape.
  It would follow lane_tree.as.
- Pair finding: 0.3 to 0.65 ms per step. The trees cross cheaply as above, but `b3ShouldCreatePair` reads shape
  filters, the bodies' joint lists and the pair set, and contacts must still be created on the main lane. At most
  a few tenths of a ms to win; next after the tree job.
- Finalize bodies with continuous collision: continuous needs the static tree and the shapes, which the snapshot
  and the tree job show to be cheap, but finalize writes body transforms, shape bounds, the broad-phase move buffer
  and move events, so its answer is about as large as its input (table above).

### What would change the picture

- Shared linear memory between lanes. The marshalling above, 0.4 to 2.4 ms of the main lane's own time per step at
  4 lanes for the solver and 0.9 to 5.1 ms more for the narrow phase, plus the lanes' unpacking, goes away, and
  Box3D's own scheduler runs unchanged: the enqueue callback hands the closure to a lane that sees the same memory.
  The stage syncs become wasm atomics (tens of ns) instead of host call atomics, waits and copies (3 to 9 us).
  With the marshalling gone and the sync at native cost, the solver stages would follow native's 3.4x at 4 workers
  and 3.7x at 8 (large_pyramid), and the narrow phase, which native takes from 0.42 to 0.13 ms on large_pyramid and
  1.26 to 0.32 on mesh_drop, would scale instead of losing up to 6x.
- Short of that, a region a lane can read and write in place (a typed array view over a `SharedRegion`, with the
  region's atomics) removes the copies but not the per record packing, since contacts and shapes are objects in
  each lane's own heap: the solver's round trip would drop by the bulk copy share, about 10%. What it would change
  is the exchange: shared bodies' velocities could live in one region every lane writes and reads in place, with an
  epoch per color, saving the 0.2 to 0.5 us lock and copy calls per neighbor and leaving the wait.
- The section "What AS threading should change" ranks these and the smaller items with their numbers.

## What AS threading should change

This port is a dogfooding run of AnimaScript threading before it ships. Ranked by how much each would have helped
it; each item says what we hit, what it cost or forced, a repro where it is a refusal or a bug, a proposal, and
whether it looks cheap for the runtime team or needs design. Numbers are bench/threads (Apple M5 Max, AOT -O3) and
bench/threads/micro (`rive bench/threads/micro --aot --bench=18 --define=WORKERS=7 --worker-threads=8`).

### 1. Shared linear memory between lanes (design)

- Hit: no lane can read the world. Every task serializes its inputs and outputs, and the solver's colored stages
  swap shared bodies' velocities through regions after each of the 84 stages a step.
- Cost: moving the solver's data once each way costs 60 to 100% of the solver's own work (the replica table above:
  large_pyramid 2.5 ms of marshalling for 3.4 ms of solving, many_bodies 7.3 for 7.1), and the main lane's share
  alone (pack and apply, 0.4 to 2.4 ms at 4 lanes) is serial, an Amdahl ceiling of about 2 to 3x on the solver
  before any sync. The narrow phase's round trip costs the main lane 1.7 ms on large_pyramid for 0.6 ms of work.
  Each colored stage's exchange costs 3 to 9 us per wait, against 4 to 8 us of work per lane. Code: 2950
  lines of lane code, about 1200 of them serialization (contact state, section and answer codecs, the shape and
  geometry snapshot, the tree codec), with 137 raw load, store and memory.copy sites.
- Proposal: wasm threads style shared memory for a job or worker team (one memory, instances per thread, `memory.atomic.*`),
  so a task callback can hand its closure to a lane as C does; Box3D's scheduler would then run unchanged. Without
  the object graph, a typed array view over a `SharedRegion` that lanes read and write in place with the region's
  atomics would take the bulk copies (about 10% of the round trip) and the exchange's lock and copy calls (0.2 to
  0.5 us per neighbor), not the per record packing.
- What it would buy: the marshalling and most of the sync, so the solver could follow native's 3.4x at 4 workers
  and 3.7x at 8 instead of 1.3 to 1.8x, and the narrow phase could scale instead of losing up to 6x.

### 2. A fork and join with barriers that does not cost a thread wake (cheap to moderate)

- Hit: the colored stages need every partition running at once and a barrier per stage. Ports cost 20 us per round
  trip at load 8 and 447 us at load 40; a parked worker wakes 45 to 800 us after a `Signal.notify` (maximum 3 to 15
  ms); `Worker.sleep(0.0002)` polling 57 to 533 us. So the layer spins on host call atomics while a command is in
  flight (1.1, 3.5 to 3.9 and 7.6 to 14.9 us per barrier at 2, 4 and 8 lanes), wakes workers at the start of each
  hook so the wake hides behind packing, and has about 150 lines of epoch, ready word and done word protocol.
  Jobs cannot be used this way unless the pool has a thread for every lane: with `--job-threads=1` and 4 lanes the
  solver deadlocks (repro: `OPTIONS="SCENE=0 STEPS=20 LANES=4 BACKEND=1 TASKS=2" bench/threads/run1.sh 90
  --bench=30 --job-threads=1` never finishes before the layer's cap was added).
- Proposal: a `Barrier` shared object (arrive and wait, spin then futex park in the host, like `lockWord` plus a
  park), and a gang call, `Jobs.runAll(fn, n, ...args)`, that starts n instances at once or answers that it cannot,
  so spin synchronized work never waits on a job that has no thread. A futex wake is a few microseconds where the
  current resume path is tens to hundreds.

### 3. Jobs that keep their instance (cheap to moderate)

- Hit: a job starts from fresh globals, and its instance is reused only while its memory stays at or below 256
  pages (`kReuseMaxPages`, 16 MB). A job touching fresh memory each run (micro `memory`): 1 MB 10 us, 4 MB 20 us, 12
  MB 115 us, then past the limit 20 MB 0.83 ms and 40 MB 1.6 ms per run, a fresh instance and its page faults each
  time. Our solver replica for many_bodies at 2 and 4 lanes is past it: the jobs backend runs the solver at 0.50 and
  0.59x serial there and 1.27x at 8 lanes, where each replica is small (its first wait at 4 lanes is 5.3 ms against
  0.0003 on workers). Collide jobs decode the shape snapshot every run; `all-jobs` loses to `all` on every scene.
- Proposal: an opt-in resident job, `@job({ resident: true })`, whose instance, memory and globals persist between
  runs of the same function on the same pool thread, so caches and decoded geometry survive; and the reuse limit as
  a setting rather than a constant. Workers do keep state, but cost the wake of item 2 per sync.

### 4. A region lock that does not park, and reads without the lock (cheap)

- Hit: `SharedRegion.exclusive` is `std::timed_mutex::try_lock_until`, which parks contended waiters (findings.md).
  64 byte reads, 20000 rounds per worker: 7 workers on one region 11 to 35 us per lock, each on its own region 0.33
  to 0.48 us. One exchange region per lane, read by its neighbors, cost 37 us per color stage at 4 lanes and 155 at
  8; we moved to one region per sending and receiving pair, 64 regions at 8 lanes, at 7 us. A blob made and
  released per exchange is a lock free alternative: 0.115 us to make and release 1 KB, 2.1 us per 64 byte read with
  7 lanes reading one blob at once.
- Proposal: spin briefly before parking (as `lockWord` does), a reader mode (`view.shared(...)`, many readers or one
  writer), or lock free `readRange` of a region under an epoch the writer bumps.

### 5. Structs and struct arrays across lanes and into blobs (moderate, compiler)

- Hit: a struct, a `StaticArray<struct>` or an `Array<struct>` is refused as a job argument, and `Blob.fromArray`
  refuses struct arrays too, although they are plain data with a fixed layout per bake:

```as
export struct Pt { x: f32 = 0; y: f32 = 0; }
@job export function sumPts(pts: StaticArray<Pt>): f32 { return pts.length > 0 ? pts[0].x : 0; }
// E3001: @job sumPts: pts[] (Pt) cannot be sent to another lane; send numbers, bools, strings, ArrayBuffers, typed arrays, arrays, or classes of those
let blob = Blob.fromArray(new StaticArray<Pt>(4));
// E3001: Blob.fromArray, readInto and writeFrom take a typed array, or an Array or StaticArray of numbers
```

- Cost: every section is hand written with `unsafe` raw copies at `offsetof` addresses into a byte buffer, struct
  sections carry their element size by hand since a struct cannot be a generic argument, and each copy goes through
  a staging `Uint8Array` before the blob (one extra memory.copy, 0.02 ms per MB). The copies themselves are already
  as fast as an encoder's would be; the cost is the code and its unsafety.
- Proposal: structs, `StaticArray<struct>` and `Array<struct>` cross as one block of bytes, as typed arrays do, and
  `Blob.fromArray`, `readInto` and `writeFrom` take them.

### 6. A clock on lanes (cheap)

- Hit: `clock()` in lane code fails the build (E3004, findings.md), though the message lists timers as allowed. No
  lane can time its stages, so the per stage numbers here are the main lane's partition, and the lanes' unpacking
  had to be measured by running a replica on the main lane.
- Proposal: allow the monotonic `clock()` in jobs and workers; it changes no result. The perf pass noted the host
  clock reads `getenv` on every call (`rtNowImpl`, 1 to 2% of cycles), worth fixing alongside.

### 7. A warning when lane code reads a global the main lane changes (cheap, compiler)

- Hit: a lane sees a global's initializer, silently. Box3D's length units live in one (`b3_lengthUnitsPerMeter`,
  read by `B3_SPECULATIVE_DISTANCE()` in the solver and narrow phase), so the layer sends them in every section and
  sets them on the lane.

```as
let unitsPerMeter: f32 = 1.0;
export function setUnits(units: f32): void { unitsPerMeter = units; }
@job export function scaled(x: f32): f32 { return x * unitsPerMeter; }
// init: setUnits(100); Jobs.run(scaled, 2.0) answers 2.0 while scaled(2.0) on the main lane answers 200.0
```

- Proposal: the compiler already follows writes for the top level rule; a warning when lane reachable code reads a
  global that main lane reachable code writes would have named this one.

### 8. Result decoding: document Blob and joinInto (cheap, docs)

- Hit, checked against the seed figure of 2.4 ms per MB: typed array, `Array<f32>` and `StaticArray<f32>` arguments
  cost 56 to 62 us per MB with the job's start and join included, so the encoder already copies them as blocks.
  Arrays of small objects cost 1.2 to 1.4 ms per MB in and 4.4 to 5.4 ms per MB out. A typed array result read with
  `join` costs 0.8 to 1.7 ms per MB, since the main lane allocates it; with `joinInto` 0.12, and as a `Blob` read
  with `readInto` into a kept buffer 0.06.
- Proposal: say in the threading guide that large results should be blobs or `joinInto`, and that arrays of objects
  cost 20 to 80 times what typed arrays do.

### 9. Traps and logs from lanes (cheap to moderate)

- Hit: a lane trap reports `worker b3LaneWorker failed: unreachable`, no function, in release and debug bakes; lane
  logs reach the console only at the main lane's polls, so a main lane spinning on a hung lane shows nothing. The
  layer's bugs (a header off by one word, a lane order base) were found with a replica on the main lane and with
  progress words written into the control region.
- Proposal: name the function, from the name section, in a lane's failure; flush lane logs from a host thread when
  the main lane has not polled for a while.

### 10. Run options without a rebake (cheap, CLI)

- Hit: every `--define` set is a separate bake and AOT compile of the main and both lane modules, about 15.6 s
  cold, so a parity sweep of 130 runs took over an hour. Options read from a blob asset instead (`options.txt`,
  bench/threads/run1.sh) keep the wasm identical and the AOT cache hot: 4.5 s per run. Lane modules here are 280 to
  289 KB of wasm and 500 to 566 KB of AOT code each, against 1.30 MB and 1.59 MB for the main module.
- Proposal: a CLI flag for runtime arguments a script can read (`--arg name=value`, read through the context).

### Smaller things noted

- Host call atomics are 11 to 15 ns uncontended and contend on the region's registry slot when many lanes spin on
  one word (barrier above); part of item 1.
- A worker past the worker pool still runs, promoted to its own thread after 20 ms without awaiting; 8 lanes on the
  default pool of 4 workers ran large_pyramid at 6.17 ms per step against 5.96 with 8 worker threads.
- Captures: the layer joins every lane within the step, so `--advance` and `--screenshot` settle at once; nothing
  needed `--no-settle-lanes`.
- Every module runs on the AOT tier here; the lane modules carry the v128 solver kernels, so WAMR's fast interpreter
  v128 local limit (findings.md) applies to them too.
- `Object` is abstract and a class does not convert to it, so the lane shape cache keeps one array per geometry class.

### Cheap versus design

Cheap for the runtime or compiler: 4 (lock spin, reader mode), 6 (lane clock, getenv), 7 (globals warning), 8
(docs), 9 (trap names, log flush), 10 (CLI args), and the gang call of 2. Moderate: the `Barrier` object and futex
park of 2, resident jobs (3), struct crossing (5). Design work: shared memory (1), which is the one that would
change the outcome of this port.

## Files

| file | what |
|---|---|
| lib/lane_common.as | the byte writer, spin wait, main lane clock |
| lib/lanes.as | the solver partitions, workers, jobs, the hooks, `b3World_EnableLanes(worldId, lanes, backend, tasks)` |
| lib/lane_collide.as | the shape snapshot, the collide replica, packing and applying |
| lib/lane_tree.as | the tree rebuild job |
| bench/threads/ | the bench project (`main.as`, `run.sh`, `run1.sh`, `table.py`) and `micro/`, the lane primitive costs |
| verify/threads.sh, threads.mjs | the threaded parity gate |
