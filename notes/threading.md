# threading: Box3D's tasks on AnimaScript lanes

Box3D threads by handing closures to `b3EnqueueTaskCallback` and joining them with `b3FinishTaskCallback`:
`b3ParallelFor` (narrow phase, finalize bodies, pair finding, sensors, bullets), the solver's stage workers and the
tree rebuild. AnimaScript lanes share no memory and no closure crosses to one, so this pass maps the tasks whose
inputs and outputs can be serialized onto jobs and workers, keeps every result bit exact with the serial port and
native at any lane count, and measures what it costs.

## Results

Bit exact: verify/threads.sh runs all 13 trajectory scenes with the narrow phase, the solver stages and the tree
rebuild on lanes, at 1 lane, 1 lane through the serialized path, 2, 4 and 8 lanes, on workers and on jobs, and
compares every step's state hash with the native scalar recording: 130 of 130 runs pass (chains and plank_pile
take the serial solver every step, joints and the overflow color not being covered, and lanes for the rest).
verify/traj.sh, the serial gate, still passes all 13 scenes.

Scaling, whole step, ms per step (bench/threads/run.sh, session `bench/threads/out/session_20261024_134807.log`,
3 rounds interleaved with native SIMD at the same worker counts, AOT -O3, SIMD artifact, Apple M5 Max, 1 minute
load 10 to 37 from other agents, in parentheses per cell; min over rounds; x is AS serial over the cell). Configs:
`solve` the solver stages on workers, `solve+tree` plus the tree rebuild job, `all` plus the narrow phase,
`-jobs` the same on jobs.

| scene | impl | 1 lanes | 2 lanes | 4 lanes | 8 lanes |
|---|---|---|---|---|---|
| large_pyramid | AS serial | **5.006** (10) | | | |
| large_pyramid | native simd | 3.651 (10) | 2.588 (10) | 1.750 (10) | 1.208 (16) |
| large_pyramid | AS solve | 5.867 (10, x0.85) | 6.252 (21, x0.80) | 5.882 (10, x0.85) | 6.444 (10, x0.78) |
| large_pyramid | AS solve+tree | 6.056 (12, x0.83) | 5.744 (12, x0.87) | 6.123 (13, x0.82) | 6.543 (19, x0.77) |
| large_pyramid | AS all | 5.205 (13, x0.96) | 6.693 (16, x0.75) | 7.454 (15, x0.67) | 9.316 (14, x0.54) |
| large_pyramid | AS solve-jobs | 5.581 (14, x0.90) | 9.539 (13, x0.52) | 4.646 (14, x1.08) | 6.936 (16, x0.72) |
| large_pyramid | AS all-jobs | 6.427 (15, x0.78) | 16.647 (14, x0.30) | 6.709 (15, x0.75) | 8.948 (14, x0.56) |
| many_bodies | AS serial | **12.758** (14) | | | |
| many_bodies | native simd | 7.594 (20) | 5.004 (19) | 3.458 (14) | 2.112 (19) |
| many_bodies | AS solve | 10.488 (13, x1.22) | 11.036 (12, x1.16) | 10.142 (19, x1.26) | 9.866 (12, x1.29) |
| many_bodies | AS solve+tree | 10.759 (12, x1.19) | 11.269 (11, x1.13) | 10.634 (11, x1.20) | 9.976 (11, x1.28) |
| many_bodies | AS all | 10.297 (11, x1.24) | 16.341 (10, x0.78) | 15.560 (10, x0.82) | 18.073 (10, x0.71) |
| many_bodies | AS solve-jobs | 11.509 (10, x1.11) | 23.589 (19, x0.54) | 18.782 (10, x0.68) | 8.335 (19, x1.53) |
| many_bodies | AS all-jobs | 12.972 (23, x0.98) | 34.038 (21, x0.37) | 31.352 (19, x0.41) | 22.480 (18, x0.57) |
| mesh_drop | AS serial | **7.784** (16) | | | |
| mesh_drop | native simd | 5.354 (16) | 3.518 (16) | 2.201 (16) | 1.458 (10) |
| mesh_drop | AS solve | 8.374 (15, x0.93) | 7.937 (10, x0.98) | 8.155 (11, x0.95) | 8.946 (11, x0.87) |
| mesh_drop | AS solve+tree | 8.290 (10, x0.94) | 7.570 (10, x1.03) | 7.755 (10, x1.00) | 8.318 (10, x0.94) |
| mesh_drop | AS all | 7.871 (10, x0.99) | 8.151 (10, x0.95) | 7.358 (10, x1.06) | 8.036 (10, x0.97) |
| mesh_drop | AS solve-jobs | 7.581 (12, x1.03) | 8.633 (19, x0.90) | 8.019 (37, x0.97) | 8.054 (36, x0.97) |
| mesh_drop | AS all-jobs | 8.306 (11, x0.94) | 9.630 (30, x0.81) | 7.860 (28, x0.99) | 6.742 (12, x1.15) |
| village | AS serial | **3.684** (23) | | | |
| village | native simd | 2.343 (24) | 1.714 (24) | 1.479 (18) | 1.205 (24) |
| village | AS solve | 3.487 (22, x1.06) | 3.811 (20, x0.97) | 5.656 (21, x0.65) | 6.865 (15, x0.54) |
| village | AS solve+tree | 3.688 (22, x1.00) | 4.065 (21, x0.91) | 5.251 (21, x0.70) | 6.915 (20, x0.53) |
| village | AS all | 3.772 (17, x0.98) | 7.294 (23, x0.51) | 5.329 (17, x0.69) | 7.906 (16, x0.47) |
| village | AS solve-jobs | 3.562 (16, x1.03) | 4.707 (15, x0.78) | 4.608 (17, x0.80) | 6.268 (16, x0.59) |
| village | AS all-jobs | 4.199 (15, x0.88) | 5.688 (18, x0.65) | 5.462 (16, x0.67) | 6.119 (15, x0.60) |

The solver stages alone (`constraints`, the b3Solve span the lanes take), mean over the run then the fastest step,
min over rounds:

| scene | impl | 1 lanes | 2 lanes | 4 lanes | 8 lanes |
|---|---|---|---|---|---|
| large_pyramid | AS serial | **3.721** / 3.496 | | | |
| large_pyramid | native simd | 2.768 | 1.859 | 1.193 | 0.827 |
| large_pyramid | AS solve | 4.302 / 3.603 (x0.86) | 4.508 / 3.681 (x0.83) | 3.774 / 2.950 (x0.99) | 4.133 / 3.268 (x0.90) |
| large_pyramid | AS solve+tree | 4.486 / 3.843 (x0.83) | 4.083 / 3.175 (x0.91) | 3.924 / 2.880 (x0.95) | 4.229 / 3.142 (x0.88) |
| large_pyramid | AS all | 3.873 / 3.476 (x0.96) | 3.667 / 3.261 (x1.01) | 3.568 / 2.991 (x1.04) | 3.973 / 3.054 (x0.94) |
| large_pyramid | AS solve-jobs | 4.087 / 3.474 (x0.91) | 7.649 / 6.111 (x0.49) | 3.253 / 2.734 (x1.14) | 4.529 / 3.245 (x0.82) |
| large_pyramid | AS all-jobs | 4.569 / 3.783 (x0.81) | 8.593 / 7.072 (x0.43) | 3.414 / 3.068 (x1.09) | 4.009 / 3.176 (x0.93) |
| many_bodies | AS serial | **9.452** / 7.637 | | | |
| many_bodies | native simd | 6.020 | 3.823 | 2.546 | 1.550 |
| many_bodies | AS solve | 8.126 / 7.572 (x1.16) | 8.501 / 7.328 (x1.11) | 6.688 / 5.770 (x1.41) | 5.948 / 4.758 (x1.59) |
| many_bodies | AS solve+tree | 8.278 / 7.522 (x1.14) | 8.627 / 7.042 (x1.10) | 7.024 / 5.851 (x1.35) | 5.941 / 4.505 (x1.59) |
| many_bodies | AS all | 7.957 / 7.473 (x1.19) | 9.294 / 7.865 (x1.02) | 7.041 / 5.777 (x1.34) | 6.447 / 5.409 (x1.47) |
| many_bodies | AS solve-jobs | 8.861 / 7.822 (x1.07) | 19.267 / 15.639 (x0.49) | 15.089 / 13.007 (x0.63) | 5.455 / 4.795 (x1.73) |
| many_bodies | AS all-jobs | 9.933 / 7.891 (x0.95) | 17.605 / 14.899 (x0.54) | 15.876 / 13.227 (x0.60) | 10.179 / 8.584 (x0.93) |
| mesh_drop | AS serial | **2.111** / 0.059 | | | |
| mesh_drop | native simd | 1.952 | 1.404 | 0.846 | 0.493 |
| mesh_drop | AS solve | 2.281 / 0.065 (x0.93) | 1.912 / 0.053 (x1.10) | 1.679 / 0.051 (x1.26) | 1.636 / 0.111 (x1.29) |
| mesh_drop | AS solve+tree | 2.386 / 0.065 (x0.88) | 1.853 / 0.068 (x1.14) | 1.553 / 0.050 (x1.36) | 1.499 / 0.113 (x1.41) |
| mesh_drop | AS all | 2.237 / 0.066 (x0.94) | 1.916 / 0.060 (x1.10) | 1.593 / 0.061 (x1.33) | 1.846 / 0.130 (x1.14) |
| mesh_drop | AS solve-jobs | 2.089 / 0.061 (x1.01) | 2.044 / 0.089 (x1.03) | 1.600 / 0.081 (x1.32) | 1.478 / 0.095 (x1.43) |
| mesh_drop | AS all-jobs | 2.364 / 0.067 (x0.89) | 2.529 / 0.126 (x0.83) | 2.092 / 0.151 (x1.01) | 1.419 / 0.229 (x1.49) |
| village | AS serial | **1.194** / 0.309 | | | |
| village | native simd | 0.910 | 0.690 | 0.573 | 0.429 |
| village | AS solve | 1.148 / 0.323 (x1.04) | 1.387 / 0.276 (x0.86) | 2.242 / 0.248 (x0.53) | 3.518 / 0.239 (x0.34) |
| village | AS solve+tree | 1.271 / 0.323 (x0.94) | 1.563 / 0.334 (x0.76) | 2.261 / 0.257 (x0.53) | 3.608 / 0.253 (x0.33) |
| village | AS all | 1.300 / 0.344 (x0.92) | 2.292 / 0.380 (x0.52) | 2.055 / 0.245 (x0.58) | 3.719 / 0.234 (x0.32) |
| village | AS solve-jobs | 1.200 / 0.343 (x1.00) | 1.880 / 0.353 (x0.64) | 1.873 / 0.292 (x0.64) | 2.970 / 0.282 (x0.40) |
| village | AS all-jobs | 1.415 / 0.346 (x0.84) | 1.847 / 0.367 (x0.65) | 2.044 / 0.288 (x0.58) | 2.527 / 0.291 (x0.47) |

The narrow phase (`collide`), where `all` puts it on lanes:

| scene | impl | 1 lanes | 2 lanes | 4 lanes | 8 lanes |
|---|---|---|---|---|---|
| large_pyramid | AS serial | **0.703** / 0.550 | | | |
| large_pyramid | native simd | 0.486 | 0.486 | 0.350 | 0.228 |
| large_pyramid | AS all | 0.797 / 0.652 (x0.88) | 2.466 / 2.069 (x0.29) | 3.228 / 2.208 (x0.22) | 4.641 / 3.318 (x0.15) |
| large_pyramid | AS all-jobs | 1.255 / 0.732 (x0.56) | 7.457 / 6.208 (x0.09) | 2.686 / 2.409 (x0.26) | 4.065 / 2.610 (x0.17) |
| many_bodies | AS serial | **2.602** / 1.617 | | | |
| many_bodies | native simd | 1.170 | 0.930 | 0.701 | 0.413 |
| many_bodies | AS all | 1.798 / 1.413 (x1.45) | 6.450 / 5.292 (x0.40) | 7.811 / 6.003 (x0.33) | 10.850 / 8.497 (x0.24) |
| many_bodies | AS all-jobs | 2.368 / 1.723 (x1.10) | 15.797 / 13.001 (x0.16) | 14.754 / 12.900 (x0.18) | 11.576 / 10.061 (x0.22) |
| mesh_drop | AS serial | **2.392** / 0.000 | | | |
| mesh_drop | native simd | 1.567 | 1.081 | 0.647 | 0.410 |
| mesh_drop | AS all | 2.567 / 0.000 (x0.93) | 3.093 / 0.000 (x0.77) | 2.421 / 0.000 (x0.99) | 2.521 / 0.000 (x0.95) |
| mesh_drop | AS all-jobs | 2.808 / 0.000 (x0.85) | 3.954 / 0.000 (x0.60) | 2.781 / 0.000 (x0.86) | 1.958 / 0.000 (x1.22) |
| village | AS serial | **1.170** / 0.000 | | | |
| village | native simd | 0.640 | 0.460 | 0.320 | 0.207 |
| village | AS all | 1.243 / 0.000 (x0.94) | 3.244 / 0.000 (x0.36) | 1.929 / 0.000 (x0.61) | 2.645 / 0.000 (x0.44) |
| village | AS all-jobs | 1.438 / 0.000 (x0.81) | 2.320 / 0.000 (x0.50) | 2.065 / 0.000 (x0.57) | 2.113 / 0.000 (x0.55) |

What the tables say:

- The step does not get faster. The best whole step results are 1.2 to 1.5x on many_bodies, which the serial
  port's own run to run spread (about 20%) half covers, about 1.0 to 1.3x on mesh_drop, and none on large_pyramid
  and village. Native SIMD gets 3.0x (large_pyramid), 3.6x (many_bodies), 3.7x (mesh_drop) and 1.9x (village)
  from 1 to 8 workers.
- The solver stages scale a little where few bodies are shared: many_bodies 1.4x at 4 lanes and 1.6 to 1.7x at
  8, mesh_drop 1.3 to 1.5x, large_pyramid 1.0 to 1.1x at 4 lanes. Village, with 10 to 18% of its bodies shared,
  gets slower with every lane (0.3x at 8). Native: 3.3x, 3.9x, 4.0x and 2.1x on the same stage.
- The narrow phase on lanes loses everywhere but one cell (mesh_drop jobs at 8 lanes, 1.2x): 0.15 to 0.3x on the
  pyramids, whose 15000 and 28000 contacts are all recycled at about 50 ns each, while each contact's state round
  trip costs the main lane alone about 180 ns.
- The tree rebuild job takes 0.3 ms off `pairs` on mesh_drop (0.65 to 0.33) and 0.1 to 0.2 ms on village for 0.06
  to 0.09 ms of main lane marshalling; the pyramids rebuild little.
- Jobs and workers trade places. Jobs start faster (no parked worker to wake), so they win the solver at 4 lanes
  on large_pyramid (1.14x against 0.99x) and at 8 on many_bodies (1.73x); but a job whose replica passes 16 MB is
  instantiated afresh every run (below), which makes many_bodies at 2 and 4 lanes two to three times slower than
  serial, and jobs decode the shape snapshot every run, which sinks `all-jobs`.

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
| large_pyramid | 4.60 / 1.08 | 1.04 + 0.59 | 9.23 / 4.41 | 2.00 + 0.78 | 0.70 | 1.40 | 0.04 + 0.03 |
| many_bodies | 8.86 / 2.12 | 2.25 + 1.17 | 17.69 / 8.38 | 4.85 + 2.00 | 2.60 | 3.14 | 0.22 + 0.19 |
| mesh_drop | 2.68 / 0.59 | 0.55 + 0.19 | 3.72 / 2.43 | 1.17 + 0.44 | 2.39 | 2.10 | 0.05 + 0.03 |
| village | 2.14 / 0.36 | 0.49 + 0.18 | 3.19 / 1.46 | 1.07 + 0.34 | 1.17 | 1.40 | 0.04 + 0.03 |

The whole round trip, measured alone by sending the main lane's own partition through the serialized path with
one lane (`REPLICA=1`), so that nothing runs in parallel and the difference to the in place run is all
marshalling:

| scene | solver stages in place ms | through the replica ms | pack (incl. blob) | lane unpack | lane answer | apply | MB in / out |
|---|---|---|---|---|---|---|---|
| large_pyramid | 3.60 to 3.65 | 5.72 to 6.16 | 0.71 | 0.73 | 0.50 | 0.61 | 6.01 / 1.45 |
| many_bodies | 7.41 to 9.47 | 14.04 to 15.96 | 1.96 | 2.16 | 1.50 | 1.57 | 11.79 / 2.83 |
| mesh_drop | 2.09 to 2.19 | 3.33 to 3.64 | 0.57 | 0.43 | 0.18 | 0.21 | 3.87 / 0.86 |
| village | 1.12 to 1.25 | 1.65 to 1.96 | 0.25 | 0.20 | 0.11 | 0.11 | 2.61 / 0.50 |

So the solver's data costs 60 to 85% of the solver's own work to move once each way, 0.34 to 0.49 ms per MB, of
which the bulk copies are about 0.02 to 0.03 ms per MB (bench/threads/micro: `memory.copy` 22.7, `Blob.fromArray`
34, `readInto` 22.6, region `writeFrom` 29 and `readInto` 24.7 us per MB at 8 MB). The rest is per record work:
walking `world.contacts` to each contact object and its separately allocated manifold array, and on the lane
filling contact objects and sims back in.

### Why the colored stages cannot scale here

A step runs 84 colored stages (7 active colors, warm start, solve and relax over 4 sub-steps), and after each the
lanes that share a body swap its velocity. large_pyramid's solver takes about 3.6 ms serially, so a color stage
holds about 40 us of work: 10 us per lane at 4 lanes, 5 at 8. One exchange wait costs 7.5 us at 4 lanes (108 waits,
0.81 ms per step) and 9.8 at 8 (156 waits, 1.53 ms), mostly spent waiting for the slowest neighbor to arrive, then
two lock and copy calls of 0.2 to 0.5 us each. At 4 lanes the sync costs as much as the work it separates; at 8,
twice as much. Village has 353 waits per step at 8 lanes (2.25 ms) against a 1.2 ms solver.

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
them, would ship each stage's inputs and outputs per stage. At the measured 0.34 to 0.49 ms per MB of round trip:

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

- Shared linear memory between lanes. The marshalling above, 1.3 to 3.4 ms of the main lane's own time per step at
  4 lanes for the solver and 2.4 to 6.9 ms more for the narrow phase, plus the lanes' unpacking, goes away, and
  Box3D's own scheduler runs unchanged: the enqueue callback hands the closure to a lane that sees the same memory.
  The stage syncs become wasm atomics (tens of ns) instead of host call atomics, waits and copies (7 to 10 us).
  With the marshalling gone and the sync at native cost, the solver stages would follow native's 2.3x at 4 workers
  and 3.3x at 8 (large_pyramid) minus the port's own ratio, and the narrow phase, which native takes from 0.49 to
  0.23 ms on large_pyramid and 1.57 to 0.41 on mesh_drop, would scale instead of losing 3 to 6x.
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
- Cost: moving the solver's data once each way costs 60 to 85% of the solver's own work (the replica table above:
  large_pyramid 2.5 ms of marshalling for 3.6 ms of solving, many_bodies 7.2 for 7.4 to 9.5), and the main lane's
  share alone (pack and apply, 1.3 to 3.4 ms at 4 lanes) is serial, an Amdahl ceiling of about 2 to 3x on the
  solver before any sync. The narrow phase's round trip costs the main lane 2.8 ms on large_pyramid for 0.7 ms of
  work. Each colored stage's exchange costs 7.5 to 10 us per wait, against 5 to 10 us of work per lane. Code: 2950
  lines of lane code, about 1200 of them serialization (contact state, section and answer codecs, the shape and
  geometry snapshot, the tree codec), with 137 raw load, store and memory.copy sites.
- Proposal: wasm threads style shared memory for a job or worker team (one memory, instances per thread, `memory.atomic.*`),
  so a task callback can hand its closure to a lane as C does; Box3D's scheduler would then run unchanged. Without
  the object graph, a typed array view over a `SharedRegion` that lanes read and write in place with the region's
  atomics would take the bulk copies (about 10% of the round trip) and the exchange's lock and copy calls (0.2 to
  0.5 us per neighbor), not the per record packing.
- What it would buy: the marshalling and most of the sync, so the solver could follow native's 2.3x at 4 workers
  and 3.3x at 8 instead of 1.0 to 1.7x, and the narrow phase could scale instead of losing 3 to 6x.

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
  time. Our solver replica for many_bodies at 2 and 4 lanes is past it: the jobs backend runs the solver at 0.49 and
  0.63x serial there and 1.73x at 8 lanes, where each replica is small (its first wait at 4 lanes is 6.2 ms against
  0.0004 on workers). Collide jobs decode the shape snapshot every run; `all-jobs` loses to `all` on every scene.
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
