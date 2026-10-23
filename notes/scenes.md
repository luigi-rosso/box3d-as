# Trajectory parity scenes and native benchmarks

Native side of the trajectory parity test. Everything here is C against the public Box3D API
(`box3d/box3d.h`, `collision.h`, `math_functions.h`) at upstream `f36d061`.

| path | what |
|---|---|
| `verify/scenes/scenes.h`, `scenes.c` | scene builders and the scripted per step input |
| `verify/scenes/traj.c` | recorder, `traj <scene> <steps> <workers> <out.bin>` |
| `verify/scenes/dump.mjs` | prints a recording, diffs two recordings |
| `verify/scenes/build.sh` | builds `bin/traj_scalar` (parity build) and `bin/traj_simd` (NEON build) |
| `verify/scenes/determinism.sh`, `determinism.mjs` | the worker and build sweep below |
| `bench/native/bench.c`, `build.sh`, `run.sh`, `table.py` | native step timings |

All scenes step at `dt = 1.0f / 60.0f` with 4 sub-steps (samples/sample.h defaults, and what
`benchmark/main.c` uses), world from `b3DefaultWorldDef()` with only `workerCount` changed. The default
contact recycle distance of the world (`B3_CONTACT_RECYCLE_DISTANCE`, 0.05) equals the samples' setting.

## Scenes

Counts are at creation. Bodies include static ones; every body is recorded in creation order.

| scene | source | bodies | shapes | joints | steps | exercises |
|---|---|---|---|---|---|---|
| `pyramid` | shared/benchmarks.c CreateLargePyramid, base 10 | 56 | 56 | 0 | 300 | box hull stacking, sleep off, warm start, graph coloring |
| `large_pyramid` | same, base 100 (benchmark size) | 5051 | 5051 | 0 | 200 | as above at 14950 contacts, one island |
| `card_house` | samples/sample_stacking.cpp CardHouse | 41 | 41 | 0 | 300 | thin rotated hulls, rolling resistance, friction 0.7, sleeps at step 56 |
| `chains` | samples/sample_joint.cpp BallAndChain + 4 humans from shared/human.c on a box (RagdollOnBox / RagdollPile) | 83 | 90 | 89 | 300 | spherical joints with motor, cone and twist limits, springs; revolute joints with limits, springs, motors; filter joints; capsule contacts; TOI |
| `capsule_mesh` | samples/sample_continuous.cpp MeshDrop | 257 | 261 | 0 | 300 | wave triangle mesh (`b3CreateWaveMesh`) with 4 hull walls, 224 capsules, 16 spheres, 16 boxes with random velocities, continuous vs mesh |
| `heightfield` | shared/determinism.c CreateWavePile (height field API as in samples/sample_mesh.cpp HeightField) | 101 | 101 | 0 | 400 | `b3CreateWave` height field, spheres, capsules, boxes, `b3CreateRock` hulls, rolling resistance, island sleep (all asleep at step 320) |
| `bullets` | samples/sample_continuous.cpp BulletVersusStack + ThinWall | 16 (+2 launched) | 16 (+2) | 0 | 150 | `isBullet` sphere launched at steps 20 and 80 (the second launch destroys the first, as `Launch()` does), 180 m/s sphere, capsule and box against a 0.2 thick wall, TOI, body destroy and id reuse |
| `mover` | samples/mover.cpp CharacterMover + samples/sample_character.cpp | 3 | 5 | 0 | 300 | `b3World_CastRayClosest` pogo, `b3World_CollideMover`, `b3SolvePlanes`, `b3World_CastMover`, `b3ClipVector`, mover pushing a dynamic box with `b3Body_ApplyLinearImpulse` |
| `many_bodies` | shared/benchmarks.c CreateManyPyramids (release sizes) | 10781 | 10781 | 0 | 100 | 14 x 14 pyramids of base 10, 28420 contacts, 196 islands, per body `enableSleep = false` |

`many_bodies` is 196 pyramids of 55 boxes, 10780 dynamic bodies plus the ground.

Recording sizes: `large_pyramid` 69 MB for 200 steps, `many_bodies` 73 MB for 100 steps, the rest 0.2 to 5 MB.

### Deviations from the source samples

- `pyramid`/`large_pyramid`: base count is a parameter (`BENCHMARK_DEBUG` picked 20 or 100).
- `chains`: the sample has no ground under the chain; the ragdoll box (extent 10) sits at z = -30 so the chain,
  swinging in the xy plane from the origin, never reaches it. Humans `i = 0..3` at
  `{ 0.2 i, 2 i, -30 + 0.1 i }`, friction torque 10, hertz 0.5, damping 0.7 (RagdollPile values),
  `groupIndex = i + 1`. `CreateHuman` is copied verbatim minus colors, body names and user data, and the
  filter joints are not stored.
- `capsule_mesh`: 16 x 16 grid instead of 32 x 32; shape by `index % 8` (6 sphere, 7 box, else capsule) instead
  of one shape type per run; capsules use rolling resistance 0.4, others 0.1 (the sample's per type values);
  seed 12345 into the scene LCG instead of `b3GetTicks()`.
- `heightfield`: `RandomVec3Uniform` jitter comes from the scene LCG (seed 52977) and `RandomQuat` is replaced by
  `b3MakeQuatFromAxisAngle( b3Normalize( random axis in [-1,1]^3 ), random angle in [-pi, pi] )`.
- `bullets`: both samples share one ground box of extent 50 (BulletVersusStack's); ThinWall's wall and
  projectiles are shifted to z = -40. The bullet is launched by script. Its -500 m/s is clamped to the world's
  400 m/s maximum linear speed.
- `mover`: GeometricMover's level is an obj file, so the level is built from boxes: ground box extent 20, a
  0.3 high step at x = 4, a ramp tilted 20 degrees about z at x = 9, a wall at x = 14 (z in [-6, 6]), plus one
  dynamic 0.4 box at (1.5, 0.4, 0.5) in the path. No shape has `MoverShapeUserData` and there is no ignore list,
  so every plane is rigid (`pushLimit = FLT_MAX`) and clips. Camera vectors are fixed: forward +x, right +z.
  The query filters keep the sample's 32 bit masks: `~2u` and `~0u` zero extend to `0xFFFFFFFD` and
  `0xFFFFFFFF` in the 64 bit `maskBits`. Script: steps 0-139 throttle (1, 0); 140-199 throttle (1, 1) with a
  jump at 170; 200-259 throttle (-1, 0) with sprint held; 260-299 no input. The mover runs before
  `b3World_Step`, as `GeometricMover::Step` calls `m_mover.Step` before `Sample::Step`.

### Notes for the AnimaScript mirror

- Scene LCG: `seed = seed * 1664525 + 1013904223` (u32 wrap), float in [lo, hi] is
  `lo + (hi - lo) * ((f32)(seed >> 8) * (1.0 / 16777216.0))`, all f32, in that order.
- `scenes.c` never relies on C evaluation order: random draws are separate statements (C leaves function
  argument and initializer list order unspecified). Mirror the statement order exactly.
- Int to float conversions are explicit casts in `scenes.c`; each one needs `<f32>` in AnimaScript.
- The C mover uses the C++ operator expressions of `mover.cpp` rewritten as `b3MulSV`/`b3Add` with the same
  association (`maxSpeed * throttle.x * forward` is `b3MulSV( maxSpeed * throttle.x, forward )`).
- Destroyed bodies stay in the scene's body list; the recorder writes them with flags 0.

## Trajectory file format (version 1)

Little endian, every field one 32 bit word, floats as raw IEEE 754 bit patterns.

Header, 16 words:

| word | field |
|---|---|
| 0 | magic `0x52543342` ("B3TR") |
| 1 | version, 1 |
| 2..9 | scene name, 32 bytes, zero padded |
| 10 | step count |
| 11 | worker count |
| 12 | time step (f32) |
| 13 | sub-step count |
| 14 | counter count C (56) |
| 15 | mover word count M (112 for `mover`, else 0) |

Then per step, after `Scene_Step` (scripted input, then `b3World_Step`):

1. step index, body count N
2. C counters, i32:
   0 bodyCount, 1 shapeCount, 2 contactCount, 3 jointCount, 4 islandCount, 5 awakeBodyCount
   (`b3World_GetAwakeBodyCount`), 6 awakeContactCount, 7 recycledContactCount, 8 staticTreeHeight,
   9 treeHeight, 10 satCallCount, 11 satCacheHitCount, 12 distanceIterations, 13 pushBackIterations,
   14 rootIterations, 15 contact begin events, 16 contact end events, 17 contact hit events,
   18 body move events, 19..42 colorCounts[24], 43..50 manifoldCounts[8].
   From 51 on they depend on the build and worker count and are skipped by `diff` unless `--all`:
   51 taskCount, 52 stackUsed, 53 arenaCapacity, 54 byteCount low, 55 byteCount high.
3. N body records of 17 words: body index, flags (bit 0 valid, bit 1 awake, bits 4..7 `b3BodyType`),
   `id.index1`, `id.generation`, position xyz, rotation quaternion xyzw (`q.v`, `q.s`), linear velocity xyz,
   angular velocity xyz. A destroyed body writes its index and 16 zero words.
4. M mover words: position xyz, velocity xyz, pogoVelocity, onGround, sprint, planeCount, total plane solver
   iterations, move loop iterations, ray hit, ray fraction, last cast fraction, reserved; then 8 plane slots of
   12 words: normal xyz, offset, pushLimit, push, clipVelocity, world point xyz, shape `index1`, shape
   `generation`. Slots past planeCount are zero. The planes are the last move iteration's, after
   `b3SolvePlanes` filled `push`.

`dump.mjs` exports `readTrajectory` and `diff` so the port's node checks can reuse them.

## Running

```sh
cd ~/Projects/box3d-rive-as
verify/scenes/build.sh                                   # bin/traj_scalar, bin/traj_simd
verify/scenes/bin/traj_scalar chains 300 1 /tmp/chains.bin
node verify/scenes/dump.mjs print /tmp/chains.bin --step=10 --body=5
node verify/scenes/dump.mjs diff a.bin b.bin             # exit 0 when identical; --all adds env counters
verify/scenes/determinism.sh                             # sweep below into verify/scenes/out (KEEP_ALL=1 keeps big files)
bench/native/run.sh                                      # REPEATS=, WORKERS=, SCENES="name:steps ..." override
```

`build.sh` scripts call `cmake --build` on the existing build dirs (configuration untouched) and compile with
`-O2/-O3 -DNDEBUG -ffp-contract=off`. `NDEBUG` is required: without it the inline `B3_ASSERT`s in the public
headers reference `b3InternalAssert`, which a Release library does not export.

## Determinism results

Every recording compared to scalar at 1 worker over all steps, all bodies, all fields, the mover state and the
51 physics counters (2026-10-23 run; load averages about 6 / 29 / 37 during the sweep).

| scene | scalar w2 | scalar w4 | scalar w8 | simd w1 | simd w2 | simd w4 | simd w8 |
|---|---|---|---|---|---|---|---|
| pyramid | = | = | = | file = | = | = | = |
| large_pyramid | = | = | = | file = | = | = | = |
| card_house | = | = | = | file = | = | = | = |
| chains | = | = | = | file = | = | = | = |
| capsule_mesh | = | = | = | file = | = | = | = |
| heightfield | = | = | = | file = | = | = | = |
| bullets | = | = | = | file = | = | = | = |
| mover | = | = | = | file = | = | = | = |
| many_bodies | = | = | = | file = | = | = | = |

`=` is bit identical state (bodies, mover, physics counters); the files differ only in taskCount, stackUsed and
byteCount, which scale with workers. `file =` is a byte identical file, environment counters included.
A second pass recorded chains, capsule_mesh, bullets, heightfield and card_house five times each with the SIMD
build at 3, 8 and 16 workers: all 75 recordings identical to scalar w1.

What this decides:

- Scalar and NEON agree bit for bit at every worker count, so the reference can be any build; we keep the
  scalar build as the stated reference since the port mirrors `simd_scalar.h`. The NEON backend only uses lane
  wise IEEE ops plus `vdivq_f32`/`vsqrtq_f32` (no estimates, and no fused multiply add in either library's
  disassembly), which is why it matches. A wasm SIMD backend built the same way (f32x4 add, sub, mul, div,
  sqrt, min, max) can keep bit parity too; `f32x4.min/max` propagate NaN and order -0 below +0 like NEON
  `fmin/fmax`, but `pmin/pmax` do not.
- Results do not depend on the worker count, so a threaded port can claim bit parity with the serial port and
  with native at any worker count, provided it keeps Box3D's task decomposition (results reduced per fixed
  block or color, not per thread).

## Native benchmark

`bench <scene> <steps> <workers> <repeats>`: each repeat builds the scene, takes one untimed step (as
`benchmark/main.c` does), times the remaining steps; ms/step is median and min over repeats, stage columns
are the mean per step of each `b3Profile` field, min over repeats. Apple M5 Max, 18 cores in two tiers
(`hw.perflevel0` 6, `hw.perflevel1` 12), so 8 workers spill past the fastest tier. The machine was
shared; load is the 1 minute average from `uptime` right before each row. Prefer the min column.

Pass 2, 9 repeats (`out/run_20261023_115017.log`), with pass 1, 5 repeats (`out/run_20261023_114719.log`) for
spread. ms per step:

| scene | build | workers | load | median | min | min, pass 1 (load) | pairs | collide | solve | constraints | transforms | refit | sleepIslands |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| large_pyramid | simd | 1 | 12.7 | 6.95 | **6.64** | 6.97 (6.7) | 0.32 | 1.20 | 5.12 | 4.82 | 0.28 | 0.01 | 0.00 |
| large_pyramid | simd | 2 | 11.9 | 3.98 | **3.88** | 5.35 (7.0) | 0.13 | 0.70 | 3.04 | 2.83 | 0.19 | 0.01 | 0.00 |
| large_pyramid | simd | 4 | 11.2 | 3.03 | **2.51** | 3.23 (7.6) | 0.08 | 0.43 | 2.00 | 1.84 | 0.14 | 0.02 | 0.00 |
| large_pyramid | simd | 8 | 11.1 | 3.75 | **2.86** | 2.57 (10.2) | 0.09 | 0.46 | 2.31 | 2.13 | 0.16 | 0.02 | 0.00 |
| large_pyramid | scalar | 1 | 12.2 | 10.76 | **10.54** | 13.69 (10.2) | 0.32 | 1.26 | 8.91 | 8.61 | 0.28 | 0.01 | 0.00 |
| large_pyramid | scalar | 2 | 11.2 | 5.75 | **4.84** | 7.97 (10.4) | 0.11 | 0.49 | 4.24 | 4.07 | 0.16 | 0.01 | 0.00 |
| large_pyramid | scalar | 4 | 10.6 | 3.40 | **3.14** | 4.26 (12.3) | 0.08 | 0.30 | 2.76 | 2.63 | 0.11 | 0.01 | 0.00 |
| large_pyramid | scalar | 8 | 10.7 | 3.13 | **2.76** | 2.67 (12.4) | 0.08 | 0.33 | 2.28 | 2.10 | 0.13 | 0.01 | 0.00 |
| many_bodies | simd | 1 | 10.9 | 13.47 | **9.99** | 13.93 (13.1) | 0.00 | 2.00 | 7.99 | 7.56 | 0.42 | 0.00 | 0.00 |
| many_bodies | simd | 2 | 10.1 | 6.52 | **5.16** | 10.11 (12.9) | 0.00 | 0.95 | 4.20 | 3.96 | 0.24 | 0.00 | 0.00 |
| many_bodies | simd | 4 | 10.0 | 4.22 | **3.93** | 7.55 (13.2) | 0.00 | 0.65 | 3.28 | 3.09 | 0.19 | 0.00 | 0.00 |
| many_bodies | simd | 8 | 9.7 | 3.04 | **2.92** | 8.74 (13.3) | 0.00 | 0.47 | 2.44 | 2.27 | 0.15 | 0.00 | 0.00 |
| many_bodies | scalar | 1 | 9.8 | 15.98 | **14.65** | 22.28 (14.2) | 0.00 | 1.94 | 12.71 | 12.31 | 0.39 | 0.00 | 0.00 |
| many_bodies | scalar | 2 | 8.8 | 10.81 | **8.36** | 12.88 (13.9) | 0.00 | 1.02 | 7.35 | 7.10 | 0.24 | 0.00 | 0.00 |
| many_bodies | scalar | 4 | 8.6 | 5.59 | **4.76** | 7.57 (13.3) | 0.00 | 0.54 | 4.23 | 4.08 | 0.15 | 0.00 | 0.00 |
| many_bodies | scalar | 8 | 8.7 | 3.90 | **3.54** | 4.53 (13.4) | 0.00 | 0.43 | 3.11 | 2.96 | 0.14 | 0.00 | 0.00 |

The full logs carry every `b3Profile` field (prepareConstraints, warmStart, solveImpulses, relaxImpulses, ...).

Upstream's `benchmark -b=large_pyramid -t=N -w=N -r=R` as a sanity check (total of steps 1..199, min over
runs):

| build | workers | upstream min ms/step, pass 2 (load) | upstream, pass 1 (load) | ours, pass 2 |
|---|---|---|---|---|
| simd | 1 | 4.10 (9.1) | 8.01 (13.4) | 6.64 |
| simd | 2 | 2.43 (8.9) | 4.95 (13.2) | 3.88 |
| simd | 4 | 1.88 (8.9) | 2.77 (13.4) | 2.51 |
| simd | 8 | 1.98 (9.3) | 1.87 (13.4) | 2.86 |
| scalar | 1 | 7.88 (9.7) | 11.10 (13.8) | 10.54 |
| scalar | 2 | 3.91 (8.9) | 6.00 (13.0) | 4.84 |
| scalar | 4 | 2.88 (8.8) | 3.65 (12.9) | 3.14 |
| scalar | 8 | 2.30 (8.8) | 2.86 (12.6) | 2.76 |

Our tool and upstream's swap places between passes, so the gap is load, not a measurement difference.
Interleaved back to back at 1 worker (load 8.8, 3 repeats each, three rounds): ours 6.56, 4.34, 4.65 ms/step,
upstream 4.44, 4.88, 4.81 ms/step. Rerun `run.sh` on a quiet machine before quoting absolute numbers; until
then the SIMD/scalar ratio (about 1.5x at 1 worker) and the worker scaling shape are the usable facts.

## Surprises

- The scalar and SIMD libraries produce byte identical recordings at 1 worker, `byteCount` included.
- The scalar build is not scalar machine code: clang autovectorizes `simd_scalar.h` lanes at `-O3` (about 3000
  `.4s` instructions in `wide4.c.o`, against about 4000 for NEON). The results are the same either way.
- `pairs` reads 0 in `many_bodies`: the pyramids barely move, so no proxy leaves its fat AABB.
- `card_house` stands and falls asleep at step 56, `heightfield` at step 320, the mover's box at step 84; the
  pyramids never sleep (sleeping is disabled in both pyramid builders).
- The bullet's 500 m/s launch is clamped to 400 m/s by `maximumLinearSpeed`.
- Under load 8 workers often lose to 4 (large_pyramid SIMD 2.86 vs 2.51 ms/step): only 6 cores are in the
  fastest tier and the other jobs on the machine take some of them.
