# Demo: debug draw and app shell

A rive-cli project that runs Box3D scenes from `lib/` and draws the solids
instanced on the GPU with a depth buffer, through AnimaScript's GPU API and
WGSL, into a canvas composited under vector overlays drawn with the Rive
renderer. The 2D painter that came before it, 3D shapes flattened to paths
and painted back to front, stays behind `--define=VECTOR_DRAW` and as the
fallback when the host has no GPU canvas. The panels around the view are RML,
bound to a view model the script writes.

## Files

| File | What it holds |
|---|---|
| `rive.yaml` | project; `main: Box3D`, `scripting: wasm`, `dangerouslyFast: true`; compiles `lib/`, excludes `demo/gen`, `lib/parallel`, `verify`, `bench`, `notes` |
| `main.as` | the `Main` layout: steps and draws the scene, camera and pointer input, keys, follows the view model |
| `demo/shell.rml` | generated, do not edit: the `Box3D` artboard, its panels, state machine and the `Demo` view model |
| `demo/gen/shell.py` | writes `demo/shell.rml`: theme, layout, icons, controls, the view model |
| `demo/gen/rml.py` | the RML DSL it uses: elements with explicit ids, view models, binds, listeners, layers |
| `demo/shell.as` | `Shell`, the script's handle on the `Demo` view model |
| `demo/vec.as` | `Vec3`, `Quat`, `Xf` (p + q), `Ray`, f32 helpers |
| `demo/camera.as` | `OrbitCamera`: yaw, pitch, distance, target, perspective framed in the clear area, rays, `shiftTarget` for following |
| `demo/debug_draw.as` | `DebugDraw` plus `DebugHull`, `DebugMesh`, `DebugHeightField`; the vector painter and overlays |
| `demo/gpu_draw.as` | `GpuDraw`, the instanced GPU renderer, plus `GpuMesh` and `MeshWriter` |
| `demo/scene3d.wgsl` | its shaders: faceted solids, spheres and capsules, their shadow casters, the shadow filters, blob shadows (asset name `demo/scene3d`) |
| `demo/shadows.as` | `ShadowCascades`, the sun's cascade fit, and `ViewSpan`, the view depths a frame's solids span |
| `demo/scene.as` | `DemoScene` and `SceneStats`, the contract for scenes |
| `demo/b3view.as` | Box3D to DebugDraw: transform conversion, `ShapeCache`, `BodyView`, `fieldHeight` |
| `demo/physics.as` | `PhysicsScene`, the Box3D base every scene extends |
| `demo/human.as` | `createHuman`, the ragdoll from shared/human.c via verify/scenes/scenes.as |
| `demo/mover.as` | `CharacterMover`, samples/mover.cpp |
| `demo/scenes.as` | the eight scenes and `makeScenes()`, the registry the shell reads |
| `demo/threads.as` | `ThreadsState` (`g_threads`): which backends and thread counts this runtime has, putting the Threads scene's world on one, the step history |
| `demo/threads_graph.as` | `ThreadsGraph`, the script in the Threads panel that draws the timeline and the step history |
| `demo/fonts/InterVariable.ttf` | Inter variable (OFL, license beside it), used at weight 300 and 420 with tabular figures |

`dangerouslyFast` is on because the live window otherwise plays the fueled
debug bake, whose 200 ms script budget a single physics step can exceed (the
first capture after re-including `lib/` trapped in `advance`). The cost is no
breakpoints, and a runaway loop hangs instead of trapping.

## Running

```sh
RIVE=~/Projects/rive-master/packages/rive-cli/out/release/rive
cd ~/Projects/box3d-rive-as
WAMRC=~/Projects/rive-parallel-calls/wamrc-build/wamrc
RIVE_WAMRC=$WAMRC RIVE_WASM_AOT_SYNC=o3 $RIVE . --aot                  # live window, AOT
$RIVE . --verify                                                       # compile only
$RIVE . --screenshot=/tmp/x.png --define=START_SCENE=3 --advance=120
RIVE_WAMRC=$WAMRC RIVE_WASM_AOT_SYNC=o3 $RIVE . --aot \
    --screenshot=/tmp/x.png --define=START_SCENE=1 --define=STEP_LOG --advance=600
```

Run the live window on AOT. The WAMR interpreter is 30 to 40 times slower here
(the awake pyramid steps in 165 ms interpreted, 4.5 ms on AOT), which is fine
for captures but not for play. The AOT compile takes about a minute.

`START_SCENE` (1 based) picks the scene at launch. `VECTOR_DRAW` draws with
the 2D painter instead of the GPU. `NO_SHADOWS` starts the GPU renderer with
its shadow maps off and blob shadows in their place, and `CSM_DEBUG` tints
each shadow cascade (red, green, blue, yellow, near to far). `DRAW_LOG` logs the mean and max script
draw time per 120 frames with the triangle (or, painted, face) count; use it
with `--bench`, which draws every frame where captures draw only the last. `STEP_LOG` logs the mean
and max step time once per simulated second (`console :0 step <scene> s<n>
mean <ms> max <ms>`), which is how the table below was measured; `--bench`
refuses `--key` and reports frames, not steps.

Captures that a live window is watching rebuild `build/` under it; capture
from an rsync mirror of the project instead (the one used here is
`<scratchpad>/demo3d/cap.sh`, one mirror per capture so they can run in
parallel).

## Scenes

| # | Scene | From | Bodies | Step, AOT |
|---|---|---|---|---|
| 1 | Pyramid | shared/benchmarks.c CreateLargePyramid, base 20 (210 boxes) | 211 | 4.5 ms mean, 18.8 max over the first second; asleep after 2 s |
| 2 | Card house | sample_stacking.cpp CardHouse, 2 mm cards | 41 | 1.0 ms mean while settling, then asleep |
| 3 | Ragdolls and chain | sample_joint.cpp BallAndChain (8 links, 1 m ball) through six shared/human.c ragdolls | 83, 93 joints | 0.3 to 0.9 ms mean, 2.9 max |
| 4 | Rain on mesh | sample_continuous.cpp MeshDrop: 40 by 40 wave mesh, walls, a capsule or box every third step, oldest recycled past 260 | up to 261 | rises to 1.0 ms mean at 200 bodies, 5.7 max |
| 5 | Height field | b3CreateWave 41 by 41 with spheres, 14 sided wheels, capsules and rocks rolling from the peaks; a static baked compound (6 capsule posts, 2 box hulls, a sphere dome) in the middle; a body every 40 steps up to 110, the fallen respawn over a peak | 50 to 112 | 0.6 to 1.0 ms mean, 7.5 max |
| 6 | Bullets | sample_continuous.cpp BulletVersusStack (500 m/s bullet at step 30) beside ThinWall (sphere, capsule and box at 180 m/s every 150 steps against a 20 cm static wall) | 17 | 0.1 to 0.26 ms mean |
| 7 | Character | samples/mover.cpp on a level: a 30 cm step, an 18 degree ramp to a ledge, a long wall in 2 m blocks, a short wall, eight 20 cm treads to a landing, five crates | 7 | 0.003 to 0.035 ms |
| 8 | Threads | a walled pit of 1365 balls and 683 capsules stirred by a kinematic paddle, sleep off; see The Threads scene below | 2051 | 2.2 to 2.7 ms serial, 1.0 ms on 4 parallel parts |

Times are `STEP_LOG` means per simulated second over 600 frames, release AOT
bake (`aot -O3`), Apple M5 Max, one scene at a time, from the commit that adds
this table. The step includes the scene's own per step work (spawning, the
mover's queries).

## Controls

| Input | Action |
|---|---|
| drag | orbit; a fling keeps spinning and eases out |
| Shift drag | pan |
| wheel, trackpad, pinch | zoom; trackpad sideways also orbits |
| press a dynamic body (Pick tool) | grab it with a motor joint to a kinematic mouse body; release throws it |
| click (Shoot tool) | shoot a bullet sphere along the pointer ray |
| Space or B | shoot from the screen center, unless the scene uses Space |
| P / O / R | pause, single step (pauses), reset |
| Tab | switch Pick and Shoot |
| 1-8 | open a scene; the list on the left does the same |
| F | ease back to the scene's home view |
| G | grid on or off |
| H | shadow quality: high, soft, low, off (blob shadows); logs the one picked |
| C | contact points and normals on or off (off by default) |
| J | joints on or off (on by default) |
| WASD, arrows | walk the character, relative to the camera |
| Space, Shift | jump, run (Character) |

Headless digits need names: `--key=key3`, not `--key=3`, which sends key code 3.
Held keys: `--key=w:down --advance=60`.

## The UI shell

The panels are RML: `demo/shell.rml`, generated by `demo/gen/shell.py`. Edit
the generator, never the RML, and rerun it from the repo root:

```sh
python3 demo/gen/shell.py
```

Every element carries an explicit id, so rive never writes ids back into the
file and a rebuild is deterministic. The ids are renumbered whenever the
generator changes, which is fine for a file nobody pushes.

### Structure

The `Box3D` artboard (`main: Box3D`) lays out to the window. Its children,
front to back (in RML an earlier sibling draws on top):

| Element | What it is |
|---|---|
| `Overlay` | absolute at 0, 0, sized `uiWidth` by `uiHeight` and scaled by `uiScale`, so it is laid out in UI units and drawn at the window size; holds `Panels` and the scrims |
| `Panels` | absolute, inset 28 (18 on the left, where the rows reach past the margin): a column of `Top`, `Touch Help` and `Bottom` |
| `Top` / `Left` | title block, then the eight scene rows (196 by 26, 4 apart) or, compact, the `Chips` (eight 34 by 26 chips and the open scene's name), then a 24 point rule and the wrapped description |
| `Top` / `Stat Line` | compact only: STEP, DRAW, BODIES, AWAKE as runs of one text |
| `Top` / `Clear` | a fill box between the panels; its computed bounds are bound back into `clearX`.. `clearHeight` |
| `Top` / `Right Room` | wide only: an empty 168 column that keeps `Clear` off the right panel |
| `Top` / `Right` | absolute in the top right corner, 168 wide: STEP and DRAW, a rule, the six counts, a rule, B3 PROFILE with its stacked bar and one row per part |
| `Threads` | only while the Threads scene is open (the `Threads Panel` layer on `threadsOpen`): the backend and thread count choices with a note under them, and `Graph`, a fill box holding a second `ScriptedLayout` running `demo/threads_graph.as`; a row 190 tall, or in compact a column 290 tall with the two choices side by side |
| `Touch Help` | compact only: drag to orbit, pinch to zoom, tap a body to pick |
| `Bottom` | pause, step, reset (34 square), the Pick and Shoot pair in one outline, the PAUSED tag, and the key help, right aligned, filling the rest and wrapping upward when narrow (each hint holds together with no break spaces) |
| `Scrim Left`, `Scrim Right`, `Scrim Top`, `Scrim Bottom` | gradients that keep the panels legible over the scene; the side pair outside compact, the top and bottom pair in it; they scale with the overlay |
| `Stage` / `Demo` | a fill box holding the `ScriptedLayout` running `main.as`, with the `FocusData` that keys need |

The icons are `PointsPath`s in a `Node` at the button center, so they keep
their drawn size; the reset arc and its head are computed in `reset_paths()`
from the same numbers the hand drawn version used. The bar is seven `fill`
boxes whose `fractionalWidth` is bound to the part times, multiplied by 1e6
first because Yoga shares out only part of the free space when the weights sum
below 1.

Every control (the rows, the three transport buttons, Pick and Shoot) gets one
state machine layer with idle, hover and, where it has one, active states,
crossfading over 120 ms; a state keys the colors of its fill, hairline and ink
(and for pause, which icon and whether PAUSED shows). Three listeners each:
`enter` and `exit` write its `_hover<Name>` boolean, `down` performs its action
and raises `_uiPress`. The chips are controls of their own beside the rows,
sharing `scene`.

### Size classes

On `resize` the script picks a class and a scale (`sizeClass` and `uiScaleFor`
in `demo/shell.as`, window size in points):

| Class | `layoutClass` | When | Shows |
|---|---|---|---|
| wide | 0 | width at least 1100 and at least 1.2 times the height | everything, as above |
| medium | 1 | otherwise | the rows; STEP, DRAW, BODIES and AWAKE, floating in the top right corner (absolute, so the view takes the width under it); no description, profile or the other counts |
| compact | 2 | width under 720, or portrait | chips and the open scene's name across the top, the stat line, touch hints over the transport and tools; no rows, right panel or key help |

The UI scale is the window's smaller side over 800, clamped to 0.85 to 1.3.
The overlay is laid out at the window size divided by it and scaled back up,
so text stays vector sharp; the class is picked from the unscaled size.

One state machine layer, `Size`, has a state per class, entered with no
crossfade when `layoutClass` changes. Its states key what `per_class` and
`shown_in` gathered in the generator: `displayValue` to show and hide boxes
(a hidden box is out of hit testing too) and `Top`'s `flexDirectionValue`
(row, or column in compact). `Right` is absolute in every class rather than
keyed absolute in medium: a box keyed to absolute in the frame it comes back
from `display: none` is laid out at the left edge, which put the medium stats
over the title after a resize out of compact. Wide holds its column open with
`Right Room` instead.

### Camera fit

The home views were tuned for a 1280 by 800 window at a vertical field of 0.75
radians. The camera now fits a fixed cone, half tangents 0.40 across and 0.34
up (`FIT_X`, `FIT_Y` in `demo/camera.as`), into the `Clear` box: the focal
length is the smaller of half its width over `FIT_X` and half its height over
`FIT_Y`, so a narrow or portrait area holds the horizontal field and a wide one
the vertical, and the principal point is the box's center. That is an off
center projection, so orbit still pivots on the target and rays, pan and the
Space shot all go through the same center. At 1280 by 800 the focal length is
976, against 1018 before.

That cone only promises what the 1280 by 800 tuning showed, and a scene wider
or taller than it spilled out of `Clear` and under the panels: the Character
level's ramp under the wide stats, the chain's anchor off the top of a phone.
So the fit is four tangents, left, right, below and above the target, which
`animateTo` resets to the cone and `include(lo, hi)` widens until a world box
seen from the home view lies inside. The focal length fits their spans into
`Clear` and the principal point sits where it centers them, so the target is
off the box's center when the content is lopsided. Ragdolls and chain includes
the pile and the anchor (and its target rose to 5 m), Character the whole
level (from 22 m, its target 4 m right of the capsule), Threads the pit's walls.
A scene that fits the cone keeps its framing.

The script reads the bounds every `advance` and calls `camera.frame()`. The
bound `computedRootX/Y` already include the overlay's scale, but
`computedWidth/Height` are in its unscaled units, so the script multiplies
those by `uiScale`. Until the layout first runs the camera frames the whole
viewport.

### The view model contract

`Demo`, bound to the artboard. `demo/shell.as` (`Shell`) holds the script's
handles; the generator and it name the same properties.

| Property | Type | Written by | Meaning |
|---|---|---|---|
| `scene` | number | rows, keys 1-7 | index of the open scene; the active row follows it |
| `paused` | boolean | pause button, P, step | the pause button and PAUSED follow it |
| `tool` | number | Pick, Shoot, Tab | 0 Pick, 1 Shoot |
| `stepOnce`, `reset` | trigger | step and reset buttons | the script listens with `addListenerWith`; O and R call the same handlers |
| `sceneName0`..`sceneName7` | string | script, once | row titles from `DemoScene.name()` |
| `sceneTitle` | string | script | the open scene's name, under the chips |
| `description` | string | script | the open scene's `hint()` |
| `stepMs`, `drawMs`, `profileMs` | string | script | formatted with `formatMs` (3 decimals under 0.1 ms) |
| `bodies`, `awake`, `contacts`, `joints`, `islands`, `faces` | number | script | shown through `DataConverterToString` |
| `profilePairs` .. `profileOther` | number | script | ms per part, shown to 2 decimals and weighting the bar |
| `threadsOpen` | boolean | script | the Threads scene is open, which shows its panel |
| `backend`, `threads` | number | Threads panel, script | the backend (0 serial, 1 lanes, 2 parallel) and thread count asked for; the script writes back what the world steps on |
| `_okLanes`, `_okParallel`, `_okThreads2`, `_okThreads4`, `_okThreads8` | boolean | script | whether each choice is there to take; while false a dim stand in shows instead |
| `threadsNote` | string | script | what the backend does, and why the choices that are off are off |
| `layoutClass` | number | script, on resize | 0 wide, 1 medium, 2 compact |
| `uiScale`, `uiWidth`, `uiHeight` | number | script, on resize | the overlay's scale and its size in UI units |
| `clearX`, `clearY`, `clearWidth`, `clearHeight` | number | the layout | `Clear`'s computed bounds, written back by binds with `direction` |
| `_uiPress` | boolean | every control's `down` | a press a control took; the script clears it and ignores the press |
| `_hover<Name>` | boolean | `enter` / `exit` | drives the hover state of one control |

The script binds lazily (the view model can arrive after `init`), then each
`advance` reads `scene`, `paused` and `tool` back, so a click and a key end in
the same state. Keys write the view model through `syncShell()`. The stats are
written from `advance` four times a second.

### Input

A listener in front of the script does not stop the script seeing the press:
the script is handed every pointer event in the artboard. Marking the controls
opaque would turn every move or release over them into `pointerExit` for the
script, so an orbit dragged across a panel would end there. Instead a control's
`down` listener raises `_uiPress`, which the hit walk (front to back) sets
before the script's `pointerDown` runs, and the script drops that press. Moves
and releases still reach it, so drags pass over the panels.

### Captures

`--advance` does not draw the frames it skips, and a value written in `draw`
reaches its binds only on the next advance, so the script writes the overlay
from `advance`. DRAW and FACES are measured in `draw` and so read 0 in a
headless capture; they are live in the window.

## The Threads scene

Scene 8 shows the threaded step as it runs. The pit is 16 by 16 by 8 bodies (every third a capsule) in 2 m wall
blocks, stirred by a kinematic paddle at 0.45 rad/s, with sleep off so every step has about the same work: about
11500 contacts once settled. Bodies the paddle flings over the wall drop back in once a second, so the count stays
put. The scene sets `crowd`: at 1176 bodies the spin rings cost 4 ms of a 10.5 ms frame and the shadows 2, more than
the balls themselves, and a pile of 864 boxes was no better, its overlapping faces taking 18 ms a frame to order
(7 after the sweep above). With both off, 2048 bodies draw in about 7.3 ms on AOT.

The panel under the view (`Threads` above) picks the backend and the thread count; the script follows it between
steps (`ThreadsState.apply`) and writes back what the world steps on:

| Backend | What runs where | Threads |
|---|---|---|
| Serial | everything on the main thread | 1 |
| Lanes | `b3World_EnableLanes` on jobs with collide, solve and the tree: the narrow phase and the solver stages on copies of what they read, partition 0 on the main lane, the tree rebuild on a job | 1 to `Jobs.threads() + 1`, at most 8 |
| Parallel | `b3GetParallelBackend().enable` with collide and solve: parallel calls over the world's own memory, capped at the thread count by `limit` | 1 to `Parallel.size()`, at most 8 |

Choices it cannot take show dim, with the reason in the note under them. What each place offers, detected at run
time from `Jobs.threads()`, `b3GetParallelBackend()` and its `size()`:

| Where | Serial | Lanes | Parallel |
|---|---|---|---|
| master rive-cli | yes | 1, 2, 4 (8 with `--job-threads=7`) | off: needs the parallel-calls toolchain and lib/parallel |
| parallel-calls rive-cli, as checked in | yes | the same | off, the same, since rive.yaml leaves out `lib/parallel` |
| parallel-calls rive-cli, `lib/parallel` included | yes | the same | 1, 2, 4 (`Parallel.size()` is the 6 fast cores; 8 with `--job-threads=7`) |
| a browser runtime without threads, such as the live demo's | yes | off: this runtime has no threads | off, the same |

It opens on the best backend there is, at up to 8 threads. Leaving the scene (`leave()`) puts the world back on the
serial path and turns tracing off, since the lane hooks are global and would otherwise reach the next scene's
world.

The graphs are `ThreadsGraph` (`demo/threads_graph.as`), a script in the panel's layout box, so it draws over the
scrims, scales with the overlay and gets its size from the layout; it reads `g_threads` and the trace, which it
shares with `main.as` as module globals:

- **Latest step, by thread**: a row per thread (main, then lane 1.. and the tree job, or part 1..), each span of
  `lib/trace.as` placed by when it ran in the step, in the profile panel's tones (pairs, collide, solver, finalize,
  tree) and the copying to and from lanes in the accent. The axis is ms and eases toward the latest steps' length;
  a mark shows where the step ended. Rows are 16 points or less; the legend shows the groups the step has.
- **Step time**: ms per step over the last 300 steps, in the backend's ink (serial grey, lanes accent, parallel
  white), a mark wherever the backend or thread count changed, and once a serial step has been seen this session,
  a dashed line at it and the speed up against it.
- In compact the graph sits under the choices (which go side by side), the legend goes and the history keeps only
  its figures.

How it records is in notes/threading.md (Tracing). Each span is two `clock()` reads and three stores: about 180
spans per row per step here, so 360 reads on serial and 1460 on 4 parts. Turned off in the scene, the same bakes
read 2.30 to 2.73 against 2.24 to 2.70 ms serial and 0.97 to 1.26 against 1.01 to 1.34 ms on 4 parts, inside the
machine's noise; the difference is at most about 0.05 ms. The other scenes never turn it on.

Ms per step, `STEP_LOG` means over seconds 6 to 10, min over 2 to 3 interleaved rounds, release AOT bake (`aot
-O3`), Apple M5 Max, shared with other agents' runs (1 minute load 21 to 37):

| Toolchain | Serial | Lanes 1 | Lanes 2 | Lanes 4 | Lanes 8 | Parallel 1 | Parallel 2 | Parallel 4 | Parallel 8 |
|---|---|---|---|---|---|---|---|---|---|
| master | 2.24 | 2.57 | 4.05 | 4.14 | 7.74 | | | | |
| parallel-calls, lib/parallel | 2.26 | | | 4.91 | | 2.62 | 1.76 | 1.01 | 1.21 |

- **Parallel calls** take the step from 2.26 to 1.01 ms at 4 parts, 2.2x; 8 parts (with `--job-threads=7`) is no
  faster, two of them landing on the efficiency cores while the machine is busy. The timeline shows why it is not
  4x: the pairs are found in parallel, but making the contacts, the solver set up and the island work between the
  calls stay on the main row, and each of the solver's 170 or so stages ends in a sync.
- **Lanes lose here**: 4.1 ms at 4 and 7.7 at 8. The timeline shows the main lane packing each lane's section one
  after another (the accent spans on the main row) while the lanes wait, then copying the results back; this scene
  has 11500 contacts to copy each step, against the solver work of a few hundred µs per lane.
- In headless Chrome (`bench/web/lanes.html` on the threads build of the web harness, 4 parts, 3 job threads) the
  same scene steps in 1.8 to 2.9 ms under the same load, and every part's first span lands within 0.1 ms of the
  step's start, so the parts' clocks agree there. The page harness draws nothing, so that run was checked from a
  logged summary of the rows, not a capture.
- On the harness built without threads (node, `bench/web/lanes.mjs` on `out/wasm_release`) the release bake
  opens on Serial with "This runtime has no threads", and steps without trapping.

## The interface scenes implement

```as
// demo/scene.as
export class SceneStats {
    bodies, awake, shapes, contacts, joints, islands: i32;
    profileStep: f32;                    // b3Profile.step, smoothed
    profile: StaticArray<f32>;           // PROFILE_PAIRS .. PROFILE_OTHER, smoothed
}

export interface DemoScene {
    name(): string;
    hint(): string;
    reset(): void;
    leave(): void;                       // another scene is opening
    step(dt: f32): void;                 // fixed 1/60 s; the shell times it
    draw(dd: DebugDraw): void;
    stats(out: SceneStats): void;
    home(camera: OrbitCamera): void;
    updateCamera(camera: OrbitCamera): void; // every frame after the camera advances
    pointerDown(ray: Ray): bool;
    pointerMove(ray: Ray): void;
    pointerUp(ray: Ray): void;
    shoot(ray: Ray): void;
    keyDown(key: i32): bool;             // GLFW codes; Shift is forwarded too
    keyUp(key: i32): bool;
}
```

The shell calls `leave()` on the scene it leaves and `reset()` and `home()`
on the one that opens, `step()` at 60 Hz while running (at most three per
frame), `updateCamera()` every frame and `draw()` every frame between
`dd.begin()` and `dd.end()`, after setting `dd.showContacts` and
`dd.showJoints` from the C and J toggles.

## PhysicsScene

`demo/physics.as`. A scene extends it and overrides `build()` plus the hooks it
needs: `name`, `hint`, `home`, `worldDef`, `preStep(dt)` (before
`b3World_Step`), `postStep(dt)`, `drawExtra(dd)`, `groundHeight(x, z)`,
`updateCamera`, `keyDown`/`keyUp`. Options set in `build()`: `shotRadius`,
`shotSpeed`, `shotDensity`, `shotBullet`, `maxShots`, `contactScale`,
`uneven` (shadows at `groundHeight` under each body instead of on a plane),
`crowd` (no rings or shadows, for hundreds of round bodies).

- **World**: `reset()` destroys and recreates the world with one worker,
  clears the views, joints, shots and shape cache, and reseeds `random(lo,
  hi)` (the scenes' LCG). `step()` moves the mouse body with
  `b3Body_SetTargetTransform`, calls `preStep`, `b3World_Step(dt, 4)`,
  `postStep`, then blends `b3World_GetProfile` into the overlay figures.
- **Bodies**: there is no body iterator in the API, so scenes create bodies
  through `addBody(def, ink)` (or `addStanding` for static geometry that stands
  up, see `STATE_STANDING`), which records a `BodyView`; `removeBody` destroys
  and forgets one. `addJoint` records a joint for drawing.
- **Shapes** (`demo/b3view.as`): a `BodyView` resolves its shapes on first
  draw with `b3Body_GetShapes` into `ShapeView`s: sphere and capsule data,
  a `DebugHull` from the hull's half edge rings (`faces[f].edge`, then
  `edges[e].next`, collecting `origin`), a `DebugMesh` from vertices and
  triangles with the mesh scale applied (winding flipped under a mirroring
  scale), a `DebugHeightField` with heights decompressed as `minHeight +
  heightScale * compressed` and `B3_HEIGHT_FIELD_HOLE` cells as holes, and one
  view per compound child (`b3GetCompoundChild`) carrying its child transform.
  `ShapeCache` shares the derived geometry: hulls by `hash` (so 210 boxes
  share one `DebugHull`), meshes by data and scale, height fields by data.
- **Draw**: per body `b3Body_GetTransform`, `b3Body_GetType` and
  `b3Body_IsAwake` pick the state (static, standing, kinematic, sleeping,
  awake, plus selected for the grabbed body).
- **Contacts** (C): `b3Body_GetContactData` for each non static body; a
  contact is drawn once, from body A, or from body B when A is static. Points
  are `centerA + anchorA` (`b3Body_GetWorldCenter`), touching points (negative
  separation or nonzero total impulse) as dots with an arrow along the
  manifold normal, speculative ones as smaller muted dots.
  `b3Body_GetContactData` allocates a `b3ContactData` per contact, so this
  allocates every frame while on.
- **Joints** (J): every recorded joint except filter joints, as segments from
  each body origin to its anchor (`b3Joint_GetLocalFrameA/B` through the body
  transform), a segment between the anchors and a dot.
- **Picking**: `b3World_CastRayClosest` along the camera ray (400 m); a
  dynamic hit gets a kinematic mouse body and a motor joint exactly as
  `samples/sample.cpp` `MouseDown` sets it up (7.5 Hz, damping 1, spring
  force 100 mg, velocity torque from the inertia trace). Moving keeps the hit
  fraction along the new ray. The spring is drawn in the accent.
- **Shooting**: a bullet sphere from 2 m along the ray; the oldest of
  `maxShots` is destroyed.
- **Stats**: `b3World_GetCounters` (bodies, shapes, contacts, joints,
  islands), `b3World_GetAwakeBodyCount`, and the profile grouped as PAIRS
  (`pairs`), COLLIDE (`collide`), SOLVER (`constraints`), UPDATE
  (`transforms + refit`), CCD (`bullets`), ISLANDS (`sleepIslands +
  splitIslands`) and OTHER (the rest of `step`), each an exponential average
  with weight 0.08. The overlay draws them as a stacked bar plus one row each.

The character is not a body: `CharacterMover` keeps a capsule transform and
moves it with `b3World_CollideMover` (planes gathered in a closure),
`b3SolvePlanes`, `b3World_CastMover` and `b3ClipVector`, pushes dynamic bodies
in its planes with an impulse, and floats on a pogo spring from a
`b3World_CastRayClosest` ray, all as `mover.cpp`. The scene draws it as a
kinematic capsule with its velocity, its pogo ray and its collision planes,
and the camera follows it through `shiftTarget`, so orbit and zoom keep
working.

## DebugDraw

All positions are world space, Y up. `color` is ARGB (`0xFF000000 | hex`
turns a `b3HexColor` into one). `state` is one of `STATE_AWAKE`,
`STATE_SLEEPING`, `STATE_STATIC`, `STATE_KINEMATIC`, optionally or'ed with
`STATE_SELECTED`.

```as
dd.box(xf, halfExtents, color, state)
dd.hull(xf, hull: DebugHull, color, state)
dd.sphere(xf, localCenter, radius, color, state)
dd.capsule(xf, localCenter1, localCenter2, radius, color, state)
dd.mesh(xf, mesh: DebugMesh, color, state)
dd.heightField(xf, field: DebugHeightField, color, state)
dd.segment(a, b, color)    dd.arrow(from, to, color)    dd.point(p, sizePx, color)
dd.transform(xf, size)     dd.bounds(lower, upper, color)
dd.label(p, text, color)   dd.shadow(p, radius, groundY)
```

`STATE_STANDING` (32) can be or'ed into `STATE_STATIC`: the shape keeps the
static look but sorts with the dynamic layer, for walls, ramps and stairs that
bodies pass in front of. Static shapes never cast automatic blob shadows.
On the GPU renderer with its shadow maps on, every solid casts and `shadow`
records nothing.

Options a scene sets in `draw()`: `showGrid`, `gridY`, `gridExtent`,
`shadows` with `shadowY` (automatic contact shadows under awake and
kinematic solids on a flat ground), `lightDir`. Over uneven ground turn
`shadows` off and call `dd.shadow()` with the ground height under each body.
`rings` (on by default) draws the great circles on spheres and the seams on
capsules. `bodyInk(i)` gives a stable tint from the warm grey family.

### The GPU renderer

`Main.init` gives `DebugDraw` a `GpuDraw` from `GpuDraw.create(context)`,
which answers null when the host has no GPU canvas or the shader is missing,
and the solids then go to it instead of the painter. Scenes call the same
`DebugDraw` methods either way.

- **Target.** One `GPUCanvas` at the window size times the device pixel ratio
  (`resize`'s scale, through `dd.setScale`), cleared transparent, drawn with
  `drawImage` over the vector background and under the vector overlays. It
  renders 4x MSAA into a multisampled color texture that resolves into the
  canvas, with a `depth32float` depth texture of the same sample count; both
  are made again only when the canvas size changes.
- **Depth.** Reverse Z: the vertex shader writes clip z as the near plane and
  w as the camera depth, so stored depth is near over depth, 1 at the near
  plane falling toward 0; it clears to 0 and tests `greater`. The projection
  is `OrbitCamera`'s own, the focal length and the off center principal point
  of the `Clear` box, built in the shader from the eye, the camera axes and
  four numbers, so overlays, picking rays and the GPU image agree exactly.
- **Geometry.** A `GpuMesh` is an immutable vertex buffer of triangles,
  three vertices each: position, flat normal, and the corner's edge distances.
  Each `DebugHull` builds one on first use, its faces fanned, and keeps it, and
  `ShapeCache` already shares hulls by content hash, so the pyramid's 210
  boxes and the ground are two meshes. `DebugMesh` and `DebugHeightField` build
  theirs the same way, every triangle edge drawn as before, with no face cap.
  `dd.box` uses a unit cube scaled per instance. Spheres and capsules share a
  unit capsule: two hemispheres of unit directions, each tagged with its end,
  joined by a band that stretches to the capsule's length and vanishes for a
  sphere. It comes in two levels of detail, 40 by 2x10 and 16 by 2x4; under
  14 device pixels of radius on screen an instance takes the coarse one.
- **Instances.** A faceted instance is 48 bytes: rotation quaternion, origin,
  scale, fill and edge as `unorm8x4`. A round one is 56: both centers and the
  radius, the body's rotation (which turns a sphere's rings), fill and edge.
  A shadow is 32: ground point, spread, opacity, lift. Recording writes
  each into a preallocated staging `Float32Array` with a batch key; `render`
  groups them by key with a counting sort into a second array and uploads
  each kind with one `GPUBuffer.update`. A key is a geometry and whether the
  body is static (and for rounds, the level of detail), so a frame is one
  draw per hull, mesh or field geometry and layer, up to four round draws and
  one shadow draw. WebGL2 has no `firstInstance`, so each batch binds the
  instance buffer at its byte offset instead.
- **Order.** Static geometry first, then the shadows, which test depth but
  never write it, then everything that moves. A shadow therefore never covers
  a body, and needs no sorting.
- **Edges.** Every corner of a triangle carries one of (1,0,0), (0,1,0),
  (0,0,1), and a component stays 1 at all three corners when the edge
  opposite that corner is a fan diagonal inside a face. The fragment shader
  turns each component into a pixel distance with its screen gradient
  (`dpdxFine`, `dpdyFine`) and covers 0.6 points either side of the outline,
  so an inner edge is 1.2 points shared by its two faces and a silhouette
  0.6 points plus the MSAA edge. Faces only a few pixels across blend toward
  the share of their area the lines would cover, so dense meshes keep their
  tone without shimmer. Round shapes draw their rings (a sphere's two great
  circles, a capsule's two cap seams) from signed distances to those planes,
  and their outline from the facing ratio's own gradient.
- **Shading.** The painter's Lambert (`0.42 + 0.5 n.l + 0.12 sky`), with
  `n.l` scaled by how much sun the shadow maps let through, per pixel: with a
  flat normal on faceted solids, flipped toward the eye for one sided meshes,
  and an interpolated one on spheres and capsules. State tints (sleeping,
  kinematic, selected) and edge inks are worked out on the CPU as before.
- **Grid.** Drawn on the ground's upward faces in the faceted shader rather
  than as a plane of its own: a plane at y 0 cut through the rain scene's wave
  mesh. Ground is a static, not standing, shape; it arrives with a clear fill
  alpha, which the shader reads as the flag. Minor lines every meter and
  major every five, from `fract` distances in pixels, fading out from the
  grid center and where they crowd closer than a few points.
- **Shadows.** Cascaded sun shadow maps, below. With them off (`H` to off,
  or `NO_SHADOWS`) the painter's blob shadows come back as instanced quads on
  the ground, a radial smoothstep in the fragment shader. Over uneven ground
  (`shadows` off, the scene passing heights) the disc is pulled toward the
  eye by 0.8 of its spread, so it shows on slopes the flat disc would dip
  under; it can then show through a thin static wall right in front of it.
- **No per frame allocation.** The pass descriptor, its attachment, the pass
  object (`beginRenderPass(desc, into)`), the canvas image, the staging and
  upload arrays and the GPU buffers are all made once. Capacities: 8192
  faceted and 4096 round instances, 4096 shadows, 256 geometries a frame;
  past them `droppedCount` counts.

### Sun shadows

Ported from the Fields CSM by way of the town's moon (`~/Projects/town`,
branch `dusk`: `main.as` `updateMoon`, `lighting.wgsl`), with the maps
project's stand in and its rays through the view (`~/Projects/maps-rive`,
`shadows.as`).

- **Sun.** `DebugDraw.lightDir`, now `normalize(-0.42, 0.78, 0.62)`: about
  46 degrees up, from the left of the home views and a little toward the
  camera, so lit faces stay toward the viewer and shadows fall back and to
  the right where they read. The painter shades with the same vector, so
  `VECTOR_DRAW` moved with it.
- **Atlas.** One `depth32float` texture holding four cascade tiles two by
  two: 4096 square with 2048 tiles on high and soft, 2048 with 1024 tiles on
  low, which is also the start when `maxTextureSize2D` is under 4096. It is
  made on the first frame that wants it and again when the quality changes
  its size. One depth only pass clears it and draws every tile, each under
  its own `setViewport`.
- **Splits.** `ViewSpan` gathers the view depths the frame's solids reach
  while they are recorded: small ones by their bounding spheres when inside
  the view's side planes, large ones (grounds, walls, the height field and
  the rain mesh, wider than 8 m) by the band of heights they fill, which 25
  rays across the view cross. Without the band the ground made every scene
  start its first cascade at the near plane, where nothing is. The shadows
  reach at most two camera distances plus 12 m. Between those depths the
  splits are the practical scheme, 0.6 geometric and 0.4 even, the
  geometric part starting no nearer than a twentieth of the camera
  distance.
- **Fit.** Each slice of the view is bound by a sphere, found in camera
  space along the line through the centers of the slice's ends with a
  ternary search over the eight corners (the projection is off center, so
  the center is not on the forward axis). Working in camera space keeps the
  radius the same however the camera turns; it is then rounded up to a
  power of 1.08, so a zoom seldom changes the texel size. Each slice starts
  12% of the previous one's depth early, so it covers the crossfade.
- **Snapping.** The sphere's center is floored to whole texels along the
  light's two axes across the sun; with the radius fixed, an orbit moves the
  cascade by whole texels and shadow edges hold still. The light depth runs
  from 40 m beyond the sphere toward the sun (casters outside the view, and
  WebGPU has no depth clamp) to the sphere's far side.
- **Casters.** Two depth only pipelines, `vs_flat_cast` and `vs_round_cast`,
  over the same vertex and instance buffers and batches as the main pass;
  their vertex code is the main pass's, split into `flatWorld` and
  `roundPoint`. Every solid casts, static ones included. The first cascade
  draws spheres and capsules at their screen level of detail; the others
  draw them all coarse. Casters use `less` and a slope scaled bias of 1,
  without culling, since meshes and height fields are one sided.
- **Bindings.** One layout from the shader for group 0: the view uniforms,
  a 64 byte caster block holding one cascade's matrix, the atlas as
  `texture_depth_2d` (a `depthOnly` view) and a `less` comparison sampler.
  The caster passes cannot bind the atlas they draw into, so their four
  groups (one per cascade's caster block) bind a 1x1 depth texture there.
- **Bias.** Three parts, all scaled by the cascade's own texel: the receiver
  moves one texel along its normal, compares one texel of light depth
  nearer the sun, and each filter tap follows the receiver's plane, whose
  depth slope across the map comes from its normal (`n.r / n.l` and
  `n.u / n.l`, clamped to 10 texels per texel). The plane term is what lets
  the other two stay this small, so a box resting on the ground or on
  another box shows a contact shadow right up to its edge and faces grazing
  the sun do not darken.
- **Filtering.** High is a 7 texel tent from 16 bilinear comparisons, low a
  5 texel tent from 9 (Castano's optimized PCF, the Witness filter), each
  tap clamped inside its tile. Soft is PCSS: 8 taps search 10 texels around
  the receiver, each comparing at the receiver and at three depth gaps
  nearer the sun (the gaps a sun 0.05 wide in tangent needs to spread a
  penumbra 3, 7 and 14 texels), and the share of blockers past each gap
  steps the width of a 16 tap Vogel disc. GLSL cannot read raw depth from a
  depth texture (`textureLoad` on one fails the GLSL generation), so the
  blocker depths are counted with comparisons instead of averaged.
- **Selection.** By view depth, as the splits are made: the cascade a pixel
  falls in, crossfading into the next over the last 12% of its depth, and
  fading out over the last 15% of the reach. Branches on it are non
  uniform, so the filters use only the `Level` sampling variants, and the
  fragment shaders take their derivatives before calling them.
- **Receivers.** Every solid and the ground. A shadowed surface keeps the
  ambient and sky terms and its light leans warm (`1.05, 0.99, 0.9`, mixed
  in as the direct light falls under 0.4), so shadows sit at about 60% of
  the lit ground, never black. The grid is shaded with the ground it lies
  on; edges and rings mix in after the light, so they stay crisp over
  shadows. Blob shadows are not recorded while the maps are on, so the
  vector painter alone keeps them.

Draw cost against the painter, release AOT bake, `--bench=900` at 1280 by
800, Apple M5 Max, two interleaved rounds after the merge of the Threads
scene. Script is `DRAW_LOG`'s mean over the last windows, `dd.begin` through
`dd.end` with the scene's own draw; frame is the bench's render mean, which
adds Rive's flush on the CPU but not GPU time, which the API cannot time
(notes/findings.md). Before is the painter on main at db958de, painter is
this branch with `VECTOR_DRAW`.

| Scene | Script, painter | Script, GPU | Frame, before | Frame, painter | Frame, GPU |
|---|---|---|---|---|---|
| 1 Pyramid, asleep | 0.61 ms | 0.011 ms | 1.16 ms | 1.15 ms | 0.15 ms |
| 4 Rain, up to 260 bodies | 1.59 to 1.61 ms | 0.022 ms | 4.01 ms | 4.03 ms | 0.16 ms |
| 5 Height field | 1.12 to 1.14 ms | 0.007 ms | 3.26 ms | 3.28 ms | 0.15 ms |
| 8 Threads, 2048 rounds | 1.61 to 1.79 ms | 0.093 to 0.10 ms | 6.77 ms | 6.88 ms | 0.47 ms |

The Threads pit is two round draws on the GPU (coarse and fine detail) plus
a few hull draws for the walls and paddle; it honors `crowd`, with no rings
and no blob shadows, which would sit under the pile unseen. The shadow maps
do shade it: the pile's balls shadow each other and the walls the pile.

Shadow map cost, release AOT bake, `--bench=900` at 1280 by 800, Apple M5
Max, `DRAW_LOG`. Before is bbf209d (blob shadows), maps the high quality
default, off the same build with `NO_SHADOWS`. Each figure is the lowest of
four interleaved rounds (the machine was shared and the spread between rounds
was up to 3x, so the minimum is the comparable one): script is `DRAW_LOG`'s
lowest window mean, frame the bench's render p50, CPU only.

| Scene | Script, before | Script, maps | Script, off | Frame, before | Frame, maps | Frame, off |
|---|---|---|---|---|---|---|
| 1 Pyramid | 0.013 ms | 0.015 ms | 0.011 ms | 0.169 ms | 0.171 ms | 0.136 ms |
| 4 Rain | 0.022 ms | 0.029 ms | 0.022 ms | 0.155 ms | 0.176 ms | 0.159 ms |
| 5 Height field | 0.007 ms | 0.011 ms | 0.007 ms | 0.138 ms | 0.158 ms | 0.140 ms |
| 8 Threads | 0.104 ms | 0.117 ms | 0.101 ms | 0.490 ms | 0.526 ms | 0.491 ms |

The script pays for `ViewSpan` (a sphere test per solid) and the cascade fit;
the frame for one more pass and its draws, four times the batches. The GPU
side, which the API cannot time, was read from headless Chrome on the
published `@rive-app/webgl2` 2.48.0 with vsync and the frame rate limit off
(`webperf.mjs` below), the `requestAnimationFrame` interval's p50 over 120
frames, two runs: Threads 1.5 to 3.0 ms on high, 4.7 to 4.8 on soft, 1.1 to
1.4 on low, 1.1 to 1.2 off; the pyramid 0.7 to 1.1 ms on every quality. So
the 2048 body pit's four cascades cost well under a millisecond on high,
and soft, 48 comparisons a pixel over a screen of balls, about three.

The script now spends its draw on reading body transforms and writing 12 or
14 floats per body. In headless Chrome on the published `@rive-app/webgl2`
2.48.0 runtime the overlay's DRAW reads 0.06 to 0.14 ms and every scene holds
60 fps, at 1x and 2x.

The vector painter, below, is what `VECTOR_DRAW` and the fallback run.

How the vector painter draws:

- Solid faces are flat shaded from a fixed world light, back face culled,
  clipped against the near plane and culled off screen, then sorted per face
  by mean depth across all shapes and painted far to near, each with a
  hairline edge. Solid faces that overlap on screen are then ordered by their
  depths inside the overlap, with a topological sort. The overlapping pairs come
  from a sweep over the faces' screen left edges, which finds the pairs a loop
  over every two faces finds, measured the same way, so it orders the same; for
  a pile of 864 boxes (about 2500 faces) it took the ordering from 18 to 7 ms a
  frame, most of what is left being the overlap clipping.
- `STATE_STATIC` shapes go to a background layer painted first, then the
  grid, then shadows, then the sorted dynamic layer. That keeps large ground
  faces from sorting over the bodies resting on them. Standing static shapes
  sort with the bodies instead, per face by mean depth, so a long face can
  still paint over a body in front of it; the character level builds its long
  wall from 2 m blocks for that reason.
- Spheres are a shade toned disc with a feathered highlight toward the light,
  clipped to the silhouette, plus the camera facing halves of two great
  circles so spin reads. Capsules are the screen space hull of their two end
  circles, shaded the same way, with seam rings at the end centers.
- Sleeping bodies are desaturated and darkened; selected ones take the
  accent. Lines, points and labels draw over everything.
- Meshes and height fields add at most `MESH_FACE_CAP` (6000) faces each by
  striding; a frame holds 16384 primitives, beyond which `droppedCount`
  counts what was skipped.
- One `Path` and a handful of `Paint`s are reused for everything and the
  primitive store is preallocated, so a frame allocates nothing; a 300 frame
  bench grows wasm memory by zero pages.

The painter's draw cost on the release AOT bake, from the overlay's DRAW figure (the shell
times `dd.begin` through `dd.end`, including the scene's draw and the Box3D
queries in it): about 1 ms for the 210 box pyramid, 2.6 ms for the height
field scene, 5 ms for rain at 200 bodies over the 3200 triangle mesh (3725
faces). The interpreter is roughly ten times slower.

## What is left

- No sensors, events or explosions are shown; `b3World_Explode` and the
  contact begin and hit events are ported and would make good additions.
- Contact drawing allocates a `b3ContactData` per contact per frame (the
  port's `b3Body_GetContactData` news them); a fill-in-place variant in `lib/`
  would remove that, but `lib/` is not this task's to edit.
- The painter keeps its sort artifacts for large dynamic faces against each
  other; the GPU renderer has none.
- The painter's shadows over uneven ground are lifted discs, not draped on
  the terrain; the GPU renderer's shadow maps have no such limit.
- Static casters are drawn into every cascade every frame; their tiles could
  be kept while the camera and the sun hold still.
- The mover ignores `MoverShapeUserData` (no soft planes, no allies on
  category 2) and has no third person camera or mouse look.
- `SceneStats.shapes` is gathered but not shown.
- The G, C and J toggles have keys but no controls in the panels.
- The Threads panel has no workers backend choice; `b3_laneBackendWorkers` works with the same tracing.
- DRAW and FACES read 0 in headless captures (see Captures above). FACES
  counts triangles on the GPU renderer and faces on the painter.

## Screenshots

Final captures in `/private/tmp/claude-501/-Users-luigi-Projects-rive/65e4f28b-1249-4d28-8600-bb5fb40b325a/scratchpad/demo3d/shots/`
(the `*.log` beside each has the build and run log):

- `r1.png` pyramid at 20 frames; `s1_a.png` asleep at 120 (AOT)
- `i1pick.png` a box pulled out of the pyramid with the mouse joint; `i1shoot.png` two shots into it
- `r2c.png` card house with contacts on; `s2_a.png` asleep
- `r3.png` the ball through the ragdolls; `i3grab.png` a ragdoll held by its forearm
- `r4.png` rain at 600 frames (202 bodies)
- `r5.png` height field and compound at 200 frames
- `r6.png` the stack bullet arriving at frame 34; `s6_a.png` after the volley hit the thin wall
- `i7home.png` the character level; `i7climb.png` up the stairs onto the landing (W held);
  `i7push.png` crates pushed (S and D held)
- `t1.png` .. `t7.png` the AOT timing runs at 600 frames

The Threads scene, release AOT at 600 frames, in `/private/tmp/claude-501/-Users-luigi-Projects-rive/65e4f28b-1249-4d28-8600-bb5fb40b325a/scratchpad/t8final/`
(the `.log` beside each has `STEP_LOG`): `m_s_r2.png` serial, `m_l1_r1`, `m_l2_r1`, `m_l4_r1`, `m_l8_r2` lanes at
1, 2, 4 and 8 on master; `pp_s_r2`, `pp_l4_r2`, `pp_p1_r2`, `pp_p2_r2`, `pp_p4_r2`, `pp_p8_r2` on parallel-calls
with lib/parallel; `p_p4_r2` the same CLI without it, Parallel off and the scene on lanes. In
`/private/tmp/claude-501/-Users-luigi-Projects-rive/65e4f28b-1249-4d28-8600-bb5fb40b325a/scratchpad/t8shots/`, interpreted: `u5med.png` medium, `u6cmp.png` compact, `u6wide.png`
wide, `u4click.png` after clicking 2, Serial and Lanes, and in `cmp/` the other seven scenes before and after
(`before_<n>.png`, `after_<n>.png`; the view is pixel identical, only the scene list, the key help and the timing
digits differ).

The RML shell, all AOT at 40 frames unless noted, in the same folder:

- `rmlb1.png` .. `rmlb7.png` the hand drawn shell before it, `rmla1.png` ..
  `rmla7.png` the RML shell after; `rmlb4p` / `rmla4p` rain at 600 frames,
  `rmlb1paused` / `rmla1paused` paused with P; `cmp_<n>.png` puts the panel
  regions side by side, `pair_<n>.png` the whole frames
- `rmli_row.png` row 5 clicked; `rmli_pause`, `rmli_step` (two clicks),
  `rmli_reset` (rain at 300 frames, then reset), `rmli_keys` (key3, P, Tab),
  `rmli_hover` (row 3), `rmli_orbit`, `rmli_pick`; the `.json` beside the
  click captures is the `--data-dump`
- `rmli_noleak.json` and `rmli_shoot.json`: with Shoot on, clicks on a row,
  reset and Shoot leave 211 bodies, a click on the view makes 212;
  `rmli_orbitui.png` a drag that starts on row 3 opens the scene without
  orbiting
- `rmln.png` at 900 by 600, the key help wrapped

GPU renderer captures, 1280 by 800, interpreted, in
`/private/tmp/claude-501/-Users-luigi-Projects-rive/65e4f28b-1249-4d28-8600-bb5fb40b325a/scratchpad/gpu/shots/`:

- `before_s1.png` .. `before_s7.png` and `before_collapse.png` the painter at
  642691d, `after_*` the GPU renderer, `before_after_sheet.png` side by side;
  150 frames, and the collapse is scene 1, 30 frames, a drag from 700,540 to
  1100,700, 200 frames
- `close_edges.png`, `close_round.png`, `close_ragdoll.png` and their
  `_crop` zooms; `cmp_s1_edges.png`, `cmp_s7_grid.png` painter beside GPU
- `r_s1_*`, `r_s7_*` and `responsive_sheet.png` the size classes;
  `i_shoot`, `i_orbit`, `i_contacts` and `interact_sheet.png` input
- `web_*.png` (webgl2 2.48.0), `webt_*.png` (webgl2-threads 2.48.0) and
  `web2x_*.png` (2x) from headless Chrome on a local COOP/COEP mirror of the
  page
- after the Threads merge: `m2_gpu_s1.png` .. `m2_gpu_s8.png` and
  `m2_vec_s1.png` .. `m2_vec_s8.png` (`VECTOR_DRAW`), side by side in
  `m2_vector_gpu_sheet.png`; `m2_gpu_collapse.png`; `m2web_*.png` and
  `m2webt_*.png` the page on webgl2 and webgl2-threads 2.48.0

Sun shadow captures, 1280 by 800, interpreted, 150 frames unless noted, in
`/private/tmp/claude-501/-Users-luigi-Projects-rive/65e4f28b-1249-4d28-8600-bb5fb40b325a/scratchpad/csm/` (the
`cap.sh` there mirrors the project and, through `campatch.py`, adds `CAM_X`, `CAM_Y`, `CAM_Z` (centimeters
added to the home target), `CAM_D` (centimeters), `CAM_YAW` (milliradians added) and `CAM_PITCH` (hundredths)
defines to the mirror only):

- `contact_sheet.png` all eight scenes before (bbf209d, blob shadows) and after (high); the frames are
  `shots/before_s<n>.png` and `shots/final_s<n>.png`
- `closeups_sheet.png` before and after close-ups: the card house, the pyramid's shadow side, the bullet
  scene's stack at frame 25, the ragdolls (`shots/zb_*`, `shots/zf_*`)
- `cmp_q.png` the card house close-up on high, soft, low and off (`shots/q_*`)
- `dbg_grid.png` `CSM_DEBUG` on every scene, cascades tinted red, green, blue, yellow near to far
- `orbit_strip.png` and `orbit.gif` the card house close-up over twelve yaw steps of 6 milliradians: the
  shadow edges and the light let through between the cards move with the cards, without crawling
- `web_grid.png` and `webt_grid.png` the published webgl2 and webgl2-threads 2.48.0 runtimes in headless
  Chrome on a local COOP/COEP mirror (`serve.mjs`, `webcheck.mjs`), all eight scenes;
  `web_vs_native_card.png` the card house close-up native above, webgl2 below (the tiles land the same);
  `webperf.mjs` is `webcheck.mjs` with vsync and the frame rate limit off, for the GPU timings above
- `perf.log`, `perf2.log` the bench rounds; `manifest_<quality>.json` the `--gpu-manifest` per quality,
  which Dawn's null backend (`tools/webgpu_validate`) accepts: 1 shader, 5 pipelines, 6 bind groups (5 off)

## Host API notes

- Every `.as` in the project compiles even if nothing imports it.
- `--advance` advances without drawing the frames in between, so anything
  measured in `draw()` (face count, draw time) is stale if refreshed from
  `advance()`. A view model write from `draw()` binds only on the next
  advance, so the shell writes from `advance()` and accepts the staleness.
- A `.ttf` in a script only project resolves through `context.font("Name")`
  (the basename without extension); `context.blob()` returns null for it. In
  an RML project the same call finds the `FontAsset` by its `name`.
  Variable axes and `tnum` go through `Font.withOptions`.
- A text run without a paint draws nothing unless `Text.draw` is given one,
  and `Text.maxWidth` only wraps with `sizing = TextSizing.autoHeight`.
- Feathered fills need `FillRule.clockwise` and clockwise winding on screen;
  the debug draw keeps a second `Path` for them, and the main path stays
  `nonZero` so faces fill whatever their projected winding.
- Pointer events carry no buttons or modifiers; Shift is tracked from
  keyboard events (keys 340 and 344).
- `--key=3` sends key code 3; digits are `key0` through `key9`.
- `clock()` is real time even under `--advance`.

## rasc findings

- `Array<T>.shift()` is refused for value class arrays (`E3001: 'shift' is
  not supported on value class arrays yet`); `takeFirst` in
  `demo/physics.as` shifts by hand.
- A ternary of float literals types as f64 even inside an f32 sum:
  `let x: f32 = a + (c ? 0.0 : 20.0);` is `cannot implicitly convert f64 to
  f32`, while `let x: f32 = c ? 5.0 : 15.0;` compiles. Use an `if`.

Earlier, from the stand-ins:

No miscompiles or refusals hit. One doc mismatch:

- `rasc_site/docs/value-classes.html` lists "ternaries on values" among the
  struct uses that are compile errors, but rasc accepts them and they run
  correctly. The demo avoids them anyway. Repro:

  ```as
  struct V { x: f32 = 0; constructor(x: f32) { this.x = x; } }
  export function pick(c: bool): f32 {
      let v = c ? V(1) : V(2);
      return v.x;
  }
  ```

  `rasc check` is clean, and the `-O` build returns 1 and 2 for `pick(1)` and
  `pick(0)` in node.
