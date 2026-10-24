# Demo: debug draw and app shell

A rive-cli artboard that runs Box3D scenes from `lib/` and draws them with the
Rive renderer: 3D shapes flattened to 2D paths through an orbit camera,
painted back to front.

## Files

| File | What it holds |
|---|---|
| `rive.yaml` | project; `scripting: wasm`, `dangerouslyFast: true`; compiles `lib/`, excludes `verify`, `bench`, `notes` |
| `main.as` | the shell: scene list, transport, stats and profile overlay, camera and pointer input, keys |
| `demo/vec.as` | `Vec3`, `Quat`, `Xf` (p + q), `Ray`, f32 helpers |
| `demo/camera.as` | `OrbitCamera`: yaw, pitch, distance, target, perspective, rays, `shiftTarget` for following |
| `demo/debug_draw.as` | `DebugDraw` plus `DebugHull`, `DebugMesh`, `DebugHeightField` |
| `demo/scene.as` | `DemoScene` and `SceneStats`, the contract for scenes |
| `demo/b3view.as` | Box3D to DebugDraw: transform conversion, `ShapeCache`, `BodyView`, `fieldHeight` |
| `demo/physics.as` | `PhysicsScene`, the Box3D base every scene extends |
| `demo/human.as` | `createHuman`, the ragdoll from shared/human.c via verify/scenes/scenes.as |
| `demo/mover.as` | `CharacterMover`, samples/mover.cpp |
| `demo/scenes.as` | the seven scenes and `makeScenes()`, the registry the shell reads |
| `demo/ui.as` | cached text runs, rects, rounded rects, UI palette |
| `demo/fonts/InterVariable.ttf` | Inter variable (OFL, license beside it), used at weight 300 and 420 |

`dangerouslyFast` is on because the live window otherwise plays the fueled
debug bake, whose 200 ms script budget a single physics step can exceed (the
first capture after re-including `lib/` trapped in `advance`). The cost is no
breakpoints, and a runaway loop hangs instead of trapping.

## Running

```sh
RIVE=~/Projects/rive-master/packages/rive-cli/out/release/rive
cd ~/Projects/box3d-rive-as
RIVE_WAMRC=~/Projects/tools/wamrc RIVE_WASM_AOT_SYNC=o3 $RIVE . --aot   # live window, AOT
$RIVE . --verify                                                       # compile only
$RIVE . --screenshot=/tmp/x.png --define=START_SCENE=3 --advance=120
RIVE_WAMRC=~/Projects/tools/wamrc RIVE_WASM_AOT_SYNC=o3 $RIVE . --aot \
    --screenshot=/tmp/x.png --define=START_SCENE=1 --define=STEP_LOG --advance=600
```

Run the live window on AOT. The WAMR interpreter is 30 to 40 times slower here
(the awake pyramid steps in 165 ms interpreted, 4.5 ms on AOT), which is fine
for captures but not for play. The AOT compile takes about a minute.

`START_SCENE` (1 based) picks the scene at launch. `STEP_LOG` logs the mean
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
| 1-7 | open a scene; the list on the left does the same |
| F | ease back to the scene's home view |
| G | grid on or off |
| C | contact points and normals on or off (off by default) |
| J | joints on or off (on by default) |
| WASD, arrows | walk the character, relative to the camera |
| Space, Shift | jump, run (Character) |

Headless digits need names: `--key=key3`, not `--key=3`, which sends key code 3.
Held keys: `--key=w:down --advance=60`.

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

The shell calls `reset()` and `home()` when a scene opens, `step()` at 60 Hz
while running (at most three per frame), `updateCamera()` every frame and
`draw()` every frame between `dd.begin()` and `dd.end()`, after setting
`dd.showContacts` and `dd.showJoints` from the C and J toggles.

## PhysicsScene

`demo/physics.as`. A scene extends it and overrides `build()` plus the hooks it
needs: `name`, `hint`, `home`, `worldDef`, `preStep(dt)` (before
`b3World_Step`), `postStep(dt)`, `drawExtra(dd)`, `groundHeight(x, z)`,
`updateCamera`, `keyDown`/`keyUp`. Options set in `build()`: `shotRadius`,
`shotSpeed`, `shotDensity`, `shotBullet`, `maxShots`, `contactScale`,
`uneven` (shadows at `groundHeight` under each body instead of on a plane).

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
bodies pass in front of. Static shapes never cast automatic shadows.

Options a scene sets in `draw()`: `showGrid`, `gridY`, `gridExtent`,
`shadows` with `shadowY` (automatic contact shadows under awake and
kinematic solids on a flat ground), `lightDir`. Over uneven ground turn
`shadows` off and call `dd.shadow()` with the ground height under each body.
`bodyInk(i)` gives a stable tint from the warm grey family.

How it draws:

- Solid faces are flat shaded from a fixed world light, back face culled,
  clipped against the near plane and culled off screen, then sorted per face
  by mean depth across all shapes and painted far to near, each with a
  hairline edge.
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

Draw cost on the release AOT bake, from the overlay's DRAW figure (the shell
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
- Painter's sort artifacts remain for large dynamic faces against each other;
  there is no depth buffer.
- The mover ignores `MoverShapeUserData` (no soft planes, no allies on
  category 2) and has no third person camera or mouse look.
- `SceneStats.shapes` is gathered but not shown.

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

## Host API notes

- Every `.as` in the project compiles even if nothing imports it.
- `--advance` advances without drawing the frames in between, so anything
  measured in `draw()` (face count, draw time) is stale if refreshed from
  `advance()`; the shell refreshes the overlay from `draw()`.
- A `.ttf` in a script only project resolves through `context.font("Name")`
  (the basename without extension); `context.blob()` returns null for it.
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
