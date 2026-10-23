# Demo: debug draw and app shell

A rive-cli artboard that draws physics bodies with the Rive renderer: 3D
shapes flattened to 2D paths through an orbit camera, painted back to front.
Today it runs three stand-in scenes driven by toy kinematics. The Box3D
scenes plug in behind `DemoScene` once `lib/` lands.

## Files

| File | What it holds |
|---|---|
| `rive.yaml` | project; `scripting: wasm`; excludes `lib`, `verify`, `bench`, `notes` |
| `main.as` | the shell: scene list, transport, stats, camera and pointer input, keys |
| `demo/vec.as` | `Vec3`, `Quat`, `Xf` (p + q), `Ray`, f32 helpers |
| `demo/camera.as` | `OrbitCamera`: yaw, pitch, distance, target, perspective, rays |
| `demo/debug_draw.as` | `DebugDraw` plus `DebugHull`, `DebugMesh`, `DebugHeightField` |
| `demo/scene.as` | `DemoScene` and `SceneStats`, the contract for scenes |
| `demo/scenes.as` | the stand-ins and `makeScenes()`, the registry the shell reads |
| `demo/ui.as` | cached text runs, rects, rounded rects, UI palette |
| `demo/fonts/InterVariable.ttf` | Inter variable (OFL, license beside it), used at weight 300 and 420 |

**`lib/` is excluded in `rive.yaml` for now.** rive-cli compiles every `.as`
in the project, whether or not anything imports it, so a half ported `lib/`
breaks the demo build. Remove the `- lib` line once `lib/` compiles, then
import from `./lib/box3d` in the scene files.

## Running

```sh
RIVE=~/Projects/rive-master/packages/rive-cli/out/release/rive
cd ~/Projects/box3d-rive-as
$RIVE .                                   # live window
$RIVE . --verify                          # compile only
$RIVE . --screenshot=/tmp/x.png --advance=120
$RIVE . --screenshot=/tmp/x.png --define=START_SCENE=3 --advance=120
RIVE_WAMRC=~/Projects/tools/wamrc $RIVE . --bench=300 --define=START_SCENE=3
```

`START_SCENE` (1 based) picks the scene at launch; `--bench` refuses `--key`,
so it is the only way to bench a scene other than the first.

## Controls

| Input | Action |
|---|---|
| drag | orbit; a fling keeps spinning and eases out |
| Shift drag | pan |
| wheel, trackpad, pinch | zoom; trackpad sideways also orbits |
| press a body (Pick tool) | grab and drag it; release throws it |
| click (Shoot tool) | shoot a ball along the pointer ray |
| Space or B | shoot from the screen center, unless the scene uses Space |
| P / O / R | pause, single step (pauses), reset |
| Tab | switch Pick and Shoot |
| 1-9 | open a scene; the list on the left does the same |
| F | ease back to the scene's home view |
| G | grid on or off |
| C | contact points on or off (stand-ins) |
| WASD, arrows, Space | walk and jump the capsule character (Shapes on mesh) |

Headless digits need names: `--key=key3`, not `--key=3`, which sends key code 3.

## The interface scenes implement

```as
// demo/scene.as
export class SceneStats {
    bodies: i32 = 0;
    awake: i32 = 0;
    contacts: i32 = 0;
    joints: i32 = 0;
}

export interface DemoScene {
    name(): string;
    hint(): string;                 // a line under the scene list, or ""
    reset(): void;                  // build or rebuild the world
    step(dt: f32): void;            // fixed 1/60 s; the shell times it
    draw(dd: DebugDraw): void;
    stats(out: SceneStats): void;
    home(camera: OrbitCamera): void; // camera.animateTo(target, distance, yaw, pitch)
    pointerDown(ray: Ray): bool;    // true owns the drag (picked a body), false orbits
    pointerMove(ray: Ray): void;    // only while the scene owns the drag
    pointerUp(ray: Ray): void;
    shoot(ray: Ray): void;
    keyDown(key: i32): bool;        // GLFW codes; true when used
    keyUp(key: i32): bool;
}
```

Register a scene by pushing it in `makeScenes()` in `demo/scenes.as`; the
list, the number keys and `START_SCENE` follow the array order. The shell
calls `reset()` and `home()` when a scene opens, `step()` at 60 Hz while
running (at most three per frame), and `draw()` every frame between
`dd.begin()` and `dd.end()`. Step time, draw time and the face count in the
overlay come from the shell; bodies, awake and contacts from `stats()`.

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
  faces from sorting over the bodies resting on them; a static obstacle that
  stands up (a wall, a ramp) can therefore be overdrawn by a body behind it.
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

Performance, release AOT bake (`--bench=300`): Box stack renders in 0.3 ms
mean, Height field (about 2000 faces, two draws each) in 2.5 ms. The live
window runs the interpreted debug bake with fuel and is several times slower;
the overlay's DRAW figure shows it.

## Wiring the port

| Box3D | Demo |
|---|---|
| `b3Vec3`, `b3Quat`, `b3WorldTransform` | `Vec3`, `Quat`, `Xf(p, q)` field for field |
| `b3Body_GetTransform`, `b3Body_IsAwake`, `b3Body_GetType` | `xf` and `state` |
| `b3Shape_GetSphere` / `GetCapsule` | `dd.sphere(xf, s.center, s.radius, ...)`, `dd.capsule(xf, c.center1, c.center2, c.radius, ...)` |
| `b3Shape_GetHull` | a `DebugHull` built once per hull: walk each face's half edges (`next`, `origin`) into `faceStart` and `faceIndex`; cache it by hull |
| `b3Shape_GetMesh` | a `DebugMesh` from the mesh's vertices and triangle indices, cached; flip winding if the mesh is clockwise |
| `b3Shape_GetHeightField` | a `DebugHeightField(columnCount, rowCount, scale, heights)` with heights decompressed as `minHeight + heightScale * compressed`; material `0xFF` cells go to `holes` |
| `b3Body_GetContactData`, `b3ContactData` manifold points | `dd.point` and `dd.arrow` along the normal |
| `b3World_GetCounters` | `bodyCount`, `contactCount`, `jointCount` into `SceneStats`; `b3World_GetAwakeBodyCount` for awake |
| picking | `b3World_CastRayClosest(origin, dir * far)` then a motor joint to a kinematic target, as `samples/sample.cpp` `MouseDown` does |
| shooting | create a bullet sphere at `ray.origin` with velocity `ray.dir * speed` |

A sketch of a Box3D scene, once `lib/` exports these:

```as
class PyramidScene implements DemoScene {
    worldId: b3WorldId = b3WorldId();
    name(): string { return "Pyramid"; }
    hint(): string { return ""; }
    reset(): void { /* b3DestroyWorld, b3CreateWorld, ground box, pyramid of b3MakeBoxHull */ }
    step(dt: f32): void { b3World_Step(this.worldId, dt, 4); }
    draw(dd: DebugDraw): void { /* for each body and shape: dd.box / hull / sphere / capsule */ }
    stats(out: SceneStats): void { let c = b3World_GetCounters(this.worldId); out.bodies = c.bodyCount; /* ... */ }
    home(camera: OrbitCamera): void { camera.animateTo(v3(0, 4, 0), 30, 0.6, 0.35); }
    // pointer, shoot and key hooks as above
}
```

## What is left for the real scenes

- Port-side adapters: body and shape iteration into DebugDraw, the hull, mesh
  and height field converters with a cache keyed by the shape data, contact
  drawing from `b3ContactData`.
- Picking with a motor joint (`b3World_CastRayClosest` + `b3CreateMotorJoint`)
  and the spring line from anchor to target; the stand-ins teleport the body.
- Scenes: pyramid, card house, ragdolls and chains (joints draw as
  `segment`s and `point`s; add a joint pass to DebugDraw if they need more),
  capsule on mesh, bullets (fast spheres, continuous collision), and the
  kinematic capsule character on `keyDown`/`keyUp` (the Shapes stand-in shows
  the key plumbing, including Space consumed for jump).
- `SceneStats.joints` is not shown yet; add a row to the overlay when the
  joint scenes arrive.
- Re-include `lib/` in `rive.yaml`.

## Screenshots

In the scratchpad, `/private/tmp/claude-501/-Users-luigi-Projects-rive/65e4f28b-1249-4d28-8600-bb5fb40b325a/scratchpad/demo_agent/shots/`:

- `stack_falling.png` (60 frames) and `stack_rest.png` (300 frames, asleep)
- `shapes.png`: boxes, spheres, capsules and hulls on the triangle mesh
- `character.png`: the capsule walked with W and A, mid jump
- `terrain.png`: spheres and hulls on the height field
- `pick.png`: a box grabbed off the tower, selected tint and label
- `shoot.png`: Shoot tool, two balls fired
- `orbit_low.png`: orbit drag down to the pitch limit plus wheel zoom
- `paused_narrow.png`: 900x640, paused, wrapped hint, help line hidden

## Host API notes

- Every `.as` in the project compiles even if nothing imports it, hence the
  `lib` exclude.
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
