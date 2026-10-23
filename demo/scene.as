// The contract between the demo shell and a physics scene. The stand-ins in
// scenes.as implement it now; the Box3D scenes implement it once lib/ lands.

import { Ray } from "./vec";
import { OrbitCamera } from "./camera";
import { DebugDraw } from "./debug_draw";

// What the overlay shows. The shell times step() itself.
export class SceneStats {
    bodies: i32 = 0;
    awake: i32 = 0;
    contacts: i32 = 0;
    joints: i32 = 0;
}

export interface DemoScene {
    name(): string;
    // A line of guidance under the scene list, or "".
    hint(): string;

    // Builds or rebuilds the world from scratch.
    reset(): void;
    // One fixed step; the shell calls it at 60 Hz while running.
    step(dt: f32): void;
    draw(dd: DebugDraw): void;
    stats(out: SceneStats): void;
    // The view to frame when the scene opens.
    home(camera: OrbitCamera): void;

    // A press over the scene. Return true to own the drag (a picked body),
    // false to let the shell orbit the camera.
    pointerDown(ray: Ray): bool;
    pointerMove(ray: Ray): void;
    pointerUp(ray: Ray): void;
    // Launch a projectile along the ray.
    shoot(ray: Ray): void;

    // GLFW key codes. Return true when the scene used the key.
    keyDown(key: i32): bool;
    keyUp(key: i32): bool;
}
