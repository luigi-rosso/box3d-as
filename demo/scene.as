// The contract between the demo shell and a physics scene.

import { Ray } from "./vec";
import { OrbitCamera } from "./camera";
import { DebugDraw } from "./debug_draw";

// The b3Profile breakdown the overlay shows, in milliseconds per step.
export const PROFILE_PAIRS: i32 = 0;
export const PROFILE_COLLIDE: i32 = 1;
export const PROFILE_SOLVER: i32 = 2;
export const PROFILE_UPDATE: i32 = 3;
export const PROFILE_CCD: i32 = 4;
export const PROFILE_ISLANDS: i32 = 5;
export const PROFILE_OTHER: i32 = 6;
export const PROFILE_COUNT: i32 = 7;

// What the overlay shows. The shell times step() itself.
export class SceneStats {
    bodies: i32 = 0;
    awake: i32 = 0;
    shapes: i32 = 0;
    contacts: i32 = 0;
    joints: i32 = 0;
    islands: i32 = 0;
    // b3Profile.step, and its parts indexed by PROFILE_*.
    profileStep: f32 = 0;
    profile: StaticArray<f32> = new StaticArray<f32>(PROFILE_COUNT);
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
    // Every frame after the camera advances, so a scene can follow a body.
    updateCamera(camera: OrbitCamera): void;

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
