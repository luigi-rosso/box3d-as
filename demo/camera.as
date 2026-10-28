// Orbit camera: yaw and pitch around a target, perspective onto the artboard.
// World is Y up, as in Box3D.

import { Vec3, Ray, v3, add, sub, scale, dot, cross, normalize, madd, clampf } from "./vec";

const PITCH_MAX: f32 = 1.52;
// Scenes sit on a ground; looking up from under it shows nothing useful.
const PITCH_MIN: f32 = -0.12;
const MIN_DISTANCE: f32 = 1.5;
const MAX_DISTANCE: f32 = 400;
// Half tangents of the view every scene's home must show: the part of a
// 1280 by 800 frame at a 0.75 radian vertical field the home views were tuned to.
const FIT_X: f32 = 0.4;
const FIT_Y: f32 = 0.34;

export class OrbitCamera {
    yaw: f32 = 0.6;
    pitch: f32 = 0.45;
    distance: f32 = 18;
    target: Vec3 = Vec3(0, 1.5, 0);
    near: f32 = 0.1;

    width: f32 = 1;
    height: f32 = 1;
    // The area the scene is framed in, the whole viewport until frame() narrows it.
    frameX: f32 = 0;
    frameY: f32 = 0;
    frameWidth: f32 = 0;
    frameHeight: f32 = 0;
    // Tangents the frame must show left, right, below and above the target.
    fitLeft: f32 = FIT_X;
    fitRight: f32 = FIT_X;
    fitDown: f32 = FIT_Y;
    fitUp: f32 = FIT_Y;

    // Derived by update().
    eye: Vec3 = Vec3(0, 0, 0);
    right: Vec3 = Vec3(1, 0, 0);
    up: Vec3 = Vec3(0, 1, 0);
    forward: Vec3 = Vec3(0, 0, -1);
    focal: f32 = 1;
    cx: f32 = 0;
    cy: f32 = 0;

    // Orbit momentum after a drag, radians per second.
    yawVelocity: f32 = 0;
    pitchVelocity: f32 = 0;

    private animating: bool = false;
    private goalYaw: f32 = 0;
    private goalPitch: f32 = 0;
    private goalDistance: f32 = 0;
    private goalTarget: Vec3 = Vec3(0, 0, 0);

    constructor() {
        this.update();
    }

    setViewport(width: f32, height: f32): void {
        this.width = width > 1 ? width : 1;
        this.height = height > 1 ? height : 1;
        this.update();
    }

    // Centers the projection on an area of the viewport and fits the home
    // view in it, so orbit still pivots on the target.
    frame(x: f32, y: f32, width: f32, height: f32): void {
        if (x == this.frameX && y == this.frameY && width == this.frameWidth && height == this.frameHeight) {
            return;
        }
        this.frameX = x;
        this.frameY = y;
        this.frameWidth = width;
        this.frameHeight = height;
        this.update();
    }

    set(target: Vec3, distance: f32, yaw: f32, pitch: f32): void {
        this.animating = false;
        this.target = target;
        this.distance = distance;
        this.yaw = yaw;
        this.pitch = pitch;
        this.yawVelocity = 0;
        this.pitchVelocity = 0;
        this.update();
    }

    // Eases toward a view; any direct input cancels it.
    animateTo(target: Vec3, distance: f32, yaw: f32, pitch: f32): void {
        this.fitLeft = FIT_X;
        this.fitRight = FIT_X;
        this.fitDown = FIT_Y;
        this.fitUp = FIT_Y;
        this.goalTarget = target;
        this.goalDistance = distance;
        // Take the short way around.
        let twoPi: f32 = 6.2831853;
        let d = yaw - this.yaw;
        d -= twoPi * Mathf.round(d / twoPi);
        this.goalYaw = this.yaw + d;
        this.goalPitch = pitch;
        this.yawVelocity = 0;
        this.pitchVelocity = 0;
        this.animating = true;
    }

    // Widens the fit so a world box shows whole from the view animateTo set.
    include(lo: Vec3, hi: Vec3): void {
        let goal = new OrbitCamera();
        goal.set(this.goalTarget, this.goalDistance, this.goalYaw, this.goalPitch);
        for (let i = 0; i < 8; i++) {
            let c = goal.toCamera(Vec3((i & 1) != 0 ? hi.x : lo.x, (i & 2) != 0 ? hi.y : lo.y, (i & 4) != 0 ? hi.z : lo.z));
            let x = c.x / c.z;
            let y = c.y / c.z;
            this.fitLeft = Mathf.max(this.fitLeft, -x);
            this.fitRight = Mathf.max(this.fitRight, x);
            this.fitDown = Mathf.max(this.fitDown, -y);
            this.fitUp = Mathf.max(this.fitUp, y);
        }
        this.update();
    }

    // Moves the target, and any eased goal with it, without cancelling input.
    shiftTarget(delta: Vec3): void {
        this.target = add(this.target, delta);
        this.goalTarget = add(this.goalTarget, delta);
        this.update();
    }

    // Jumps to the end of an animateTo.
    snap(): void {
        if (this.animating) {
            this.set(this.goalTarget, this.goalDistance, this.goalYaw, this.goalPitch);
        }
    }

    get isMoving(): bool {
        return this.animating || this.yawVelocity != 0 || this.pitchVelocity != 0;
    }

    advance(dt: f32): void {
        if (this.animating) {
            let k: f32 = 1.0 - Mathf.exp(-dt * 7.0);
            this.yaw += (this.goalYaw - this.yaw) * k;
            this.pitch += (this.goalPitch - this.pitch) * k;
            this.distance += (this.goalDistance - this.distance) * k;
            this.target = add(this.target, scale(sub(this.goalTarget, this.target), k));
            if (Mathf.abs(this.goalYaw - this.yaw) < 1e-4 && Mathf.abs(this.goalDistance - this.distance) < 1e-3) {
                this.animating = false;
            }
        } else if (this.yawVelocity != 0 || this.pitchVelocity != 0) {
            this.yaw += this.yawVelocity * dt;
            this.pitch = clampf(this.pitch + this.pitchVelocity * dt, PITCH_MIN, PITCH_MAX);
            let damp = Mathf.exp(-dt * 5.0);
            this.yawVelocity *= damp;
            this.pitchVelocity *= damp;
            if (Mathf.abs(this.yawVelocity) < 0.01 && Mathf.abs(this.pitchVelocity) < 0.01) {
                this.yawVelocity = 0;
                this.pitchVelocity = 0;
            }
        }
        this.update();
    }

    // Drag deltas in points.
    orbit(dx: f32, dy: f32): void {
        this.animating = false;
        this.yaw -= dx * 0.008;
        this.pitch = clampf(this.pitch + dy * 0.008, PITCH_MIN, PITCH_MAX);
        this.update();
    }

    pan(dx: f32, dy: f32): void {
        this.animating = false;
        let s = this.distance / this.focal;
        this.target = madd(madd(this.target, this.right, -dx * s), this.up, dy * s);
        this.update();
    }

    zoom(factor: f32): void {
        this.animating = false;
        this.distance = clampf(this.distance * factor, MIN_DISTANCE, MAX_DISTANCE);
        this.update();
    }

    update(): void {
        let cp = Mathf.cos(this.pitch);
        let offset = v3(cp * Mathf.sin(this.yaw), Mathf.sin(this.pitch), cp * Mathf.cos(this.yaw));
        this.eye = madd(this.target, offset, this.distance);
        this.forward = scale(offset, -1);
        this.right = normalize(cross(this.forward, v3(0, 1, 0)));
        this.up = cross(this.right, this.forward);
        let framed = this.frameWidth > 1 && this.frameHeight > 1;
        let w = framed ? this.frameWidth : this.width;
        let h = framed ? this.frameHeight : this.height;
        let spanX = this.fitLeft + this.fitRight;
        let spanY = this.fitDown + this.fitUp;
        this.focal = Mathf.min(w / spanX, h / spanY);
        // The fit is centered in the frame, so the target sits off center when the fit is lopsided.
        this.cx = (framed ? this.frameX : 0) + 0.5 * (w - this.focal * spanX) + this.focal * this.fitLeft;
        this.cy = (framed ? this.frameY : 0) + 0.5 * (h - this.focal * spanY) + this.focal * this.fitUp;
    }

    // Camera space: x right, y up, z depth along the view.
    @inline toCamera(p: Vec3): Vec3 {
        let d = sub(p, this.eye);
        return Vec3(dot(d, this.right), dot(d, this.up), dot(d, this.forward));
    }

    // Camera space to screen: x, y in points and z the depth, unchanged.
    @inline projectCamera(c: Vec3): Vec3 {
        let inv: f32 = this.focal / c.z;
        return Vec3(this.cx + c.x * inv, this.cy - c.y * inv, c.z);
    }

    // World to screen; z is the depth, at or below near when behind the eye.
    project(p: Vec3): Vec3 {
        let c = this.toCamera(p);
        if (c.z < this.near) {
            return Vec3(0, 0, c.z);
        }
        return this.projectCamera(c);
    }

    ray(sx: f32, sy: f32): Ray {
        let x = (sx - this.cx) / this.focal;
        let y = (this.cy - sy) / this.focal;
        let d = normalize(add(add(this.forward, scale(this.right, x)), scale(this.up, y)));
        return Ray(this.eye, d);
    }
}
