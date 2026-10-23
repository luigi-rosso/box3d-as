// Stand-in scenes: toy kinematics, not physics. Bodies fall, land on the
// ground or on whatever is under them, settle flat and sleep, so the renderer,
// picking, shooting and keys can be exercised before the Box3D port lands.

import { Color } from "rive/color";
import { Vec3, Quat, Xf, Ray, v3, add, sub, scale, madd, dot, cross, length, normalize, rotate, invRotate,
    xfPoint, quatMul, quatNormalize, quatAxisAngle, quatIntegrate, quatSlerpTo } from "./vec";
import { OrbitCamera } from "./camera";
import { DebugDraw, DebugHull, DebugMesh, DebugHeightField, STATE_AWAKE, STATE_SLEEPING, STATE_STATIC,
    STATE_KINEMATIC, STATE_SELECTED, INK_ACCENT, INK_GROUND, INK_LINE, INK_MUTED, bodyInk } from "./debug_draw";
import { DemoScene, SceneStats } from "./scene";

const KIND_BOX: i32 = 0;
const KIND_SPHERE: i32 = 1;
const KIND_CAPSULE: i32 = 2;
const KIND_HULL: i32 = 3;

const GRAVITY: f32 = 9.8;
const SLEEP_TIME: f32 = 0.6;
const MAX_PROJECTILES: i32 = 24;

// GLFW key codes the stand-ins use.
const KEY_SPACE: i32 = 32;
const KEY_A: i32 = 65;
const KEY_C: i32 = 67;
const KEY_D: i32 = 68;
const KEY_S: i32 = 83;
const KEY_W: i32 = 87;
const KEY_RIGHT: i32 = 262;
const KEY_LEFT: i32 = 263;
const KEY_DOWN: i32 = 264;
const KEY_UP: i32 = 265;

class ToyBody {
    kind: i32 = KIND_BOX;
    xf: Xf = Xf(Vec3(0, 0, 0), Quat(0, 0, 0, 1));
    vel: Vec3 = Vec3(0, 0, 0);
    spin: Vec3 = Vec3(0, 0, 0);
    half: Vec3 = Vec3(0.5, 0.5, 0.5);
    radius: f32 = 0.5;
    center1: Vec3 = Vec3(0, -0.5, 0);
    center2: Vec3 = Vec3(0, 0.5, 0);
    hull: DebugHull? = null;
    color: Color = 0;
    kinematic: bool = false;
    grabbed: bool = false;
    touching: bool = false;
    sleeping: bool = false;
    rest: f32 = 0;
    // World bounds, refreshed each step.
    lower: Vec3 = Vec3(0, 0, 0);
    upper: Vec3 = Vec3(0, 0, 0);

    get bound(): f32 {
        if (this.kind == KIND_BOX) {
            return length(this.half);
        }
        if (this.kind == KIND_CAPSULE) {
            return Mathf.max(length(this.center1), length(this.center2)) + this.radius;
        }
        if (this.kind == KIND_HULL) {
            return this.hull!.radius;
        }
        return this.radius;
    }

    refreshBounds(): void {
        let p = this.xf.p;
        if (this.kind == KIND_SPHERE) {
            this.lower = sub(p, v3(this.radius, this.radius, this.radius));
            this.upper = add(p, v3(this.radius, this.radius, this.radius));
            return;
        }
        if (this.kind == KIND_CAPSULE) {
            let a = xfPoint(this.xf, this.center1);
            let b = xfPoint(this.xf, this.center2);
            let r = this.radius;
            this.lower = v3(Mathf.min(a.x, b.x) - r, Mathf.min(a.y, b.y) - r, Mathf.min(a.z, b.z) - r);
            this.upper = v3(Mathf.max(a.x, b.x) + r, Mathf.max(a.y, b.y) + r, Mathf.max(a.z, b.z) + r);
            return;
        }
        let lo = v3(1e9, 1e9, 1e9);
        let hi = v3(-1e9, -1e9, -1e9);
        if (this.kind == KIND_BOX) {
            for (let i = 0; i < 8; i++) {
                let c = xfPoint(this.xf, v3((i & 1) != 0 ? this.half.x : -this.half.x,
                    (i & 2) != 0 ? this.half.y : -this.half.y, (i & 4) != 0 ? this.half.z : -this.half.z));
                lo = v3(Mathf.min(lo.x, c.x), Mathf.min(lo.y, c.y), Mathf.min(lo.z, c.z));
                hi = v3(Mathf.max(hi.x, c.x), Mathf.max(hi.y, c.y), Mathf.max(hi.z, c.z));
            }
        } else {
            let pts = this.hull!.points;
            for (let i = 0; i < pts.length; i++) {
                let c = xfPoint(this.xf, pts[i]);
                lo = v3(Mathf.min(lo.x, c.x), Mathf.min(lo.y, c.y), Mathf.min(lo.z, c.z));
                hi = v3(Mathf.max(hi.x, c.x), Mathf.max(hi.y, c.y), Mathf.max(hi.z, c.z));
            }
        }
        this.lower = lo;
        this.upper = hi;
    }

    get state(): i32 {
        let s = this.kinematic ? STATE_KINEMATIC : (this.sleeping ? STATE_SLEEPING : STATE_AWAKE);
        return this.grabbed ? s | STATE_SELECTED : s;
    }

    wake(): void {
        this.sleeping = false;
        this.rest = 0;
    }
}

// The rotation taking unit a onto unit b.
function quatFromTo(a: Vec3, b: Vec3): Quat {
    let c = cross(a, b);
    let d = dot(a, b);
    if (d < -0.9999) {
        let side: f32 = Mathf.abs(a.x) < 0.9 ? 1.0 : 0.0;
        let axis = normalize(cross(a, v3(side, 1.0 - side, 0)));
        return Quat(axis.x, axis.y, axis.z, 0);
    }
    return quatNormalize(Quat(c.x, c.y, c.z, 1.0 + d));
}

// The orientation nearest q that rests a face flat on the ground.
function settled(q: Quat, kind: i32): Quat {
    if (kind == KIND_SPHERE) {
        return q;
    }
    let up = v3(0, 1, 0);
    if (kind == KIND_CAPSULE) {
        // Capsules lie down along their axis.
        let axis = rotate(q, v3(0, 1, 0));
        let flat = normalize(v3(axis.x, 0, axis.z));
        if (length(flat) < 0.5) {
            flat = v3(1, 0, 0);
        }
        return quatMul(quatFromTo(axis, flat), q);
    }
    let best = v3(0, 1, 0);
    let bestDot: f32 = -2;
    for (let i = 0; i < 6; i++) {
        let s: f32 = (i & 1) != 0 ? -1.0 : 1.0;
        let axis = v3(i < 2 ? s : 0, i >= 2 && i < 4 ? s : 0, i >= 4 ? s : 0);
        let w = rotate(q, axis);
        let d = dot(w, up);
        if (d > bestDot) {
            bestDot = d;
            best = w;
        }
    }
    return quatMul(quatFromTo(best, up), q);
}

export function makeGemHull(radius: f32, height: f32): DebugHull {
    // A hexagonal frustum: top, bottom, six sides.
    let n = 6;
    let points = new StaticArray<Vec3>(2 * n);
    for (let i = 0; i < n; i++) {
        let a: f32 = <f32>i * 6.2831853 / <f32>n;
        points[i] = v3(Mathf.cos(a) * radius, -height * 0.5, Mathf.sin(a) * radius);
        points[n + i] = v3(Mathf.cos(a) * radius * 0.55, height * 0.5, Mathf.sin(a) * radius * 0.55);
    }
    let faceStart = new StaticArray<i32>(n + 3);
    let faceIndex = new StaticArray<i32>(2 * n + 4 * n);
    let k = 0;
    faceStart[0] = 0;
    // Bottom, seen from below.
    for (let i = 0; i < n; i++) {
        faceIndex[k++] = i;
    }
    faceStart[1] = k;
    // Top, seen from above.
    for (let i = n - 1; i >= 0; i--) {
        faceIndex[k++] = n + i;
    }
    faceStart[2] = k;
    for (let i = 0; i < n; i++) {
        let j = (i + 1) % n;
        faceIndex[k++] = j;
        faceIndex[k++] = i;
        faceIndex[k++] = n + i;
        faceIndex[k++] = n + j;
        faceStart[3 + i] = k;
    }
    return new DebugHull(points, faceStart, faceIndex);
}

// Shared toy world behind the stand-in scenes.
class ToyScene implements DemoScene {
    bodies: Array<ToyBody> = new Array<ToyBody>();
    projectiles: i32 = 0;
    showContacts: bool = true;
    time: f32 = 0;
    seed: u32 = 1;
    character: ToyBody? = null;
    moveX: f32 = 0;
    moveZ: f32 = 0;
    keys: i32 = 0;

    private grabbed: ToyBody? = null;
    private grabDistance: f32 = 0;
    private grabLocal: Vec3 = Vec3(0, 0, 0);
    private grabTarget: Vec3 = Vec3(0, 0, 0);
    private grabPrev: Vec3 = Vec3(0, 0, 0);

    name(): string {
        return "";
    }

    hint(): string {
        return "";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 1.5, 0), 20, 0.65, 0.42);
    }

    build(): void {}

    groundAt(x: f32, z: f32): f32 {
        return 0;
    }

    drawGround(dd: DebugDraw): void {}

    random(): f32 {
        this.seed = this.seed * 1664525 + 1013904223;
        return <f32>(this.seed >> 8) / 16777216.0;
    }

    reset(): void {
        this.bodies = new Array<ToyBody>();
        this.projectiles = 0;
        this.time = 0;
        this.seed = 1;
        this.grabbed = null;
        this.character = null;
        this.build();
        for (let i = 0; i < this.bodies.length; i++) {
            this.bodies[i].refreshBounds();
        }
    }

    spawn(kind: i32, p: Vec3, q: Quat): ToyBody {
        let b = new ToyBody();
        b.kind = kind;
        b.xf = Xf(p, q);
        b.color = bodyInk(this.bodies.length);
        this.bodies.push(b);
        return b;
    }

    step(dt: f32): void {
        this.time += dt;
        let n = this.bodies.length;
        for (let i = 0; i < n; i++) {
            let b = this.bodies[i];
            if (b.sleeping && !b.grabbed) {
                continue;
            }
            b.touching = false;
            if (b.grabbed) {
                let target = sub(this.grabTarget, rotate(b.xf.q, this.grabLocal));
                b.vel = scale(sub(target, b.xf.p), 1.0 / dt);
                b.xf.p = target;
                b.refreshBounds();
                continue;
            }
            if (b.kinematic) {
                this.moveCharacter(b, dt);
                continue;
            }
            b.vel.y -= GRAVITY * dt;
            b.xf.p = madd(b.xf.p, b.vel, dt);
            b.xf.q = quatIntegrate(b.xf.q, b.spin, dt);
            b.refreshBounds();
            this.collide(b, i, dt);
        }
    }

    private collide(b: ToyBody, index: i32, dt: f32): void {
        // Supported by the ground or by the top of anything under the body.
        let cx: f32 = 0.5 * (b.lower.x + b.upper.x);
        let cz: f32 = 0.5 * (b.lower.z + b.upper.z);
        let support = this.groundAt(cx, cz);
        let onBody = false;
        for (let j = 0; j < this.bodies.length; j++) {
            if (j == index) {
                continue;
            }
            let o = this.bodies[j];
            if (o.upper.y > b.lower.y + 0.35 * (b.upper.y - b.lower.y) || o.grabbed) {
                continue;
            }
            let sx: f32 = 0.42 * ((b.upper.x - b.lower.x) + (o.upper.x - o.lower.x));
            let sz: f32 = 0.42 * ((b.upper.z - b.lower.z) + (o.upper.z - o.lower.z));
            let ox: f32 = 0.5 * (o.lower.x + o.upper.x);
            let oz: f32 = 0.5 * (o.lower.z + o.upper.z);
            if (Mathf.abs(ox - cx) < sx && Mathf.abs(oz - cz) < sz && o.upper.y > support) {
                support = o.upper.y;
                onBody = true;
            }
        }
        let lowest = b.lower.y;
        if (b.kind == KIND_SPHERE && !onBody) {
            lowest = b.xf.p.y - b.radius;
        }
        if (lowest > support) {
            return;
        }
        b.touching = true;
        b.xf.p.y += support - lowest;
        if (b.vel.y < 0) {
            b.vel.y = b.vel.y < -1.5 ? -b.vel.y * 0.3 : 0;
        }
        let friction: f32 = b.kind == KIND_SPHERE ? 0.995 : 0.9;
        b.vel.x *= friction;
        b.vel.z *= friction;
        if (b.kind == KIND_SPHERE) {
            if (!onBody) {
                // Roll downhill.
                let e: f32 = 0.05;
                let gx = (this.groundAt(cx + e, cz) - this.groundAt(cx - e, cz)) / (2 * e);
                let gz = (this.groundAt(cx, cz + e) - this.groundAt(cx, cz - e)) / (2 * e);
                b.vel.x -= gx * GRAVITY * 0.7 * dt;
                b.vel.z -= gz * GRAVITY * 0.7 * dt;
            }
            b.spin = scale(cross(v3(0, 1, 0), b.vel), 1.0 / b.radius);
        } else {
            b.spin = scale(b.spin, 0.85);
            b.xf.q = quatSlerpTo(b.xf.q, settled(b.xf.q, b.kind), 0.12);
        }
        b.refreshBounds();
        let speed = length(b.vel) + length(b.spin) * 0.2;
        if (speed < 0.08) {
            b.rest += dt;
            if (b.rest > SLEEP_TIME) {
                b.sleeping = true;
                b.vel = v3(0, 0, 0);
                b.spin = v3(0, 0, 0);
            }
        } else {
            b.rest = 0;
        }
    }

    private moveCharacter(b: ToyBody, dt: f32): void {
        let speed: f32 = 5;
        let target = v3(this.moveX * speed, 0, this.moveZ * speed);
        b.vel.x += (target.x - b.vel.x) * Mathf.min(1, dt * 12);
        b.vel.z += (target.z - b.vel.z) * Mathf.min(1, dt * 12);
        b.vel.y -= GRAVITY * dt;
        b.xf.p = madd(b.xf.p, b.vel, dt);
        let foot = this.groundAt(b.xf.p.x, b.xf.p.z) - b.center1.y + b.radius;
        // A little slack keeps the feet planted walking downhill.
        if (b.xf.p.y <= foot + 0.08 && b.vel.y <= 0) {
            b.xf.p.y = foot;
            b.vel.y = 0;
            b.touching = true;
        }
        let flat = v3(b.vel.x, 0, b.vel.z);
        if (length(flat) > 0.3) {
            let yaw = Mathf.atan2(flat.x, flat.z);
            b.xf.q = quatSlerpTo(b.xf.q, quatAxisAngle(v3(0, 1, 0), yaw), Mathf.min(1, dt * 10));
        }
        b.refreshBounds();
        // Shove whatever the character walks into.
        for (let j = 0; j < this.bodies.length; j++) {
            let o = this.bodies[j];
            if (o == b || o.grabbed) {
                continue;
            }
            let d = sub(o.xf.p, b.xf.p);
            d.y = 0;
            let l = length(d);
            let reach = b.radius + o.bound * 0.8;
            if (l < reach && l > 1e-4 && o.lower.y < b.upper.y) {
                o.wake();
                let push = scale(d, 1.0 / l);
                o.xf.p = madd(o.xf.p, push, (reach - l) * 0.5);
                o.vel = madd(o.vel, push, 1.5);
                o.vel.y = Mathf.max(o.vel.y, 0.5);
            }
        }
    }

    draw(dd: DebugDraw): void {
        this.drawGround(dd);
        for (let i = 0; i < this.bodies.length; i++) {
            let b = this.bodies[i];
            let state = b.state;
            if (b.kind == KIND_BOX) {
                dd.box(b.xf, b.half, b.color, state);
            } else if (b.kind == KIND_SPHERE) {
                dd.sphere(b.xf, v3(0, 0, 0), b.radius, b.color, state);
            } else if (b.kind == KIND_CAPSULE) {
                dd.capsule(b.xf, b.center1, b.center2, b.radius, b.color, state);
            } else {
                dd.hull(b.xf, b.hull!, b.color, state);
            }
            if (!dd.shadows) {
                dd.shadow(b.xf.p, b.bound * 0.8, this.groundAt(b.xf.p.x, b.xf.p.z));
            }
            if (this.showContacts && b.touching && !b.grabbed) {
                this.drawContacts(dd, b);
            }
        }
        let g = this.grabbed;
        if (g != null) {
            let anchor = xfPoint(g.xf, this.grabLocal);
            dd.segment(anchor, this.grabTarget, INK_ACCENT);
            dd.point(this.grabTarget, 6, INK_ACCENT);
            dd.label(add(g.upper, v3(0, 0.2, 0)), this.describe(g), INK_LINE);
        }
    }

    private describe(b: ToyBody): string {
        let kind = b.kind == KIND_BOX ? "box" : (b.kind == KIND_SPHERE ? "sphere"
            : (b.kind == KIND_CAPSULE ? "capsule" : "hull"));
        return kind + "  " + this.bodies.indexOf(b).toString();
    }

    private drawContacts(dd: DebugDraw, b: ToyBody): void {
        let y = b.lower.y;
        if (b.kind == KIND_SPHERE) {
            let p = v3(b.xf.p.x, y, b.xf.p.z);
            dd.point(p, 4, INK_ACCENT);
            dd.arrow(p, add(p, v3(0, 0.6, 0)), INK_MUTED);
            return;
        }
        // The lowest corners of the bounds stand in for a manifold.
        let x0 = b.lower.x + 0.12 * (b.upper.x - b.lower.x);
        let x1 = b.upper.x - 0.12 * (b.upper.x - b.lower.x);
        let z0 = b.lower.z + 0.12 * (b.upper.z - b.lower.z);
        let z1 = b.upper.z - 0.12 * (b.upper.z - b.lower.z);
        for (let k = 0; k < 4; k++) {
            let p = v3((k & 1) != 0 ? x1 : x0, y, (k & 2) != 0 ? z1 : z0);
            dd.point(p, 4, INK_ACCENT);
        }
        let c = v3(0.5 * (b.lower.x + b.upper.x), y, 0.5 * (b.lower.z + b.upper.z));
        dd.arrow(c, add(c, v3(0, 0.6, 0)), INK_MUTED);
    }

    stats(out: SceneStats): void {
        let awake = 0;
        let contacts = 0;
        for (let i = 0; i < this.bodies.length; i++) {
            let b = this.bodies[i];
            if (!b.sleeping) {
                awake++;
            }
            if (b.touching || b.sleeping) {
                contacts += b.kind == KIND_SPHERE ? 1 : 4;
            }
        }
        out.bodies = this.bodies.length;
        out.awake = awake;
        out.contacts = contacts;
        out.joints = 0;
    }

    pointerDown(ray: Ray): bool {
        let best: ToyBody? = null;
        let bestT: f32 = 1e9;
        for (let i = 0; i < this.bodies.length; i++) {
            let b = this.bodies[i];
            if (b.kinematic) {
                continue;
            }
            let t = raySphere(ray, b.xf.p, b.bound * 0.85);
            if (t >= 0 && t < bestT) {
                bestT = t;
                best = b;
            }
        }
        if (best == null) {
            return false;
        }
        let hit = madd(ray.origin, ray.dir, bestT);
        best.grabbed = true;
        best.wake();
        this.grabbed = best;
        this.grabDistance = bestT;
        this.grabLocal = invRotate(best.xf.q, sub(hit, best.xf.p));
        this.grabTarget = hit;
        return true;
    }

    pointerMove(ray: Ray): void {
        if (this.grabbed != null) {
            let t = madd(ray.origin, ray.dir, this.grabDistance);
            // Keep the grab above the ground.
            t.y = Mathf.max(t.y, this.groundAt(t.x, t.z) + 0.3);
            this.grabTarget = t;
        }
    }

    pointerUp(ray: Ray): void {
        let g = this.grabbed;
        if (g != null) {
            g.grabbed = false;
            g.wake();
            g.vel = scale(g.vel, 0.6);
            g.spin = v3(this.random() - 0.5, this.random() - 0.5, this.random() - 0.5);
            this.grabbed = null;
        }
    }

    shoot(ray: Ray): void {
        // Recycle the oldest projectile past the cap.
        let b = this.projectiles >= MAX_PROJECTILES ? this.findProjectile() : this.newProjectile(ray);
        b.xf.p = madd(ray.origin, ray.dir, 1.0);
        b.vel = scale(ray.dir, 28);
        b.wake();
        b.refreshBounds();
    }

    private newProjectile(ray: Ray): ToyBody {
        let b = this.spawn(KIND_SPHERE, ray.origin, Quat(0, 0, 0, 1));
        b.color = 0xFFE9E4DC;
        b.radius = 0.28;
        b.half = v3(-1, -1, -1);
        this.projectiles++;
        return b;
    }

    private findProjectile(): ToyBody {
        // Projectiles are marked by a negative half extent.
        let oldest = -1;
        for (let i = 0; i < this.bodies.length; i++) {
            if (this.bodies[i].half.x < 0) {
                oldest = i;
                break;
            }
        }
        let b = this.bodies[oldest];
        this.bodies.splice(oldest, 1);
        this.bodies.push(b);
        return b;
    }

    keyDown(key: i32): bool {
        if (key == KEY_C) {
            this.showContacts = !this.showContacts;
            return true;
        }
        if (this.character == null) {
            return false;
        }
        let bit = keyBit(key);
        if (bit == 0) {
            if (key == KEY_SPACE) {
                if (this.character!.touching) {
                    this.character!.vel.y = 5.5;
                }
                return true;
            }
            return false;
        }
        this.keys |= bit;
        this.updateMove();
        return true;
    }

    keyUp(key: i32): bool {
        let bit = keyBit(key);
        if (bit == 0 || this.character == null) {
            return false;
        }
        this.keys &= ~bit;
        this.updateMove();
        return true;
    }

    private updateMove(): void {
        let k = this.keys;
        this.moveX = <f32>(((k & 8) != 0 ? 1 : 0) - ((k & 4) != 0 ? 1 : 0));
        this.moveZ = <f32>(((k & 2) != 0 ? 1 : 0) - ((k & 1) != 0 ? 1 : 0));
    }
}

function keyBit(key: i32): i32 {
    if (key == KEY_W || key == KEY_UP) {
        return 1;
    }
    if (key == KEY_S || key == KEY_DOWN) {
        return 2;
    }
    if (key == KEY_A || key == KEY_LEFT) {
        return 4;
    }
    if (key == KEY_D || key == KEY_RIGHT) {
        return 8;
    }
    return 0;
}

// Distance along the ray to the sphere, or -1.
function raySphere(ray: Ray, c: Vec3, r: f32): f32 {
    let m = sub(ray.origin, c);
    let b = dot(m, ray.dir);
    let cc = dot(m, m) - r * r;
    let disc = b * b - cc;
    if (disc < 0) {
        return -1;
    }
    let t = -b - Mathf.sqrt(disc);
    return t >= 0 ? t : -1;
}

// Two columns of boxes dropping onto a static slab.
class StackScene extends ToyScene {
    name(): string {
        return "Box stack";
    }

    hint(): string {
        return "Stand-in: boxes drop and stack. C toggles contacts.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 2.4, 0), 15, 0.65, 0.34);
    }

    build(): void {
        for (let i = 0; i < 7; i++) {
            let jitter = quatAxisAngle(v3(0, 1, 0), (this.random() - 0.5) * 0.5);
            let b = this.spawn(KIND_BOX, v3(-1.6 + (this.random() - 0.5) * 0.12, 0.6 + <f32>i * 1.6, 0), jitter);
            b.half = v3(0.5, 0.5, 0.5);
        }
        for (let i = 0; i < 5; i++) {
            let b = this.spawn(KIND_BOX, v3(1.6, 0.6 + <f32>i * 1.3 + 3, 0.2 * <f32>(i % 2)),
                quatAxisAngle(v3(0, 1, 0), <f32>i * 0.35));
            b.half = v3(0.9, 0.25, 0.6);
        }
        let s = this.spawn(KIND_SPHERE, v3(0, 9, 1.4), Quat(0, 0, 0, 1));
        s.radius = 0.5;
    }

    drawGround(dd: DebugDraw): void {
        dd.showGrid = true;
        dd.shadows = true;
        dd.shadowY = 0;
        dd.gridY = 0;
        dd.box(Xf(v3(0, -0.5, 0), Quat(0, 0, 0, 1)), v3(16, 0.5, 16), INK_GROUND, STATE_STATIC);
    }
}

// A mix of shapes over a triangle mesh, with a capsule character.
class ShapesScene extends ToyScene {
    mesh: DebugMesh = ShapesScene.buildMesh();

    name(): string {
        return "Shapes on mesh";
    }

    hint(): string {
        return "Stand-in: WASD or arrows walk the capsule, Space jumps.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 1, 0), 22, -0.5, 0.5);
    }

    static wave(x: f32, z: f32): f32 {
        return 0.55 * Mathf.sin(x * 0.38) * Mathf.cos(z * 0.31) + 0.25 * Mathf.sin((x + z) * 0.21);
    }

    static buildMesh(): DebugMesh {
        let n = 24;
        let size: f32 = 24;
        let verts = new StaticArray<Vec3>((n + 1) * (n + 1));
        for (let k = 0; k <= n; k++) {
            for (let i = 0; i <= n; i++) {
                let x = -size * 0.5 + size * <f32>i / <f32>n;
                let z = -size * 0.5 + size * <f32>k / <f32>n;
                verts[k * (n + 1) + i] = v3(x, ShapesScene.wave(x, z), z);
            }
        }
        let idx = new StaticArray<i32>(n * n * 6);
        let t = 0;
        for (let k = 0; k < n; k++) {
            for (let i = 0; i < n; i++) {
                let a = k * (n + 1) + i;
                let b = a + 1;
                let c = a + n + 2;
                let d = a + n + 1;
                idx[t++] = a;
                idx[t++] = d;
                idx[t++] = c;
                idx[t++] = a;
                idx[t++] = c;
                idx[t++] = b;
            }
        }
        return new DebugMesh(verts, idx);
    }

    groundAt(x: f32, z: f32): f32 {
        return ShapesScene.wave(x, z);
    }

    build(): void {
        let hull = makeGemHull(0.75, 0.9);
        for (let i = 0; i < 12; i++) {
            let a: f32 = <f32>i * 2.4;
            let r: f32 = 2.5 + <f32>(i % 4) * 1.1;
            let p = v3(Mathf.cos(a) * r, 3 + <f32>i * 0.9, Mathf.sin(a) * r);
            let q = quatAxisAngle(v3(this.random(), this.random(), this.random()), this.random() * 3);
            let kind = i % 4;
            let b = this.spawn(kind, p, q);
            b.spin = v3(this.random() * 4 - 2, this.random() * 4 - 2, this.random() * 4 - 2);
            if (kind == KIND_BOX) {
                b.half = v3(0.45, 0.35, 0.6);
            } else if (kind == KIND_SPHERE) {
                b.radius = 0.55;
            } else if (kind == KIND_CAPSULE) {
                b.radius = 0.35;
                b.center1 = v3(0, -0.55, 0);
                b.center2 = v3(0, 0.55, 0);
            } else {
                b.hull = hull;
            }
        }
        let c = this.spawn(KIND_CAPSULE, v3(0, 2, 5), Quat(0, 0, 0, 1));
        c.kinematic = true;
        c.radius = 0.4;
        c.center1 = v3(0, -0.5, 0);
        c.center2 = v3(0, 0.5, 0);
        c.color = 0xFFEDE8E1;
        this.character = c;
    }

    drawGround(dd: DebugDraw): void {
        dd.showGrid = false;
        dd.shadows = false;
        dd.mesh(Xf(v3(0, 0, 0), Quat(0, 0, 0, 1)), this.mesh, INK_GROUND, STATE_STATIC);
    }
}

// Spheres and hulls rolling into a height field valley.
class TerrainScene extends ToyScene {
    field: DebugHeightField = TerrainScene.buildField();

    name(): string {
        return "Height field";
    }

    hint(): string {
        return "Stand-in: spheres roll downhill. Press a body to grab it; Tab switches to shooting.";
    }

    home(camera: OrbitCamera): void {
        camera.animateTo(v3(0, 0.5, 0), 26, 0.9, 0.62);
    }

    static valley(x: f32, z: f32): f32 {
        let r2 = (x * x + z * z) / 90.0;
        return 2.6 * r2 / (1.0 + r2 * 0.4) + 0.35 * Mathf.sin(x * 0.7) * Mathf.sin(z * 0.6) - 1.2;
    }

    static buildField(): DebugHeightField {
        let n = 33;
        let step: f32 = 0.75;
        let heights = new StaticArray<f32>(n * n);
        let half = <f32>(n - 1) * step * 0.5;
        for (let k = 0; k < n; k++) {
            for (let i = 0; i < n; i++) {
                heights[k * n + i] = TerrainScene.valley(<f32>i * step - half, <f32>k * step - half);
            }
        }
        return new DebugHeightField(n, n, v3(step, 1, step), heights);
    }

    groundAt(x: f32, z: f32): f32 {
        return TerrainScene.valley(x, z);
    }

    build(): void {
        let hull = makeGemHull(0.6, 0.7);
        for (let i = 0; i < 14; i++) {
            let a: f32 = <f32>i * 0.449;
            let r: f32 = 9 + this.random() * 2.5;
            let p = v3(Mathf.cos(a) * r, 5 + this.random() * 3, Mathf.sin(a) * r);
            let kind = i % 5 == 4 ? KIND_HULL : KIND_SPHERE;
            let b = this.spawn(kind, p, quatAxisAngle(v3(1, 0.3, 0.2), this.random() * 3));
            b.radius = 0.4 + this.random() * 0.35;
            b.hull = hull;
        }
    }

    drawGround(dd: DebugDraw): void {
        dd.showGrid = false;
        dd.shadows = false;
        let half = <f32>(this.field.countX - 1) * this.field.scale.x * 0.5;
        dd.heightField(Xf(v3(-half, 0, -half), Quat(0, 0, 0, 1)), this.field, INK_GROUND, STATE_STATIC);
    }
}

export function makeScenes(): Array<DemoScene> {
    let list = new Array<DemoScene>();
    list.push(new StackScene());
    list.push(new ShapesScene());
    list.push(new TerrainScene());
    return list;
}
