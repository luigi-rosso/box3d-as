// Small f32 value types for the demo renderer. The physics port keeps its own
// b3Vec3 and b3Quat; scenes convert at the debug draw boundary.

export struct Vec3 {
    x: f32 = 0;
    y: f32 = 0;
    z: f32 = 0;
    constructor(x: f32, y: f32, z: f32) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}

export struct Quat {
    x: f32 = 0;
    y: f32 = 0;
    z: f32 = 0;
    w: f32 = 1;
    constructor(x: f32, y: f32, z: f32, w: f32) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }
}

// A rigid transform: rotate by q, then translate by p.
export struct Xf {
    p: Vec3 = Vec3(0, 0, 0);
    q: Quat = Quat(0, 0, 0, 1);
    constructor(p: Vec3, q: Quat) {
        this.p = p;
        this.q = q;
    }
}

export struct Ray {
    origin: Vec3 = Vec3(0, 0, 0);
    dir: Vec3 = Vec3(0, 0, -1);
    constructor(origin: Vec3, dir: Vec3) {
        this.origin = origin;
        this.dir = dir;
    }
}

@inline export function v3(x: f32, y: f32, z: f32): Vec3 {
    return Vec3(x, y, z);
}

@inline export function add(a: Vec3, b: Vec3): Vec3 {
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

@inline export function sub(a: Vec3, b: Vec3): Vec3 {
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

@inline export function scale(a: Vec3, s: f32): Vec3 {
    return Vec3(a.x * s, a.y * s, a.z * s);
}

@inline export function madd(a: Vec3, b: Vec3, s: f32): Vec3 {
    return Vec3(a.x + b.x * s, a.y + b.y * s, a.z + b.z * s);
}

@inline export function dot(a: Vec3, b: Vec3): f32 {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

@inline export function cross(a: Vec3, b: Vec3): Vec3 {
    return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

@inline export function length(a: Vec3): f32 {
    return Mathf.sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

export function normalize(a: Vec3): Vec3 {
    let l = length(a);
    if (l < 1e-12) {
        return Vec3(0, 0, 0);
    }
    let inv: f32 = 1.0 / l;
    return Vec3(a.x * inv, a.y * inv, a.z * inv);
}

@inline export function lerp3(a: Vec3, b: Vec3, t: f32): Vec3 {
    return Vec3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}

export function rotate(q: Quat, v: Vec3): Vec3 {
    // v + 2w(q x v) + 2 q x (q x v)
    let qv = Vec3(q.x, q.y, q.z);
    let t = scale(cross(qv, v), 2);
    return add(add(v, scale(t, q.w)), cross(qv, t));
}

export function invRotate(q: Quat, v: Vec3): Vec3 {
    return rotate(Quat(-q.x, -q.y, -q.z, q.w), v);
}

@inline export function xfPoint(xf: Xf, v: Vec3): Vec3 {
    return add(rotate(xf.q, v), xf.p);
}

export function quatMul(a: Quat, b: Quat): Quat {
    return Quat(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

export function quatNormalize(q: Quat): Quat {
    let l = Mathf.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-12) {
        return Quat(0, 0, 0, 1);
    }
    let inv: f32 = 1.0 / l;
    return Quat(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
}

export function quatAxisAngle(axis: Vec3, angle: f32): Quat {
    let n = normalize(axis);
    let h: f32 = angle * 0.5;
    let s = Mathf.sin(h);
    return Quat(n.x * s, n.y * s, n.z * s, Mathf.cos(h));
}

// Advances q by angular velocity w over dt.
export function quatIntegrate(q: Quat, w: Vec3, dt: f32): Quat {
    let h: f32 = 0.5 * dt;
    let dq = quatMul(Quat(w.x * h, w.y * h, w.z * h, 0), q);
    return quatNormalize(Quat(q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w));
}

export function quatSlerpTo(a: Quat, b: Quat, t: f32): Quat {
    let d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    let s: f32 = d < 0 ? -1.0 : 1.0;
    return quatNormalize(Quat(
        a.x + (b.x * s - a.x) * t,
        a.y + (b.y * s - a.y) * t,
        a.z + (b.z * s - a.z) * t,
        a.w + (b.w * s - a.w) * t));
}

@inline export function clampf(v: f32, lo: f32, hi: f32): f32 {
    return v < lo ? lo : (v > hi ? hi : v);
}
