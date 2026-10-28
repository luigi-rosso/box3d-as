// Cascaded sun shadows, ported from the Fields CSM: practical splits over the
// view, a sphere around each slice of the frustum, the sphere's center
// snapped to whole texels along the light's axes, so shadow edges hold still
// while the camera orbits.

import { Vec3, v3, cross, dot, normalize } from "./vec";
import { OrbitCamera } from "./camera";

export const CASCADES: i32 = 4;

// Shadows reach this many camera distances past the eye, plus a fixed reach for close views.
const REACH: f32 = 2.0;
const REACH_EXTRA: f32 = 12;
// Rays a side the view is sampled with to find how deep large solids lie.
const RAYS: i32 = 5;
// Solids wider than this, grounds and walls, count by the heights they span rather than their spheres.
const LARGE: f32 = 8;
// How far the splits lean from even toward geometric.
const LAMBDA: f32 = 0.6;
// The share of each cascade's depth it crossfades into the next over.
export const BLEND: f32 = 0.12;
// Casters this far beyond a cascade's sphere toward the sun still cast into it.
const CASTER_REACH: f32 = 40;
// Sphere radii come in these steps, so a zoom seldom changes the texel size.
const RADIUS_STEP: f32 = 1.08;

// The view depths a frame's solids span: small ones by their spheres when
// in view, large ones by the band of heights they fill, which the view's
// rays cross.
export class ViewSpan {
    near: f32 = Infinity;
    far: f32 = -Infinity;
    low: f32 = Infinity;
    high: f32 = -Infinity;
    private camera: OrbitCamera = new OrbitCamera();
    // The view's side planes as slopes of x and y over depth, and their normals' lengths.
    private x0: f32 = 0;
    private x1: f32 = 0;
    private y0: f32 = 0;
    private y1: f32 = 0;
    private nx0: f32 = 1;
    private nx1: f32 = 1;
    private ny0: f32 = 1;
    private ny1: f32 = 1;

    reset(camera: OrbitCamera): void {
        this.camera = camera;
        this.near = Infinity;
        this.far = -Infinity;
        this.low = Infinity;
        this.high = -Infinity;
        let f = camera.focal;
        this.x0 = -camera.cx / f;
        this.x1 = (camera.width - camera.cx) / f;
        this.y0 = (camera.cy - camera.height) / f;
        this.y1 = camera.cy / f;
        this.nx0 = Mathf.sqrt(1 + this.x0 * this.x0);
        this.nx1 = Mathf.sqrt(1 + this.x1 * this.x1);
        this.ny0 = Mathf.sqrt(1 + this.y0 * this.y0);
        this.ny1 = Mathf.sqrt(1 + this.y1 * this.y1);
    }

    get empty(): bool {
        return this.near > this.far && this.low > this.high;
    }

    // A solid in the sphere at `center`, spanning heights `bottom` to `top`.
    add(center: Vec3, radius: f32, bottom: f32, top: f32): void {
        if (radius > LARGE) {
            this.low = Mathf.min(this.low, bottom);
            this.high = Mathf.max(this.high, top);
            return;
        }
        let c = this.camera.toCamera(center);
        if (c.z + radius < this.camera.near || c.x - this.x0 * c.z < -radius * this.nx0 ||
            this.x1 * c.z - c.x < -radius * this.nx1 || c.y - this.y0 * c.z < -radius * this.ny0 ||
            this.y1 * c.z - c.y < -radius * this.ny1) {
            return;
        }
        this.near = Mathf.min(this.near, c.z - radius);
        this.far = Mathf.max(this.far, c.z + radius);
    }
}

export class ShadowCascades {
    // World to each cascade's clip space, column major, 16 floats a cascade.
    matrices: Float32Array = new Float32Array(16 * CASCADES);
    // The view depth each cascade ends at.
    splits: StaticArray<f32> = new StaticArray<f32>(CASCADES);
    // World size of one texel, and the light depth it spans, per cascade.
    texels: StaticArray<f32> = new StaticArray<f32>(CASCADES);
    texelDepths: StaticArray<f32> = new StaticArray<f32>(CASCADES);
    // The light's axes: toward the sun, then across it.
    toSun: Vec3 = v3(0, 1, 0);
    right: Vec3 = v3(1, 0, 0);
    up: Vec3 = v3(0, 0, 1);
    // The view depths the first cascade starts at and the last one ends at.
    near: f32 = 0;
    far: f32 = 0;

    // Fits every cascade to the depths of the view `span` holds, with tiles `tile` texels wide.
    fit(camera: OrbitCamera, light: Vec3, tile: i32, span: ViewSpan): void {
        let l = normalize(light);
        this.toSun = l;
        let across = Mathf.abs(l.y) > 0.99 ? v3(1, 0, 0) : v3(0, 1, 0);
        this.right = normalize(cross(across, l));
        this.up = cross(l, this.right);

        let reach = camera.distance * REACH + REACH_EXTRA;
        let near = span.near;
        let far = span.far;
        let low = span.low;
        let high = span.high;
        for (let i = 0; i < RAYS * RAYS && low <= high; i++) {
            let sx = camera.width * <f32>(i % RAYS) / <f32>(RAYS - 1);
            let sy = camera.height * <f32>(i / RAYS) / <f32>(RAYS - 1);
            // Height climbs along the ray at this rate per unit of view depth.
            let climb = camera.forward.y + camera.right.y * (sx - camera.cx) / camera.focal +
                camera.up.y * (camera.cy - sy) / camera.focal;
            let from = camera.near;
            let to = reach;
            if (Mathf.abs(climb) > 1e-6) {
                let a = (low - camera.eye.y) / climb;
                let b = (high - camera.eye.y) / climb;
                from = Mathf.max(from, Mathf.min(a, b));
                to = Mathf.min(to, Mathf.max(a, b));
            } else if (camera.eye.y < low || camera.eye.y > high) {
                continue;
            }
            if (from < to) {
                near = Mathf.min(near, from);
                far = Mathf.max(far, to);
            }
        }
        near = Mathf.max(near, camera.near);
        far = Mathf.min(far, reach);
        if (far <= near) {
            near = camera.near;
            far = reach;
        }
        this.near = near;
        this.far = far;
        // The geometric part needs a start well off the eye.
        let logStart = Mathf.max(near, camera.distance * 0.05);
        let begin = near;
        for (let c = 0; c < CASCADES; c++) {
            let t = <f32>(c + 1) / <f32>CASCADES;
            let geometric = logStart * Mathf.pow(far / logStart, t);
            let even = logStart + (far - logStart) * t;
            let end = c == CASCADES - 1 ? far : LAMBDA * geometric + (1 - LAMBDA) * even;
            this.splits[c] = end;
            this.fitCascade(camera, c, begin, end, tile);
            // The next slice starts early so it covers this one's blend.
            begin = end - (end - begin) * BLEND;
        }
    }

    // The sphere around the slice of the view from depth d0 to d1, searched
    // along the line through the centers of the slice's ends, in camera space
    // so it keeps its size however the camera turns.
    private fitCascade(camera: OrbitCamera, c: i32, d0: f32, d1: f32, tile: i32): void {
        let f = camera.focal;
        let x0 = -camera.cx / f;
        let x1 = (camera.width - camera.cx) / f;
        let y0 = (camera.cy - camera.height) / f;
        let y1 = camera.cy / f;
        let mx = 0.5 * (x0 + x1);
        let my = 0.5 * (y0 + y1);
        let lo = d0;
        let hi = d1;
        for (let i = 0; i < 24; i++) {
            let a = lo + (hi - lo) / 3;
            let b = hi - (hi - lo) / 3;
            if (sliceRadius(a, mx, my, x0, x1, y0, y1, d0, d1) < sliceRadius(b, mx, my, x0, x1, y0, y1, d0, d1)) {
                hi = b;
            } else {
                lo = a;
            }
        }
        let t = 0.5 * (lo + hi);
        let radius = sliceRadius(t, mx, my, x0, x1, y0, y1, d0, d1);
        radius = Mathf.pow(RADIUS_STEP, Mathf.ceil(Mathf.log(radius) / Mathf.log(RADIUS_STEP)));

        let center = v3(camera.eye.x + (camera.right.x * mx + camera.up.x * my + camera.forward.x) * t,
            camera.eye.y + (camera.right.y * mx + camera.up.y * my + camera.forward.y) * t,
            camera.eye.z + (camera.right.z * mx + camera.up.z * my + camera.forward.z) * t);
        let texel = 2 * radius / <f32>tile;
        let cr = Mathf.floor(dot(center, this.right) / texel) * texel;
        let cu = Mathf.floor(dot(center, this.up) / texel) * texel;
        let top = dot(center, this.toSun) + radius + CASTER_REACH;
        let range = 2 * radius + CASTER_REACH;
        this.texels[c] = texel;
        this.texelDepths[c] = texel / range;

        // Rows: across, up, and depth away from the sun from 0 to 1.
        let m = this.matrices;
        let o = c * 16;
        let r = this.right;
        let u = this.up;
        let l = this.toSun;
        m[o] = r.x / radius;
        m[o + 1] = u.x / radius;
        m[o + 2] = -l.x / range;
        m[o + 3] = 0;
        m[o + 4] = r.y / radius;
        m[o + 5] = u.y / radius;
        m[o + 6] = -l.y / range;
        m[o + 7] = 0;
        m[o + 8] = r.z / radius;
        m[o + 9] = u.z / radius;
        m[o + 10] = -l.z / range;
        m[o + 11] = 0;
        m[o + 12] = -cr / radius;
        m[o + 13] = -cu / radius;
        m[o + 14] = top / range;
        m[o + 15] = 1;
    }
}

// The farthest corner of the slice from the point t deep on its center line, in camera space.
function sliceRadius(t: f32, mx: f32, my: f32, x0: f32, x1: f32, y0: f32, y1: f32, d0: f32, d1: f32): f32 {
    let far: f32 = 0;
    for (let k = 0; k < 8; k++) {
        let d = (k & 4) != 0 ? d1 : d0;
        let x = ((k & 1) != 0 ? x1 : x0) * d - mx * t;
        let y = ((k & 2) != 0 ? y1 : y0) * d - my * t;
        let z = d - t;
        far = Mathf.max(far, x * x + y * y + z * z);
    }
    return Mathf.sqrt(far);
}
