// Debug draw for the physics demo. With a GpuDraw, solids, shadows and the
// grid render instanced on the GPU with a depth buffer. Without one, 3D shapes
// are flattened to 2D paths through an orbit camera. Shapes are recorded between
// begin() and end(), then painted back to front: static shapes first, then their
// shadows, then everything else sorted per face. Box and hull faces that overlap
// on screen are then ordered by their depths inside the overlap. Lines, points
// and labels go on top through the vector renderer either way.

import { Renderer } from "rive/renderer";
import { Paint, PaintStyle, StrokeJoin, StrokeCap } from "rive/paint";
import { Path, FillRule } from "rive/path";
import { Vector } from "rive/vector";
import { Mat2D } from "rive/mat2d";
import { Color } from "rive/color";
import { Font, Text, TextRunStyle } from "rive/text";
import { Vec3, Xf, v3, add, sub, scale, dot, cross, length, normalize, rotate, xfPoint, clampf } from "./vec";
import { OrbitCamera } from "./camera";
import { GpuDraw, GpuMesh, MeshWriter } from "./gpu_draw";

// Body states, as Box3D reports them. Or STATE_SELECTED into any of them.
export const STATE_AWAKE: i32 = 0;
export const STATE_SLEEPING: i32 = 1;
export const STATE_STATIC: i32 = 2;
export const STATE_KINEMATIC: i32 = 3;
export const STATE_SELECTED: i32 = 16;
// A static shape that stands up off the ground, sorted with the bodies instead of painted first.
export const STATE_STANDING: i32 = 32;

// Palette: warm greys and one accent.
export const INK_BACKGROUND: Color = 0xFF151412;
export const INK_GRID: Color = 0xFF2B2926;
export const INK_GRID_MAJOR: Color = 0xFF3A3733;
export const INK_GROUND: Color = 0xFF34312D;
export const INK_BODY: Color = 0xFFD8D2C8;
export const INK_STONE: Color = 0xFFBDB6AB;
export const INK_SAND: Color = 0xFFD9CBB4;
export const INK_CLAY: Color = 0xFFC9B3A3;
export const INK_SLATE: Color = 0xFFAEB0AE;
export const INK_ACCENT: Color = 0xFFE0875A;
export const INK_LINE: Color = 0xFFEDE8E1;
export const INK_MUTED: Color = 0xFF8C867E;

const BODY_INKS: StaticArray<Color> = [INK_BODY, INK_SAND, INK_STONE, INK_CLAY, INK_SLATE];

// A stable per body tint from the warm grey family.
export function bodyInk(index: i32): Color {
    return BODY_INKS[(index & 0x7fffffff) % BODY_INKS.length];
}

const K_POLY: i32 = 0;
const K_DISC: i32 = 1;
const K_STADIUM: i32 = 2;
const K_STRIP: i32 = 3;
const K_SHADOW: i32 = 4;

const LAYER_STATIC: i32 = 0;
const LAYER_SHADOW: i32 = 1;
const LAYER_MAIN: i32 = 2;

const MAX_PRIMS: i32 = 16384;
const MAX_FLOATS: i32 = 16384 * 12;
const MAX_LINES: i32 = 8192;
const MAX_POINTS: i32 = 2048;
const MAX_LABELS: i32 = 64;
const MAX_CLIP: i32 = 64;
const MAX_EDGES: i32 = 65536;
// Screen overlaps smaller than this, in square points, do not order two faces.
const MIN_OVERLAP: f32 = 0.5;

// Faces one mesh or height field may add per frame.
export const MESH_FACE_CAP: i32 = 6000;

// A convex hull as face loops, counter clockwise seen from outside.
export class DebugHull {
    points: StaticArray<Vec3>;
    faceStart: StaticArray<i32>;
    faceIndex: StaticArray<i32>;
    normals: StaticArray<Vec3>;
    radius: f32 = 0;
    gpu: GpuMesh? = null;

    // faceStart has one entry per face plus a final end offset into faceIndex.
    constructor(points: StaticArray<Vec3>, faceStart: StaticArray<i32>, faceIndex: StaticArray<i32>) {
        this.points = points;
        this.faceStart = faceStart;
        this.faceIndex = faceIndex;
        let faceCount = faceStart.length - 1;
        this.normals = new StaticArray<Vec3>(faceCount);
        for (let f = 0; f < faceCount; f++) {
            this.normals[f] = newellNormal(points, faceIndex, faceStart[f], faceStart[f + 1]);
        }
        let r: f32 = 0;
        for (let i = 0; i < points.length; i++) {
            r = Mathf.max(r, length(points[i]));
        }
        this.radius = r;
    }

    get faceCount(): i32 {
        return this.faceStart.length - 1;
    }

    gpuMesh(): GpuMesh? {
        if (this.gpu == null) {
            let w = new MeshWriter();
            let loop = new StaticArray<Vec3>(MAX_CLIP);
            for (let f = 0; f < this.faceCount; f++) {
                let s = this.faceStart[f];
                let n = min(this.faceStart[f + 1] - s, MAX_CLIP);
                for (let i = 0; i < n; i++) {
                    loop[i] = this.points[this.faceIndex[s + i]];
                }
                w.polygon(loop, n, this.normals[f]);
            }
            this.gpu = w.finish();
        }
        return this.gpu;
    }
}

// A triangle mesh, counter clockwise seen from the front.
export class DebugMesh {
    vertices: StaticArray<Vec3>;
    indices: StaticArray<i32>;
    normals: StaticArray<Vec3>;
    gpu: GpuMesh? = null;

    constructor(vertices: StaticArray<Vec3>, indices: StaticArray<i32>) {
        this.vertices = vertices;
        this.indices = indices;
        let n = indices.length / 3;
        this.normals = new StaticArray<Vec3>(n);
        for (let t = 0; t < n; t++) {
            let a = vertices[indices[3 * t]];
            let b = vertices[indices[3 * t + 1]];
            let c = vertices[indices[3 * t + 2]];
            this.normals[t] = normalize(cross(sub(b, a), sub(c, a)));
        }
    }

    get triangleCount(): i32 {
        return this.indices.length / 3;
    }

    gpuMesh(): GpuMesh? {
        if (this.gpu == null) {
            let w = new MeshWriter();
            for (let t = 0; t < this.triangleCount; t++) {
                w.triangle(this.vertices[this.indices[3 * t]], this.vertices[this.indices[3 * t + 1]],
                    this.vertices[this.indices[3 * t + 2]], this.normals[t], true, true, true);
            }
            this.gpu = w.finish();
        }
        return this.gpu;
    }
}

// Heights on a countX by countZ grid, row major along x, scaled per axis, the
// grid starting at the local origin as b3HeightFieldDef lays it out.
export class DebugHeightField {
    countX: i32;
    countZ: i32;
    scale: Vec3;
    heights: StaticArray<f32>;
    // Optional, one per cell, (countX - 1) * (countZ - 1); true skips the cell.
    holes: StaticArray<bool>? = null;
    gpu: GpuMesh? = null;

    constructor(countX: i32, countZ: i32, scale: Vec3, heights: StaticArray<f32>) {
        this.countX = countX;
        this.countZ = countZ;
        this.scale = scale;
        this.heights = heights;
    }

    @inline point(i: i32, k: i32): Vec3 {
        return Vec3(<f32>i * this.scale.x, this.heights[k * this.countX + i] * this.scale.y, <f32>k * this.scale.z);
    }

    gpuMesh(): GpuMesh? {
        if (this.gpu == null) {
            let w = new MeshWriter();
            let holes = this.holes;
            for (let k = 0; k + 1 < this.countZ; k++) {
                for (let i = 0; i + 1 < this.countX; i++) {
                    if (holes != null && holes[k * (this.countX - 1) + i]) {
                        continue;
                    }
                    let a = this.point(i, k);
                    let b = this.point(i + 1, k);
                    let c = this.point(i + 1, k + 1);
                    let d = this.point(i, k + 1);
                    w.triangle(a, d, c, normalize(cross(sub(d, a), sub(c, a))), true, true, true);
                    w.triangle(a, c, b, normalize(cross(sub(c, a), sub(b, a))), true, true, true);
                }
            }
            this.gpu = w.finish();
        }
        return this.gpu;
    }
}

const UNIT: Vec3 = Vec3(1, 1, 1);

@inline function isStatic(state: i32): bool {
    return (state & 15) == STATE_STATIC;
}

// The GPU draws the grid on ground faces, marked by a clear alpha.
function groundInk(c: Color, state: i32): Color {
    return isStatic(state) && (state & STATE_STANDING) == 0 ? c & 0x00ffffff : c;
}

function newellNormal(points: StaticArray<Vec3>, index: StaticArray<i32>, start: i32, end: i32): Vec3 {
    let n = Vec3(0, 0, 0);
    for (let i = start; i < end; i++) {
        let j = i + 1 < end ? i + 1 : start;
        let a = points[index[i]];
        let b = points[index[j]];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return normalize(n);
}

function scaleColor(c: Color, s: f32): Color {
    let r = <i32>clampf(<f32>((c >> 16) & 0xff) * s, 0, 255);
    let g = <i32>clampf(<f32>((c >> 8) & 0xff) * s, 0, 255);
    let b = <i32>clampf(<f32>(c & 0xff) * s, 0, 255);
    return (c & 0xff000000) | (<u32>r << 16) | (<u32>g << 8) | <u32>b;
}

function mixColor(a: Color, b: Color, t: f32): Color {
    return Color.lerp(a, b, t);
}

function desaturate(c: Color, t: f32): Color {
    let r = <f32>((c >> 16) & 0xff);
    let g = <f32>((c >> 8) & 0xff);
    let b = <f32>(c & 0xff);
    let l: f32 = 0.299 * r + 0.587 * g + 0.114 * b;
    let rr = <u32>(r + (l - r) * t);
    let gg = <u32>(g + (l - g) * t);
    let bb = <u32>(b + (l - b) * t);
    return (c & 0xff000000) | (rr << 16) | (gg << 8) | bb;
}

function withAlpha(c: Color, a: f32): Color {
    return (c & 0x00ffffff) | (<u32>clampf(a * 255.0, 0, 255) << 24);
}

// Appends a circular arc as cubics of at most a quarter turn.
function arcTo(path: Path, cx: f32, cy: f32, r: f32, a0: f32, a1: f32, move: bool): void {
    let sweep = a1 - a0;
    let n = <i32>Mathf.ceil(Mathf.abs(sweep) / 1.5707964);
    if (n < 1) {
        n = 1;
    }
    let step = sweep / <f32>n;
    let k: f32 = 4.0 / 3.0 * Mathf.tan(step * 0.25) * r;
    let c0 = Mathf.cos(a0);
    let s0 = Mathf.sin(a0);
    if (move) {
        path.moveTo(Vector.xy(cx + c0 * r, cy + s0 * r));
    } else {
        path.lineTo(Vector.xy(cx + c0 * r, cy + s0 * r));
    }
    for (let i = 1; i <= n; i++) {
        let a = a0 + step * <f32>i;
        let c1 = Mathf.cos(a);
        let s1 = Mathf.sin(a);
        path.cubicTo(
            Vector.xy(cx + c0 * r - s0 * k, cy + s0 * r + c0 * k),
            Vector.xy(cx + c1 * r + s1 * k, cy + s1 * r - c1 * k),
            Vector.xy(cx + c1 * r, cy + s1 * r));
        c0 = c1;
        s0 = s1;
    }
}

function addCircle(path: Path, cx: f32, cy: f32, r: f32): void {
    arcTo(path, cx, cy, r, 0, 6.2831853, true);
    path.close();
}

// The convex outline of two circles, as a capsule looks on screen.
function addStadium(path: Path, x1: f32, y1: f32, r1: f32, x2: f32, y2: f32, r2: f32): void {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let d = Mathf.sqrt(dx * dx + dy * dy);
    if (d + Mathf.min(r1, r2) <= Mathf.max(r1, r2) + 1e-3) {
        if (r1 >= r2) {
            addCircle(path, x1, y1, r1);
        } else {
            addCircle(path, x2, y2, r2);
        }
        return;
    }
    let theta = Mathf.atan2(dy, dx);
    let phi = Mathf.acos(clampf((r1 - r2) / d, -1, 1));
    // Far side of the first circle, then the near side of the second.
    arcTo(path, x1, y1, r1, theta + phi, theta + 6.2831853 - phi, true);
    arcTo(path, x2, y2, r2, theta - phi, theta + phi, false);
    path.close();
}

class Label {
    text: Text = new Text();
    value: string = "";
    width: f32 = 0;
}

export class DebugDraw {
    camera: OrbitCamera = new OrbitCamera();

    // Options a scene or the shell may set.
    showGrid: bool = true;
    gridY: f32 = 0;
    gridExtent: i32 = 16;
    shadows: bool = true;
    shadowY: f32 = 0;
    // Great circles on spheres and seams on capsules, which show spin
    rings: bool = true;
    // Toward the sun: high on the left of the home views, so shadows fall back and to the right.
    lightDir: Vec3 = normalize(v3(-0.42, 0.78, 0.62));
    // Set by the shell; scenes read them to decide what to overlay.
    showContacts: bool = false;
    showJoints: bool = true;
    // Solids go to the GPU when set; the vector painter otherwise.
    gpu: GpuDraw? = null;

    // Counts of the last frame.
    primCount: i32 = 0;
    droppedCount: i32 = 0;
    drawCalls: i32 = 0;

    private kind: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private start: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private count: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private depth: StaticArray<f32> = new StaticArray<f32>(MAX_PRIMS);
    private fill: StaticArray<u32> = new StaticArray<u32>(MAX_PRIMS);
    private edge: StaticArray<u32> = new StaticArray<u32>(MAX_PRIMS);
    private layer: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private floats: StaticArray<f32> = new StaticArray<f32>(MAX_FLOATS);
    private order: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private prims: i32 = 0;
    private used: i32 = 0;

    // Box and hull faces: camera space plane and screen bounds.
    private solid: StaticArray<bool> = new StaticArray<bool>(MAX_PRIMS);
    private plane: StaticArray<f32> = new StaticArray<f32>(4 * MAX_PRIMS);
    private screenBounds: StaticArray<f32> = new StaticArray<f32>(4 * MAX_PRIMS);
    // Scratch for ordering them: slots in the sorted order, a graph of which paints first, and clipped overlaps.
    private slots: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private solids: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private byLeft: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private indegree: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS);
    private adjStart: StaticArray<i32> = new StaticArray<i32>(MAX_PRIMS + 1);
    private emitted: StaticArray<bool> = new StaticArray<bool>(MAX_PRIMS);
    private edgeFrom: StaticArray<i32> = new StaticArray<i32>(MAX_EDGES);
    private edgeTo: StaticArray<i32> = new StaticArray<i32>(MAX_EDGES);
    private adj: StaticArray<i32> = new StaticArray<i32>(MAX_EDGES);
    private polyA: StaticArray<f32> = new StaticArray<f32>(8 * MAX_CLIP);
    private polyB: StaticArray<f32> = new StaticArray<f32>(8 * MAX_CLIP);
    private overlapX: f32 = 0;
    private overlapY: f32 = 0;

    private lineA: StaticArray<Vec3> = new StaticArray<Vec3>(MAX_LINES);
    private lineB: StaticArray<Vec3> = new StaticArray<Vec3>(MAX_LINES);
    private lineColor: StaticArray<u32> = new StaticArray<u32>(MAX_LINES);
    private lineArrow: StaticArray<bool> = new StaticArray<bool>(MAX_LINES);
    private lines: i32 = 0;

    private pointP: StaticArray<Vec3> = new StaticArray<Vec3>(MAX_POINTS);
    private pointSize: StaticArray<f32> = new StaticArray<f32>(MAX_POINTS);
    private pointColor: StaticArray<u32> = new StaticArray<u32>(MAX_POINTS);
    private points: i32 = 0;

    private labelPool: Array<Label> = new Array<Label>();
    private labelP: StaticArray<Vec3> = new StaticArray<Vec3>(MAX_LABELS);
    private labelColor: StaticArray<u32> = new StaticArray<u32>(MAX_LABELS);
    private labels: i32 = 0;
    private labelStyle: TextRunStyle? = null;

    // Scratch for one face: world, camera and clipped camera space.
    private wv: StaticArray<Vec3> = new StaticArray<Vec3>(MAX_CLIP);
    private cv: StaticArray<Vec3> = new StaticArray<Vec3>(MAX_CLIP);
    private clipA: StaticArray<Vec3> = new StaticArray<Vec3>(MAX_CLIP);
    private boxCorners: StaticArray<Vec3> = new StaticArray<Vec3>(8);

    private path: Path = new Path();
    // Feathered fills need the clockwise fill rule.
    private soft: Path = new Path();
    private fillPaint: Paint = new Paint();
    private edgePaint: Paint = new Paint();
    private softPaint: Paint = new Paint();
    private linePaint: Paint = new Paint();

    constructor() {
        this.path.fillRule = FillRule.nonZero;
        this.soft.fillRule = FillRule.clockwise;
        this.fillPaint.style = PaintStyle.fill;
        this.edgePaint.style = PaintStyle.stroke;
        this.edgePaint.thickness = 1;
        this.edgePaint.join = StrokeJoin.round;
        this.softPaint.style = PaintStyle.fill;
        this.linePaint.style = PaintStyle.stroke;
        this.linePaint.thickness = 1.25;
        this.linePaint.join = StrokeJoin.round;
        this.linePaint.cap = StrokeCap.round;
    }

    setFont(font: Font): void {
        let style = new TextRunStyle(font);
        style.size = 11;
        style.letterSpacing = 0.3;
        this.labelStyle = style;
    }

    begin(camera: OrbitCamera): void {
        this.camera = camera;
        this.prims = 0;
        this.used = 0;
        this.lines = 0;
        this.points = 0;
        this.labels = 0;
        this.droppedCount = 0;
        let gpu = this.gpu;
        if (gpu != null) {
            gpu.begin(camera);
        }
    }

    // Device pixels per point, for the GPU canvas.
    setScale(scale: f32): void {
        let gpu = this.gpu;
        if (gpu != null) {
            gpu.scale = scale;
        }
    }

    // ---- solids -----------------------------------------------------------

    box(xf: Xf, halfExtents: Vec3, color: Color, state: i32): void {
        let c = this.boxCorners;
        for (let i = 0; i < 8; i++) {
            let local = Vec3((i & 1) != 0 ? halfExtents.x : -halfExtents.x,
                (i & 2) != 0 ? halfExtents.y : -halfExtents.y,
                (i & 4) != 0 ? halfExtents.z : -halfExtents.z);
            c[i] = xfPoint(xf, local);
        }
        let base = this.stateColor(color, state);
        let layer = this.layerFor(state);
        let gpu = this.gpu;
        if (gpu != null) {
            gpu.box(xf, halfExtents, groundInk(base, state), this.edgeColor(base, state), isStatic(state));
            this.solidShadow(xf.p, length(halfExtents) * 0.85, layer, state);
            return;
        }
        // Each face as a loop over the two other axes, bit masks per axis.
        for (let axis = 0; axis < 3; axis++) {
            let bitA = 1 << axis;
            let bitB = 1 << ((axis + 1) % 3);
            let bitC = 1 << ((axis + 2) % 3);
            for (let side = 0; side < 2; side++) {
                let fixed = side == 1 ? bitA : 0;
                this.wv[0] = c[fixed];
                this.wv[1] = c[fixed | bitB];
                this.wv[2] = c[fixed | bitB | bitC];
                this.wv[3] = c[fixed | bitC];
                let sign: f32 = side == 1 ? 1.0 : -1.0;
                let n = rotate(xf.q, Vec3(axis == 0 ? sign : 0, axis == 1 ? sign : 0, axis == 2 ? sign : 0));
                this.face(4, n, base, state, layer, false, true);
            }
        }
        this.solidShadow(xf.p, length(halfExtents) * 0.85, layer, state);
    }

    hull(xf: Xf, hull: DebugHull, color: Color, state: i32): void {
        let base = this.stateColor(color, state);
        let layer = this.layerFor(state);
        let gpu = this.gpu;
        if (gpu != null) {
            let mesh = hull.gpuMesh();
            if (mesh != null) {
                gpu.flat(mesh, xf, UNIT, groundInk(base, state), this.edgeColor(base, state), isStatic(state));
            }
            this.solidShadow(xf.p, hull.radius * 0.8, layer, state);
            return;
        }
        let faceCount = hull.faceCount;
        for (let f = 0; f < faceCount; f++) {
            let s = hull.faceStart[f];
            let e = hull.faceStart[f + 1];
            let n = e - s;
            if (n > MAX_CLIP / 2) {
                n = MAX_CLIP / 2;
            }
            for (let i = 0; i < n; i++) {
                this.wv[i] = xfPoint(xf, hull.points[hull.faceIndex[s + i]]);
            }
            this.face(n, rotate(xf.q, hull.normals[f]), base, state, layer, false, true);
        }
        this.solidShadow(xf.p, hull.radius * 0.8, layer, state);
    }

    sphere(xf: Xf, center: Vec3, radius: f32, color: Color, state: i32): void {
        let cam = this.camera;
        let p = xfPoint(xf, center);
        let gpu = this.gpu;
        if (gpu != null) {
            let base = this.stateColor(color, state);
            gpu.round(p, p, radius, xf.q, base, this.edgeColor(base, state), isStatic(state), this.rings);
            this.solidShadow(p, radius, this.layerFor(state), state);
            return;
        }
        let c = cam.toCamera(p);
        if (c.z < cam.near + radius * 0.5) {
            return;
        }
        let base = this.stateColor(color, state);
        let layer = this.layerFor(state);
        let s = cam.projectCamera(c);
        let rs = cam.focal * radius / Mathf.sqrt(Mathf.max(c.z * c.z - radius * radius, 1e-4));
        if (s.x + rs < 0 || s.x - rs > cam.width || s.y + rs < 0 || s.y - rs > cam.height) {
            return;
        }
        let i = this.alloc(K_DISC, 5, c.z, base, this.edgeColor(base, state), layer);
        if (i < 0) {
            return;
        }
        let o = this.start[i];
        let h = this.highlight(rs);
        this.floats[o] = s.x;
        this.floats[o + 1] = s.y;
        this.floats[o + 2] = rs;
        this.floats[o + 3] = h.x;
        this.floats[o + 4] = h.y;
        // Two great circles show the spin.
        if (this.rings) {
            let ringDepth = c.z - radius * 0.02;
            let ringInk = withAlpha(scaleColor(base, 0.45), 0.85);
            this.ring(p, rotate(xf.q, v3(0, 1, 0)), radius, ringDepth, ringInk, layer);
            this.ring(p, rotate(xf.q, v3(1, 0, 0)), radius, ringDepth, ringInk, layer);
        }
        this.solidShadow(p, radius, layer, state);
    }

    capsule(xf: Xf, center1: Vec3, center2: Vec3, radius: f32, color: Color, state: i32): void {
        let cam = this.camera;
        let p1 = xfPoint(xf, center1);
        let p2 = xfPoint(xf, center2);
        let gpu = this.gpu;
        if (gpu != null) {
            let base = this.stateColor(color, state);
            let layer = this.layerFor(state);
            gpu.round(p1, p2, radius, xf.q, base, this.edgeColor(base, state), isStatic(state), this.rings);
            this.solidShadow(p1, radius, layer, state);
            this.solidShadow(p2, radius, layer, state);
            return;
        }
        let c1 = cam.toCamera(p1);
        let c2 = cam.toCamera(p2);
        if (c1.z < cam.near + radius * 0.5 || c2.z < cam.near + radius * 0.5) {
            return;
        }
        let base = this.stateColor(color, state);
        let layer = this.layerFor(state);
        let s1 = cam.projectCamera(c1);
        let s2 = cam.projectCamera(c2);
        let r1 = cam.focal * radius / Mathf.sqrt(Mathf.max(c1.z * c1.z - radius * radius, 1e-4));
        let r2 = cam.focal * radius / Mathf.sqrt(Mathf.max(c2.z * c2.z - radius * radius, 1e-4));
        let depth: f32 = 0.5 * (c1.z + c2.z);
        let i = this.alloc(K_STADIUM, 8, depth, base, this.edgeColor(base, state), layer);
        if (i < 0) {
            return;
        }
        let o = this.start[i];
        let h = this.highlight(Mathf.min(r1, r2));
        this.floats[o] = s1.x;
        this.floats[o + 1] = s1.y;
        this.floats[o + 2] = r1;
        this.floats[o + 3] = s2.x;
        this.floats[o + 4] = s2.y;
        this.floats[o + 5] = r2;
        this.floats[o + 6] = h.x;
        this.floats[o + 7] = h.y;
        // Seams where the cylinder meets the caps.
        if (this.rings) {
            let axis = normalize(sub(p2, p1));
            let ringInk = withAlpha(scaleColor(base, 0.45), 0.85);
            this.ring(p1, axis, radius, depth - radius * 0.02, ringInk, layer);
            this.ring(p2, axis, radius, depth - radius * 0.02, ringInk, layer);
        }
        this.solidShadow(p1, radius, layer, state);
        this.solidShadow(p2, radius, layer, state);
    }

    mesh(xf: Xf, mesh: DebugMesh, color: Color, state: i32): void {
        let base = this.stateColor(color, state);
        let layer = this.layerFor(state);
        let gpu = this.gpu;
        if (gpu != null) {
            let m = mesh.gpuMesh();
            if (m != null) {
                gpu.flat(m, xf, UNIT, groundInk(base, state), this.edgeColor(base, state), isStatic(state));
            }
            return;
        }
        let n = mesh.triangleCount;
        let stride = 1;
        while (n / stride > MESH_FACE_CAP) {
            stride++;
        }
        for (let t = 0; t < n; t += stride) {
            this.wv[0] = xfPoint(xf, mesh.vertices[mesh.indices[3 * t]]);
            this.wv[1] = xfPoint(xf, mesh.vertices[mesh.indices[3 * t + 1]]);
            this.wv[2] = xfPoint(xf, mesh.vertices[mesh.indices[3 * t + 2]]);
            this.face(3, rotate(xf.q, mesh.normals[t]), base, state, layer, false, false);
        }
    }

    heightField(xf: Xf, field: DebugHeightField, color: Color, state: i32): void {
        let base = this.stateColor(color, state);
        let layer = this.layerFor(state);
        let gpu = this.gpu;
        if (gpu != null) {
            let m = field.gpuMesh();
            if (m != null) {
                gpu.flat(m, xf, UNIT, groundInk(base, state), this.edgeColor(base, state), isStatic(state));
            }
            return;
        }
        let cells = (field.countX - 1) * (field.countZ - 1);
        let stride = 1;
        while (2 * cells / (stride * stride) > MESH_FACE_CAP) {
            stride++;
        }
        for (let k = 0; k + stride < field.countZ; k += stride) {
            for (let i = 0; i + stride < field.countX; i += stride) {
                let holes = field.holes;
                if (holes != null && holes[k * (field.countX - 1) + i]) {
                    continue;
                }
                let a = xfPoint(xf, field.point(i, k));
                let b = xfPoint(xf, field.point(i + stride, k));
                let c = xfPoint(xf, field.point(i + stride, k + stride));
                let d = xfPoint(xf, field.point(i, k + stride));
                this.wv[0] = a;
                this.wv[1] = d;
                this.wv[2] = c;
                this.face(3, normalize(cross(sub(d, a), sub(c, a))), base, state, layer, false, false);
                this.wv[0] = a;
                this.wv[1] = c;
                this.wv[2] = b;
                this.face(3, normalize(cross(sub(c, a), sub(b, a))), base, state, layer, false, false);
            }
        }
    }

    // ---- overlay ----------------------------------------------------------

    segment(a: Vec3, b: Vec3, color: Color): void {
        this.addLine(a, b, color, false);
    }

    arrow(from: Vec3, to: Vec3, color: Color): void {
        this.addLine(from, to, color, true);
    }

    point(p: Vec3, size: f32, color: Color): void {
        if (this.points >= MAX_POINTS) {
            this.droppedCount++;
            return;
        }
        this.pointP[this.points] = p;
        this.pointSize[this.points] = size;
        this.pointColor[this.points] = color;
        this.points++;
    }

    transform(xf: Xf, size: f32): void {
        this.addLine(xf.p, xfPoint(xf, v3(size, 0, 0)), INK_ACCENT, false);
        this.addLine(xf.p, xfPoint(xf, v3(0, size, 0)), INK_LINE, false);
        this.addLine(xf.p, xfPoint(xf, v3(0, 0, size)), INK_MUTED, false);
    }

    bounds(lower: Vec3, upper: Vec3, color: Color): void {
        for (let i = 0; i < 4; i++) {
            let x0 = (i & 1) != 0 ? upper.x : lower.x;
            let z0 = (i & 2) != 0 ? upper.z : lower.z;
            this.addLine(v3(x0, lower.y, z0), v3(x0, upper.y, z0), color, false);
        }
        for (let y = 0; y < 2; y++) {
            let yy = y == 1 ? upper.y : lower.y;
            this.addLine(v3(lower.x, yy, lower.z), v3(upper.x, yy, lower.z), color, false);
            this.addLine(v3(upper.x, yy, lower.z), v3(upper.x, yy, upper.z), color, false);
            this.addLine(v3(upper.x, yy, upper.z), v3(lower.x, yy, upper.z), color, false);
            this.addLine(v3(lower.x, yy, upper.z), v3(lower.x, yy, lower.z), color, false);
        }
    }

    label(p: Vec3, text: string, color: Color): void {
        if (this.labels >= MAX_LABELS || this.labelStyle == null) {
            return;
        }
        let i = this.labels;
        if (i >= this.labelPool.length) {
            this.labelPool.push(new Label());
        }
        let l = this.labelPool[i];
        if (l.value != text) {
            l.value = text;
            l.text.clear();
            l.text.append(text, this.labelStyle!);
            let b = l.text.bounds();
            l.width = b.v1.x - b.v0.x;
        }
        this.labelP[i] = p;
        this.labelColor[i] = color;
        this.labels++;
    }

    // A soft contact shadow under p on the plane y = groundY. Solids add their
    // own on shadowY when shadows is on; scenes over uneven ground turn that
    // off and call this with the ground height under each body.
    shadow(p: Vec3, radius: f32, groundY: f32): void {
        let maps = this.gpu;
        if (maps != null && maps.castsShadows) {
            return;
        }
        let h = p.y - groundY;
        if (h < -radius) {
            return;
        }
        let fade = clampf(1.0 - (h - radius) / (radius * 3.0 + 1.0), 0, 1);
        if (fade <= 0.02) {
            return;
        }
        let spread: f32 = radius * (1.0 + 0.12 * Mathf.max(h - radius, 0));
        let gpu = this.gpu;
        if (gpu != null) {
            // Shadows on uneven ground lift toward the eye, out of the slopes under them.
            gpu.shadow(v3(p.x, groundY, p.z), spread, 0.26 * fade, this.shadows ? 0 : spread * 0.8);
            return;
        }
        let cam = this.camera;
        let n = 16;
        let i = this.alloc(K_SHADOW, 2 * n + 1, 0, withAlpha(0xFF000000, 0.26 * fade), 0, LAYER_SHADOW);
        if (i < 0) {
            return;
        }
        let o = this.start[i];
        let minX: f32 = 1e9;
        let maxX: f32 = -1e9;
        for (let k = 0; k < n; k++) {
            let a: f32 = <f32>k * 6.2831853 / <f32>n;
            let c = cam.toCamera(v3(p.x + Mathf.cos(a) * spread, groundY + 0.002, p.z + Mathf.sin(a) * spread));
            if (c.z < cam.near) {
                this.drop(i);
                return;
            }
            let s = cam.projectCamera(c);
            this.floats[o + 1 + 2 * k] = s.x;
            this.floats[o + 2 + 2 * k] = s.y;
            minX = Mathf.min(minX, s.x);
            maxX = Mathf.max(maxX, s.x);
        }
        this.floats[o] = (maxX - minX) * 0.22;
    }

    // ---- painting ---------------------------------------------------------

    end(renderer: Renderer): void {
        this.drawCalls = 0;
        let gpu = this.gpu;
        if (gpu != null) {
            if (this.showGrid) {
                gpu.grid(Mathf.round(this.camera.target.x), Mathf.round(this.camera.target.z), <f32>this.gridExtent);
            }
            gpu.render(renderer, this.camera, this.lightDir, INK_GRID, INK_GRID_MAJOR);
            this.primCount = gpu.triangles;
            this.droppedCount += gpu.dropped;
            this.drawLines(renderer);
            this.drawPoints(renderer);
            this.drawLabels(renderer);
            return;
        }
        this.primCount = this.prims;
        this.paintLayer(renderer, LAYER_STATIC);
        if (this.showGrid) {
            this.drawGrid(renderer);
        }
        this.paintLayer(renderer, LAYER_SHADOW);
        this.paintLayer(renderer, LAYER_MAIN);
        this.drawLines(renderer);
        this.drawPoints(renderer);
        this.drawLabels(renderer);
    }

    // ---- internals --------------------------------------------------------

    private solidShadow(p: Vec3, radius: f32, layer: i32, state: i32): void {
        if (this.shadows && layer == LAYER_MAIN && (state & 15) != STATE_STATIC) {
            this.shadow(p, radius, this.shadowY);
        }
    }

    private layerFor(state: i32): i32 {
        return (state & 15) == STATE_STATIC && (state & STATE_STANDING) == 0 ? LAYER_STATIC : LAYER_MAIN;
    }

    private stateColor(color: Color, state: i32): Color {
        let s = state & 15;
        let c = color;
        if (s == STATE_SLEEPING) {
            c = scaleColor(desaturate(c, 0.9), 0.72);
        } else if (s == STATE_KINEMATIC) {
            c = mixColor(c, 0xFF9FA6A8, 0.35);
        }
        if ((state & STATE_SELECTED) != 0) {
            c = mixColor(c, INK_ACCENT, 0.4);
        }
        return c;
    }

    private edgeColor(base: Color, state: i32): Color {
        if ((state & STATE_SELECTED) != 0) {
            return INK_ACCENT;
        }
        if ((state & 15) == STATE_STATIC) {
            return withAlpha(scaleColor(base, 1.35), 0.55);
        }
        return withAlpha(scaleColor(base, 0.32), 0.9);
    }

    // Lambert from a fixed key light plus a little sky.
    private lit(base: Color, n: Vec3): Color {
        let d = Mathf.max(dot(n, this.lightDir), 0);
        let s: f32 = 0.42 + 0.5 * d + 0.12 * (0.5 + 0.5 * n.y);
        return scaleColor(base, s);
    }

    // Screen offset of a sphere highlight toward the light.
    private highlight(rs: f32): Vec3 {
        let cam = this.camera;
        let lx = dot(this.lightDir, cam.right);
        let ly = -dot(this.lightDir, cam.up);
        let l = Mathf.sqrt(lx * lx + ly * ly);
        if (l < 1e-4) {
            return Vec3(0, 0, 0);
        }
        let k: f32 = rs * 0.38 / l * Mathf.min(l, 1);
        return Vec3(lx * k, ly * k, 0);
    }

    private alloc(kind: i32, floats: i32, depth: f32, fill: Color, edge: Color, layer: i32): i32 {
        if (this.prims >= MAX_PRIMS || this.used + floats > MAX_FLOATS) {
            this.droppedCount++;
            return -1;
        }
        let i = this.prims++;
        this.kind[i] = kind;
        this.start[i] = this.used;
        this.count[i] = floats;
        this.depth[i] = depth;
        this.fill[i] = fill;
        this.edge[i] = edge;
        this.layer[i] = layer;
        this.solid[i] = false;
        this.used += floats;
        return i;
    }

    // Releases the most recent prim.
    private drop(i: i32): void {
        if (i == this.prims - 1) {
            this.prims--;
            this.used = this.start[i];
        }
    }

    // Records the face in wv[0..n), normal n in world space. A solid face belongs to a convex body.
    private face(n: i32, normal: Vec3, base: Color, state: i32, layer: i32, twoSided: bool, solid: bool): void {
        let cam = this.camera;
        let toEye = sub(cam.eye, this.wv[0]);
        let facing = dot(normal, toEye);
        if (facing <= 0 && !twoSided) {
            return;
        }
        let shadeN = facing < 0 ? scale(normal, -1) : normal;
        let depthSum: f32 = 0;
        let behind = 0;
        for (let i = 0; i < n; i++) {
            let c = cam.toCamera(this.wv[i]);
            this.cv[i] = c;
            depthSum += c.z;
            if (c.z < cam.near) {
                behind++;
            }
        }
        if (behind == n) {
            return;
        }
        let m = n;
        let src = this.cv;
        if (behind > 0) {
            m = this.clipNear(n);
            if (m < 3) {
                return;
            }
            src = this.clipA;
        }
        let fill = this.lit(base, shadeN);
        let i = this.alloc(K_POLY, 2 * m, depthSum / <f32>n, fill, this.edgeColor(base, state), layer);
        if (i < 0) {
            return;
        }
        let o = this.start[i];
        let minX: f32 = 1e9;
        let maxX: f32 = -1e9;
        let minY: f32 = 1e9;
        let maxY: f32 = -1e9;
        for (let k = 0; k < m; k++) {
            let s = cam.projectCamera(src[k]);
            this.floats[o + 2 * k] = s.x;
            this.floats[o + 2 * k + 1] = s.y;
            minX = Mathf.min(minX, s.x);
            maxX = Mathf.max(maxX, s.x);
            minY = Mathf.min(minY, s.y);
            maxY = Mathf.max(maxY, s.y);
        }
        if (maxX < 0 || minX > cam.width || maxY < 0 || minY > cam.height) {
            this.drop(i);
            return;
        }
        if (solid) {
            this.recordSolid(i, src, m, minX, minY, maxX, maxY);
        }
    }

    private recordSolid(i: i32, src: StaticArray<Vec3>, m: i32, minX: f32, minY: f32, maxX: f32, maxY: f32): void {
        let nx: f32 = 0;
        let ny: f32 = 0;
        let nz: f32 = 0;
        let cx: f32 = 0;
        let cy: f32 = 0;
        let cz: f32 = 0;
        for (let k = 0; k < m; k++) {
            let a = src[k];
            let b = src[k + 1 < m ? k + 1 : 0];
            nx += (a.y - b.y) * (a.z + b.z);
            ny += (a.z - b.z) * (a.x + b.x);
            nz += (a.x - b.x) * (a.y + b.y);
            cx += a.x;
            cy += a.y;
            cz += a.z;
        }
        let l = Mathf.sqrt(nx * nx + ny * ny + nz * nz);
        if (l < 1e-12) {
            return;
        }
        let inv: f32 = 1.0 / l;
        let p = 4 * i;
        this.plane[p] = nx * inv;
        this.plane[p + 1] = ny * inv;
        this.plane[p + 2] = nz * inv;
        this.plane[p + 3] = -(nx * cx + ny * cy + nz * cz) * inv / <f32>m;
        this.screenBounds[p] = minX;
        this.screenBounds[p + 1] = minY;
        this.screenBounds[p + 2] = maxX;
        this.screenBounds[p + 3] = maxY;
        this.solid[i] = true;
    }

    // Clips cv[0..n) to z >= near into clipA, returning the new count.
    private clipNear(n: i32): i32 {
        let near = this.camera.near;
        let m = 0;
        for (let i = 0; i < n; i++) {
            let a = this.cv[i];
            let b = this.cv[i + 1 < n ? i + 1 : 0];
            let ina = a.z >= near;
            let inb = b.z >= near;
            if (ina && m < MAX_CLIP) {
                this.clipA[m++] = a;
            }
            if (ina != inb && m < MAX_CLIP) {
                let t = (near - a.z) / (b.z - a.z);
                this.clipA[m++] = Vec3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, near);
            }
        }
        return m;
    }

    // The camera facing half of a circle of radius r around center, in the
    // plane with the given normal, as open strips.
    private ring(center: Vec3, normal: Vec3, r: f32, depth: f32, color: Color, layer: i32): void {
        let cam = this.camera;
        let upish: f32 = Mathf.abs(normal.y) < 0.9 ? 1.0 : 0.0;
        let u = normalize(cross(normal, v3(1.0 - upish, upish, 0)));
        let w = cross(normal, u);
        let n = 32;
        let run = -1;
        let runLength = 0;
        // Start from a hidden point so a visible run never wraps.
        let first = 0;
        for (let k = 0; k < n; k++) {
            let a: f32 = <f32>k * 6.2831853 / <f32>n;
            let d = add(scale(u, Mathf.cos(a)), scale(w, Mathf.sin(a)));
            if (dot(d, sub(cam.eye, add(center, scale(d, r)))) <= 0) {
                first = k;
                break;
            }
        }
        for (let j = 0; j <= n; j++) {
            let k = (first + j) % n;
            let a: f32 = <f32>k * 6.2831853 / <f32>n;
            let d = add(scale(u, Mathf.cos(a)), scale(w, Mathf.sin(a)));
            let p = add(center, scale(d, r));
            let visible = dot(d, sub(cam.eye, p)) > 0 && j < n;
            let c = cam.toCamera(p);
            if (visible && c.z >= cam.near) {
                if (run < 0) {
                    run = this.alloc(K_STRIP, 2 * (n + 1), depth, 0, color, layer);
                    if (run < 0) {
                        return;
                    }
                    runLength = 0;
                }
                let s = cam.projectCamera(c);
                let o = this.start[run] + 2 * runLength;
                this.floats[o] = s.x;
                this.floats[o + 1] = s.y;
                runLength++;
            } else if (run >= 0) {
                this.finishRun(run, runLength);
                run = -1;
            }
        }
        if (run >= 0) {
            this.finishRun(run, runLength);
        }
    }

    private finishRun(run: i32, runLength: i32): void {
        this.count[run] = 2 * runLength;
        if (run == this.prims - 1) {
            this.used = this.start[run] + 2 * runLength;
        }
        if (runLength < 2) {
            this.count[run] = 0;
        }
    }

    private addLine(a: Vec3, b: Vec3, color: Color, arrow: bool): void {
        if (this.lines >= MAX_LINES) {
            this.droppedCount++;
            return;
        }
        this.lineA[this.lines] = a;
        this.lineB[this.lines] = b;
        this.lineColor[this.lines] = color;
        this.lineArrow[this.lines] = arrow;
        this.lines++;
    }

    // Shell sort of one layer's prims, far to near.
    private sortLayer(which: i32): i32 {
        let n = 0;
        for (let i = 0; i < this.prims; i++) {
            if (this.layer[i] == which) {
                this.order[n++] = i;
            }
        }
        let gap = 1;
        while (gap < n / 3) {
            gap = gap * 3 + 1;
        }
        let order = this.order;
        let depth = this.depth;
        while (gap > 0) {
            for (let i = gap; i < n; i++) {
                let v = order[i];
                let dv = depth[v];
                let j = i;
                while (j >= gap && depth[order[j - gap]] < dv) {
                    order[j] = order[j - gap];
                    j -= gap;
                }
                order[j] = v;
            }
            gap = (gap - 1) / 3;
        }
        return n;
    }

    private overlapOnScreen(a: i32, b: i32): bool {
        let bo = this.screenBounds;
        let p = 4 * a;
        let q = 4 * b;
        return bo[p] < bo[q + 2] && bo[q] < bo[p + 2] && bo[p + 1] < bo[q + 3] && bo[q + 1] < bo[p + 3];
    }

    // The area where the screen polygons of faces a and b overlap, with its centroid left in overlapX, overlapY.
    private overlapArea(a: i32, b: i32): f32 {
        let f = this.floats;
        let src = this.polyA;
        let dst = this.polyB;
        let m = this.count[a] / 2;
        let oa = this.start[a];
        for (let k = 0; k < 2 * m; k++) {
            src[k] = f[oa + k];
        }
        let nb = this.count[b] / 2;
        let ob = this.start[b];
        let winding: f32 = 0;
        for (let k = 0; k < nb; k++) {
            let j = k + 1 < nb ? k + 1 : 0;
            winding += f[ob + 2 * k] * f[ob + 2 * j + 1] - f[ob + 2 * j] * f[ob + 2 * k + 1];
        }
        let sign: f32 = winding >= 0 ? 1.0 : -1.0;
        let cap = 4 * MAX_CLIP;
        // Sutherland Hodgman: a convex polygon clipped by each edge of another.
        for (let e = 0; e < nb && m >= 3; e++) {
            let j = e + 1 < nb ? e + 1 : 0;
            let ex = f[ob + 2 * e];
            let ey = f[ob + 2 * e + 1];
            let dx = f[ob + 2 * j] - ex;
            let dy = f[ob + 2 * j + 1] - ey;
            let out = 0;
            for (let k = 0; k < m && out + 2 <= cap; k++) {
                let k2 = k + 1 < m ? k + 1 : 0;
                let px = src[2 * k];
                let py = src[2 * k + 1];
                let qx = src[2 * k2];
                let qy = src[2 * k2 + 1];
                let sp = sign * (dx * (py - ey) - dy * (px - ex));
                let sq = sign * (dx * (qy - ey) - dy * (qx - ex));
                if (sp >= 0) {
                    dst[2 * out] = px;
                    dst[2 * out + 1] = py;
                    out++;
                }
                if ((sp >= 0) != (sq >= 0)) {
                    let t = sp / (sp - sq);
                    dst[2 * out] = px + (qx - px) * t;
                    dst[2 * out + 1] = py + (qy - py) * t;
                    out++;
                }
            }
            let swap = src;
            src = dst;
            dst = swap;
            m = out;
        }
        if (m < 3) {
            return 0;
        }
        let area: f32 = 0;
        let cx: f32 = 0;
        let cy: f32 = 0;
        for (let k = 0; k < m; k++) {
            let j = k + 1 < m ? k + 1 : 0;
            let c = src[2 * k] * src[2 * j + 1] - src[2 * j] * src[2 * k + 1];
            area += c;
            cx += (src[2 * k] + src[2 * j]) * c;
            cy += (src[2 * k + 1] + src[2 * j + 1]) * c;
        }
        if (Mathf.abs(area) < 1e-6) {
            return 0;
        }
        this.overlapX = cx / (3 * area);
        this.overlapY = cy / (3 * area);
        return Mathf.abs(area) * 0.5;
    }

    // How far along the eye ray through screen point (sx, sy) face a's plane lies.
    private depthAt(a: i32, sx: f32, sy: f32): f32 {
        let cam = this.camera;
        let p = 4 * a;
        let den = this.plane[p] * (sx - cam.cx) / cam.focal + this.plane[p + 1] * (cam.cy - sy) / cam.focal + this.plane[p + 2];
        return Mathf.abs(den) < 1e-9 ? 1e9 : -this.plane[p + 3] / den;
    }

    // Shell sort of the k solids' indices by their screen left edge.
    private sortByLeft(k: i32): void {
        let byLeft = this.byLeft;
        let list = this.solids;
        let bounds = this.screenBounds;
        for (let i = 0; i < k; i++) {
            byLeft[i] = i;
        }
        let gap = 1;
        while (gap < k / 3) {
            gap = gap * 3 + 1;
        }
        while (gap > 0) {
            for (let i = gap; i < k; i++) {
                let v = byLeft[i];
                let left = bounds[4 * list[v]];
                let j = i;
                while (j >= gap && bounds[4 * list[byLeft[j - gap]]] > left) {
                    byLeft[j] = byLeft[j - gap];
                    j -= gap;
                }
                byLeft[j] = v;
            }
            gap = (gap - 1) / 3;
        }
    }

    // Reorders the solid faces of the depth sorted order[0..n) in place, leaving other prims where they are.
    // Where two faces overlap on screen, the one farther away inside the overlap paints first; a topological
    // sort keeps every such pair, and the centroid order otherwise. Centroid depth alone misorders faces of
    // tumbled or touching boxes, and plane side tests leave too many touching pairs undecided.
    private orderSolids(n: i32): void {
        let order = this.order;
        let slots = this.slots;
        let list = this.solids;
        let k = 0;
        for (let j = 0; j < n; j++) {
            if (this.solid[order[j]]) {
                slots[k] = j;
                list[k] = order[j];
                k++;
            }
        }
        let indegree = this.indegree;
        let adjStart = this.adjStart;
        for (let i = 0; i <= k; i++) {
            adjStart[i] = 0;
        }
        // Only faces whose screen spans overlap in x can overlap, so sweep them by left edge; each pair is still
        // measured from its farther face, as an all pairs loop would.
        let byLeft = this.byLeft;
        this.sortByLeft(k);
        let bounds = this.screenBounds;
        let edges = 0;
        for (let s = 0; s < k && edges < MAX_EDGES; s++) {
            let right = bounds[4 * list[byLeft[s]] + 2];
            for (let t = s + 1; t < k && edges < MAX_EDGES; t++) {
                if (bounds[4 * list[byLeft[t]]] >= right) {
                    break;
                }
                let i = min(byLeft[s], byLeft[t]);
                let j = max(byLeft[s], byLeft[t]);
                let a = list[i];
                let b = list[j];
                if (!this.overlapOnScreen(a, b) || this.overlapArea(a, b) < MIN_OVERLAP) {
                    continue;
                }
                let za = this.depthAt(a, this.overlapX, this.overlapY);
                let zb = this.depthAt(b, this.overlapX, this.overlapY);
                if (za == zb) {
                    continue;
                }
                let first = za > zb ? i : j;
                this.edgeFrom[edges] = first;
                this.edgeTo[edges] = first == i ? j : i;
                adjStart[first + 1]++;
                edges++;
            }
        }
        for (let i = 0; i < k; i++) {
            adjStart[i + 1] += adjStart[i];
            indegree[i] = 0;
            this.emitted[i] = false;
        }
        let adj = this.adj;
        for (let e = 0; e < edges; e++) {
            let from = this.edgeFrom[e];
            let to = this.edgeTo[e];
            adj[adjStart[from]] = to;
            adjStart[from]++;
            indegree[to]++;
        }
        // The fill above advanced each start to the next one's; shift back.
        for (let i = k; i > 0; i--) {
            adjStart[i] = adjStart[i - 1];
        }
        adjStart[0] = 0;
        let first = 0;
        for (let out = 0; out < k; out++) {
            while (this.emitted[first]) {
                first++;
            }
            // The farthest ready face; with none ready, a cycle, so the farthest left breaks it.
            let pick = first;
            for (let c = first; c < k; c++) {
                if (!this.emitted[c] && indegree[c] <= 0) {
                    pick = c;
                    break;
                }
            }
            this.emitted[pick] = true;
            order[slots[out]] = list[pick];
            for (let e = adjStart[pick]; e < adjStart[pick + 1]; e++) {
                indegree[adj[e]]--;
            }
        }
    }

    private paintLayer(renderer: Renderer, which: i32): void {
        let n = this.sortLayer(which);
        this.orderSolids(n);
        for (let j = 0; j < n; j++) {
            this.paintPrim(renderer, this.order[j]);
        }
    }

    private paintPrim(renderer: Renderer, i: i32): void {
        let path = this.path;
        let f = this.floats;
        let o = this.start[i];
        let kind = this.kind[i];
        path.reset();
        if (kind == K_POLY) {
            let m = this.count[i] / 2;
            path.moveTo(Vector.xy(f[o], f[o + 1]));
            for (let k = 1; k < m; k++) {
                path.lineTo(Vector.xy(f[o + 2 * k], f[o + 2 * k + 1]));
            }
            path.close();
            this.fillPaint.color = this.fill[i];
            renderer.drawPath(path, this.fillPaint);
            this.edgePaint.color = this.edge[i];
            renderer.drawPath(path, this.edgePaint);
            this.drawCalls += 2;
        } else if (kind == K_STRIP) {
            let m = this.count[i] / 2;
            if (m < 2) {
                return;
            }
            path.moveTo(Vector.xy(f[o], f[o + 1]));
            for (let k = 1; k < m; k++) {
                path.lineTo(Vector.xy(f[o + 2 * k], f[o + 2 * k + 1]));
            }
            this.edgePaint.color = this.edge[i];
            renderer.drawPath(path, this.edgePaint);
            this.drawCalls++;
        } else if (kind == K_SHADOW) {
            let m = (this.count[i] - 1) / 2;
            // Feathered fills need clockwise winding on screen.
            let area: f32 = 0;
            for (let k = 0; k < m; k++) {
                let j = (k + 1) % m;
                area += f[o + 1 + 2 * k] * f[o + 2 + 2 * j] - f[o + 1 + 2 * j] * f[o + 2 + 2 * k];
            }
            let soft = this.soft;
            soft.reset();
            for (let k = 0; k < m; k++) {
                let kk = area >= 0 ? k : m - 1 - k;
                let p = Vector.xy(f[o + 1 + 2 * kk], f[o + 2 + 2 * kk]);
                if (k == 0) {
                    soft.moveTo(p);
                } else {
                    soft.lineTo(p);
                }
            }
            soft.close();
            this.softPaint.color = this.fill[i];
            this.softPaint.feather = f[o];
            renderer.drawPath(soft, this.softPaint);
            this.drawCalls++;
        } else if (kind == K_DISC) {
            let x = f[o];
            let y = f[o + 1];
            let r = f[o + 2];
            addCircle(path, x, y, r);
            this.soft.reset();
            addCircle(this.soft, x + f[o + 3], y + f[o + 4], r * 0.6);
            this.paintRound(renderer, i, r);
        } else if (kind == K_STADIUM) {
            let r = Mathf.min(f[o + 2], f[o + 5]);
            addStadium(path, f[o], f[o + 1], f[o + 2], f[o + 3], f[o + 4], f[o + 5]);
            this.soft.reset();
            addStadium(this.soft, f[o] + f[o + 6], f[o + 1] + f[o + 7], f[o + 2] * 0.55,
                f[o + 3] + f[o + 6], f[o + 4] + f[o + 7], f[o + 5] * 0.55);
            this.paintRound(renderer, i, r);
        }
    }

    // Silhouette in path, highlight in soft: a shade tone fill, the feathered
    // highlight clipped to the silhouette, then the hairline.
    private paintRound(renderer: Renderer, i: i32, r: f32): void {
        this.fillPaint.color = scaleColor(this.fill[i], 0.55);
        renderer.drawPath(this.path, this.fillPaint);
        renderer.save();
        renderer.clipPath(this.path);
        this.softPaint.color = scaleColor(this.fill[i], 1.05);
        this.softPaint.feather = r * 0.85;
        renderer.drawPath(this.soft, this.softPaint);
        renderer.restore();
        this.edgePaint.color = this.edge[i];
        renderer.drawPath(this.path, this.edgePaint);
        this.drawCalls += 3;
    }

    private drawGrid(renderer: Renderer): void {
        let ext = this.gridExtent;
        let y = this.gridY;
        let path = this.path;
        let fe = <f32>ext;
        let tx = Mathf.round(this.camera.target.x);
        let tz = Mathf.round(this.camera.target.z);
        // Minor and major lines in three distance bands, fading outward.
        for (let pass = 0; pass < 6; pass++) {
            let major = pass >= 3;
            let band = pass % 3;
            let r0: f32 = fe * <f32>band / 3.0;
            let r1: f32 = fe * <f32>(band + 1) / 3.0;
            path.reset();
            let any = false;
            for (let axis = 0; axis < 2; axis++) {
                for (let i = -ext; i <= ext; i++) {
                    if ((i % 5 == 0) != major) {
                        continue;
                    }
                    for (let j = -ext; j < ext; j++) {
                        let fi = <f32>i;
                        let fj = <f32>j + 0.5;
                        let d = Mathf.max(Mathf.abs(fi), Mathf.abs(fj));
                        if (d < r0 || d >= r1) {
                            continue;
                        }
                        let ok = axis == 0
                            ? this.pathLine(path, v3(tx + fi, y, tz + <f32>j), v3(tx + fi, y, tz + <f32>(j + 1)))
                            : this.pathLine(path, v3(tx + <f32>j, y, tz + fi), v3(tx + <f32>(j + 1), y, tz + fi));
                        if (ok) {
                            any = true;
                        }
                    }
                }
            }
            if (any) {
                let base = major ? INK_GRID_MAJOR : INK_GRID;
                this.linePaint.color = withAlpha(base, 1.0 - <f32>band * 0.33);
                this.linePaint.thickness = 1;
                renderer.drawPath(path, this.linePaint);
                this.drawCalls++;
            }
        }
        this.linePaint.thickness = 1.25;
    }

    // Adds the visible part of a world segment to path.
    private pathLine(path: Path, a: Vec3, b: Vec3): bool {
        let cam = this.camera;
        let ca = cam.toCamera(a);
        let cb = cam.toCamera(b);
        let near = cam.near;
        if (ca.z < near && cb.z < near) {
            return false;
        }
        if (ca.z < near) {
            let t = (near - ca.z) / (cb.z - ca.z);
            ca = Vec3(ca.x + (cb.x - ca.x) * t, ca.y + (cb.y - ca.y) * t, near);
        } else if (cb.z < near) {
            let t = (near - cb.z) / (ca.z - cb.z);
            cb = Vec3(cb.x + (ca.x - cb.x) * t, cb.y + (ca.y - cb.y) * t, near);
        }
        let sa = cam.projectCamera(ca);
        let sb = cam.projectCamera(cb);
        path.moveTo(Vector.xy(sa.x, sa.y));
        path.lineTo(Vector.xy(sb.x, sb.y));
        return true;
    }

    private drawLines(renderer: Renderer): void {
        let path = this.path;
        let cam = this.camera;
        let i = 0;
        while (i < this.lines) {
            let color = this.lineColor[i];
            path.reset();
            while (i < this.lines && this.lineColor[i] == color) {
                let a = this.lineA[i];
                let b = this.lineB[i];
                if (this.pathLine(path, a, b) && this.lineArrow[i]) {
                    let ca = cam.toCamera(a);
                    let cb = cam.toCamera(b);
                    if (ca.z > cam.near && cb.z > cam.near) {
                        let sa = cam.projectCamera(ca);
                        let sb = cam.projectCamera(cb);
                        let dx = sb.x - sa.x;
                        let dy = sb.y - sa.y;
                        let l = Mathf.sqrt(dx * dx + dy * dy);
                        if (l > 1) {
                            let head = Mathf.min(7, l * 0.4);
                            let ux = dx / l * head;
                            let uy = dy / l * head;
                            path.moveTo(Vector.xy(sb.x - ux - uy * 0.5, sb.y - uy + ux * 0.5));
                            path.lineTo(Vector.xy(sb.x, sb.y));
                            path.lineTo(Vector.xy(sb.x - ux + uy * 0.5, sb.y - uy - ux * 0.5));
                        }
                    }
                }
                i++;
            }
            this.linePaint.color = color;
            renderer.drawPath(path, this.linePaint);
            this.drawCalls++;
        }
    }

    private drawPoints(renderer: Renderer): void {
        let path = this.path;
        let cam = this.camera;
        let i = 0;
        while (i < this.points) {
            let color = this.pointColor[i];
            path.reset();
            while (i < this.points && this.pointColor[i] == color) {
                let c = cam.toCamera(this.pointP[i]);
                if (c.z >= cam.near) {
                    let s = cam.projectCamera(c);
                    addCircle(path, s.x, s.y, this.pointSize[i] * 0.5);
                }
                i++;
            }
            this.fillPaint.color = color;
            renderer.drawPath(path, this.fillPaint);
            this.drawCalls++;
        }
    }

    private drawLabels(renderer: Renderer): void {
        let cam = this.camera;
        for (let i = 0; i < this.labels; i++) {
            let c = cam.toCamera(this.labelP[i]);
            if (c.z < cam.near) {
                continue;
            }
            let s = cam.projectCamera(c);
            let l = this.labelPool[i];
            renderer.save();
            renderer.transform(Mat2D.withTranslation(Mathf.round(s.x - l.width * 0.5), Mathf.round(s.y - 18)));
            this.fillPaint.color = this.labelColor[i];
            l.text.draw(renderer, this.fillPaint);
            renderer.restore();
            this.drawCalls++;
        }
    }
}
