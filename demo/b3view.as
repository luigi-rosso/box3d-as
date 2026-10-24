// Adapters from Box3D shapes to DebugDraw: transform conversion, derived
// geometry cached per shape data, and a per body list of what to draw.

import { Color } from "rive/color";
import { Vec3, Quat, Xf, length, xfPoint, quatMul } from "./vec";
import { DebugDraw, DebugHull, DebugMesh, DebugHeightField } from "./debug_draw";
import {
    B3_HEIGHT_FIELD_HOLE,
    b3BodyId,
    b3Body_GetShapeCount,
    b3Body_GetShapes,
    b3CompoundData,
    b3GetCompoundChild,
    b3HeightFieldData,
    b3HullData,
    b3MeshData,
    b3ShapeId,
    b3Shape_GetCapsule,
    b3Shape_GetCompound,
    b3Shape_GetHeightField,
    b3Shape_GetHull,
    b3Shape_GetMesh,
    b3Shape_GetSphere,
    b3Shape_GetType,
    b3Transform,
    b3Vec3,
    b3_capsuleShape,
    b3_compoundShape,
    b3_heightShape,
    b3_hullShape,
    b3_meshShape,
    b3_sphereShape,
} from "../lib/box3d";

@inline export function toVec3(v: b3Vec3): Vec3 {
    return Vec3(v.x, v.y, v.z);
}

@inline export function toB3(v: Vec3): b3Vec3 {
    return b3Vec3(v.x, v.y, v.z);
}

export function toXf(t: b3Transform): Xf {
    return Xf(Vec3(t.p.x, t.p.y, t.p.z), Quat(t.q.v.x, t.q.v.y, t.q.v.z, t.q.s));
}

export function mulXf(a: Xf, b: Xf): Xf {
    return Xf(xfPoint(a, b.p), quatMul(a.q, b.q));
}

// Face loops from the half edge ring of each face, counter clockwise from outside.
export function makeDebugHull(hull: b3HullData): DebugHull {
    let points = new StaticArray<Vec3>(hull.vertexCount);
    for (let i = 0; i < hull.vertexCount; i++) {
        points[i] = toVec3(hull.points[i]);
    }
    let faceStart = new StaticArray<i32>(hull.faceCount + 1);
    let indices = new Array<i32>();
    for (let f = 0; f < hull.faceCount; f++) {
        faceStart[f] = indices.length;
        let first = <i32>hull.faces[f].edge;
        let e = first;
        do {
            indices.push(<i32>hull.edges[e].origin);
            e = <i32>hull.edges[e].next;
        } while (e != first && indices.length < 4 * hull.edgeCount);
    }
    faceStart[hull.faceCount] = indices.length;
    let faceIndex = new StaticArray<i32>(indices.length);
    for (let i = 0; i < indices.length; i++) {
        faceIndex[i] = indices[i];
    }
    return new DebugHull(points, faceStart, faceIndex);
}

export function makeDebugMesh(data: b3MeshData, scale: b3Vec3): DebugMesh {
    let vertices = new StaticArray<Vec3>(data.vertexCount);
    for (let i = 0; i < data.vertexCount; i++) {
        let v = data.vertices[i];
        vertices[i] = Vec3(v.x * scale.x, v.y * scale.y, v.z * scale.z);
    }
    // A mirroring scale turns the winding inside out.
    let flip = scale.x * scale.y * scale.z < 0;
    let indices = new StaticArray<i32>(3 * data.triangleCount);
    for (let t = 0; t < data.triangleCount; t++) {
        let tri = data.triangles[t];
        indices[3 * t] = tri.index1;
        indices[3 * t + 1] = flip ? tri.index3 : tri.index2;
        indices[3 * t + 2] = flip ? tri.index2 : tri.index3;
    }
    return new DebugMesh(vertices, indices);
}

export function makeDebugHeightField(hf: b3HeightFieldData): DebugHeightField {
    let count = hf.columnCount * hf.rowCount;
    let heights = new StaticArray<f32>(count);
    for (let i = 0; i < count; i++) {
        heights[i] = hf.minHeight + hf.heightScale * <f32>hf.compressedHeights[i];
    }
    let field = new DebugHeightField(hf.columnCount, hf.rowCount, toVec3(hf.scale), heights);
    let cells = (hf.columnCount - 1) * (hf.rowCount - 1);
    let holes = new StaticArray<bool>(cells);
    let any = false;
    for (let i = 0; i < cells; i++) {
        holes[i] = <i32>hf.materialIndices[i] == <i32>B3_HEIGHT_FIELD_HOLE;
        any = any || holes[i];
    }
    if (any) {
        field.holes = holes;
    }
    return field;
}

// Bilinear height of a field in its local frame, or the lowest height off the grid.
export function fieldHeight(field: DebugHeightField, x: f32, z: f32): f32 {
    let fx = x / field.scale.x;
    let fz = z / field.scale.z;
    if (fx < 0 || fz < 0 || fx > <f32>(field.countX - 1) || fz > <f32>(field.countZ - 1)) {
        return -1e9;
    }
    let i = <i32>Mathf.min(Mathf.floor(fx), <f32>(field.countX - 2));
    let k = <i32>Mathf.min(Mathf.floor(fz), <f32>(field.countZ - 2));
    let u = fx - <f32>i;
    let w = fz - <f32>k;
    let n = field.countX;
    let h = field.heights;
    let a = h[k * n + i] + (h[k * n + i + 1] - h[k * n + i]) * u;
    let b = h[(k + 1) * n + i] + (h[(k + 1) * n + i + 1] - h[(k + 1) * n + i]) * u;
    return (a + (b - a) * w) * field.scale.y;
}

export const VIEW_SPHERE: i32 = 0;
export const VIEW_CAPSULE: i32 = 1;
export const VIEW_HULL: i32 = 2;
export const VIEW_MESH: i32 = 3;
export const VIEW_FIELD: i32 = 4;

// One drawable piece of a body: a shape, or one child of a compound.
export class ShapeView {
    kind: i32 = VIEW_SPHERE;
    // Child frame inside a compound.
    local: Xf = Xf(Vec3(0, 0, 0), Quat(0, 0, 0, 1));
    hasLocal: bool = false;
    center1: Vec3 = Vec3(0, 0, 0);
    center2: Vec3 = Vec3(0, 0, 0);
    radius: f32 = 0;
    hull: DebugHull? = null;
    mesh: DebugMesh? = null;
    field: DebugHeightField? = null;
}

class MeshEntry {
    data: b3MeshData;
    scale: b3Vec3;
    view: DebugMesh;
    constructor(data: b3MeshData, scale: b3Vec3, view: DebugMesh) {
        this.data = data;
        this.scale = scale;
        this.view = view;
    }
}

class FieldEntry {
    data: b3HeightFieldData;
    view: DebugHeightField;
    constructor(data: b3HeightFieldData, view: DebugHeightField) {
        this.data = data;
        this.view = view;
    }
}

// Derived geometry shared by every shape built from the same data. Hulls are
// keyed by their content hash, so the hundreds of copies of a box share one.
export class ShapeCache {
    hulls: Map<u64, DebugHull> = new Map<u64, DebugHull>();
    meshes: Array<MeshEntry> = new Array<MeshEntry>();
    fields: Array<FieldEntry> = new Array<FieldEntry>();

    clear(): void {
        this.hulls = new Map<u64, DebugHull>();
        this.meshes = new Array<MeshEntry>();
        this.fields = new Array<FieldEntry>();
    }

    hull(data: b3HullData): DebugHull {
        if (this.hulls.has(data.hash)) {
            return this.hulls.get(data.hash);
        }
        let view = makeDebugHull(data);
        this.hulls.set(data.hash, view);
        return view;
    }

    mesh(data: b3MeshData, scale: b3Vec3): DebugMesh {
        for (let i = 0; i < this.meshes.length; i++) {
            let e = this.meshes[i];
            if (e.data == data && e.scale.x == scale.x && e.scale.y == scale.y && e.scale.z == scale.z) {
                return e.view;
            }
        }
        let view = makeDebugMesh(data, scale);
        this.meshes.push(new MeshEntry(data, scale, view));
        return view;
    }

    field(data: b3HeightFieldData): DebugHeightField {
        for (let i = 0; i < this.fields.length; i++) {
            if (this.fields[i].data == data) {
                return this.fields[i].view;
            }
        }
        let view = makeDebugHeightField(data);
        this.fields.push(new FieldEntry(data, view));
        return view;
    }
}

// A body the scene draws, with its shapes resolved on first use.
export class BodyView {
    id: b3BodyId;
    ink: Color;
    // Static geometry that stands up, sorted with the bodies.
    standing: bool = false;
    shapes: Array<ShapeView> = new Array<ShapeView>();
    built: bool = false;
    // Distance from the body origin that bounds every shape, for shadows.
    reach: f32 = 0;

    constructor(id: b3BodyId, ink: Color) {
        this.id = id;
        this.ink = ink;
    }

    build(cache: ShapeCache): void {
        this.built = true;
        let count = b3Body_GetShapeCount(this.id);
        let ids = new StaticArray<b3ShapeId>(count);
        count = b3Body_GetShapes(this.id, ids, count);
        for (let i = 0; i < count; i++) {
            this.addShape(cache, ids[i]);
        }
        let reach: f32 = 0;
        for (let i = 0; i < this.shapes.length; i++) {
            reach = Mathf.max(reach, viewReach(this.shapes[i]));
        }
        this.reach = reach;
    }

    private addShape(cache: ShapeCache, shapeId: b3ShapeId): void {
        let type = b3Shape_GetType(shapeId);
        if (type == b3_sphereShape) {
            let s = b3Shape_GetSphere(shapeId);
            this.shapes.push(sphereView(toVec3(s.center), s.radius));
        } else if (type == b3_capsuleShape) {
            let c = b3Shape_GetCapsule(shapeId);
            this.shapes.push(capsuleView(toVec3(c.center1), toVec3(c.center2), c.radius));
        } else if (type == b3_hullShape) {
            let hull = b3Shape_GetHull(shapeId);
            if (hull != null) {
                let v = new ShapeView();
                v.kind = VIEW_HULL;
                v.hull = cache.hull(hull);
                this.shapes.push(v);
            }
        } else if (type == b3_meshShape) {
            let mesh = b3Shape_GetMesh(shapeId);
            let v = new ShapeView();
            v.kind = VIEW_MESH;
            v.mesh = cache.mesh(mesh.data, mesh.scale);
            this.shapes.push(v);
        } else if (type == b3_heightShape) {
            let hf = b3Shape_GetHeightField(shapeId);
            if (hf != null) {
                let v = new ShapeView();
                v.kind = VIEW_FIELD;
                v.field = cache.field(hf);
                this.shapes.push(v);
            }
        } else if (type == b3_compoundShape) {
            let compound = b3Shape_GetCompound(shapeId);
            if (compound != null) {
                this.addCompound(cache, compound);
            }
        }
    }

    private addCompound(cache: ShapeCache, compound: b3CompoundData): void {
        let count = compound.capsuleCount + compound.hullCount + compound.meshCount + compound.sphereCount;
        for (let i = 0; i < count; i++) {
            let child = b3GetCompoundChild(compound, i);
            let v: ShapeView;
            if (child.type == b3_sphereShape) {
                v = sphereView(toVec3(child.sphere.center), child.sphere.radius);
            } else if (child.type == b3_capsuleShape) {
                v = capsuleView(toVec3(child.capsule.center1), toVec3(child.capsule.center2), child.capsule.radius);
            } else if (child.type == b3_hullShape && child.hull != null) {
                v = new ShapeView();
                v.kind = VIEW_HULL;
                v.hull = cache.hull(child.hull!);
            } else if (child.type == b3_meshShape && child.mesh != null) {
                v = new ShapeView();
                v.kind = VIEW_MESH;
                v.mesh = cache.mesh(child.mesh!.data, child.mesh!.scale);
            } else {
                continue;
            }
            v.local = toXf(child.transform);
            v.hasLocal = true;
            this.shapes.push(v);
        }
    }

    draw(dd: DebugDraw, body: Xf, state: i32): void {
        for (let i = 0; i < this.shapes.length; i++) {
            let v = this.shapes[i];
            let xf = v.hasLocal ? mulXf(body, v.local) : body;
            let ink = this.ink;
            if (v.kind == VIEW_SPHERE) {
                dd.sphere(xf, v.center1, v.radius, ink, state);
            } else if (v.kind == VIEW_CAPSULE) {
                dd.capsule(xf, v.center1, v.center2, v.radius, ink, state);
            } else if (v.kind == VIEW_HULL) {
                dd.hull(xf, v.hull!, ink, state);
            } else if (v.kind == VIEW_MESH) {
                dd.mesh(xf, v.mesh!, ink, state);
            } else if (v.kind == VIEW_FIELD) {
                dd.heightField(xf, v.field!, ink, state);
            }
        }
    }
}

function sphereView(center: Vec3, radius: f32): ShapeView {
    let v = new ShapeView();
    v.kind = VIEW_SPHERE;
    v.center1 = center;
    v.radius = radius;
    return v;
}

function capsuleView(center1: Vec3, center2: Vec3, radius: f32): ShapeView {
    let v = new ShapeView();
    v.kind = VIEW_CAPSULE;
    v.center1 = center1;
    v.center2 = center2;
    v.radius = radius;
    return v;
}

function viewReach(v: ShapeView): f32 {
    let offset = v.hasLocal ? length(v.local.p) : 0;
    if (v.kind == VIEW_SPHERE) {
        return offset + length(v.center1) + v.radius;
    }
    if (v.kind == VIEW_CAPSULE) {
        return offset + Mathf.max(length(v.center1), length(v.center2)) + v.radius;
    }
    if (v.kind == VIEW_HULL) {
        return offset + v.hull!.radius;
    }
    return 0;
}
