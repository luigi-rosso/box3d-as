// Instanced GPU drawing of the demo's solids, with a reverse Z depth buffer,
// into a canvas DebugDraw composites under its vector overlays. Each hull,
// mesh or height field geometry is one draw for all its instances, every
// sphere and capsule one more, and the blob shadows one. Static geometry draws
// first, then the shadows, which test depth but never write it, then
// everything that moves, so a shadow never covers a body.

import { Context } from "rive/host";
import { Renderer } from "rive/renderer";
import { Mat2D } from "rive/mat2d";
import { BlendMode } from "rive/paint";
import { ImageFilter, ImageSampler, ImageWrap } from "rive/image";
import {
    GPUBindGroup,
    GPUBindGroupLayout,
    GPUBindGroupUBO,
    GPUBlendState,
    GPUBuffer,
    GPUCanvas,
    GPUColorAttachment,
    GPUColorTargetDesc,
    GPUDepthStencilDesc,
    GPUPassDesc,
    GPUPipeline,
    GPUPipelineDesc,
    GPURenderPass,
    GPUShaderModule,
    GPUTexture,
    GPUTextureDesc,
    GPUTextureView,
    GPUVertexAttribute,
    GPUVertexBufferLayout,
} from "rive/gpu";
import {
    BlendFactor,
    BufferUsage,
    CompareFunction,
    IndexFormat,
    LoadOp,
    StoreOp,
    TextureFormat,
    VertexFormat,
    VertexStepMode,
} from "rive/gpu_enums";
import { Vec3, Quat, Xf, cross, sub, normalize } from "./vec";
import { OrbitCamera } from "./camera";

const DEPTH_FORMAT: TextureFormat = TextureFormat.depth32float;
const SHADER: string = "demo/scene3d";

const VERTEX_WORDS: i32 = 9;
const FLAT_WORDS: i32 = 12;
const ROUND_WORDS: i32 = 14;
const SHADOW_WORDS: i32 = 8;
const VIEW_WORDS: i32 = 36;

const MAX_FLAT: i32 = 8192;
const MAX_ROUND: i32 = 4096;
const MAX_SHADOWS: i32 = 4096;
// Geometries one frame can draw; each gets a static and a moving batch.
const MAX_MESHES: i32 = 256;

// The unit capsule: rings of a sphere split at the equator into two caps.
const ROUND_SEGMENTS: i32 = 40;
const ROUND_RINGS: i32 = 10;

// A geometry in a vertex buffer: position, normal and edge distances per
// vertex, three vertices per triangle.
export class GpuMesh {
    buffer: GPUBuffer;
    vertices: u32;
    // The batch it fills this frame, when frame is the renderer's.
    slot: i32 = -1;
    frame: i32 = -1;

    constructor(buffer: GPUBuffer, vertices: u32) {
        this.buffer = buffer;
        this.vertices = vertices;
    }
}

// Collects triangles for a GpuMesh.
export class MeshWriter {
    private data: Array<f32> = new Array<f32>();

    // Triangle a, b, c with its normal; each flag says whether that edge is
    // on the face outline and so drawn.
    triangle(a: Vec3, b: Vec3, c: Vec3, n: Vec3, ab: bool, bc: bool, ca: bool): void {
        // Component i stays 1 when the edge opposite corner i is inside the face.
        let ia: f32 = bc ? 0.0 : 1.0;
        let ib: f32 = ca ? 0.0 : 1.0;
        let ic: f32 = ab ? 0.0 : 1.0;
        this.vertex(a, n, 1, ib, ic);
        this.vertex(b, n, ia, 1, ic);
        this.vertex(c, n, ia, ib, 1);
    }

    // A convex polygon as a fan from its first corner.
    polygon(points: StaticArray<Vec3>, n: i32, normal: Vec3): void {
        for (let i = 1; i + 1 < n; i++) {
            this.triangle(points[0], points[i], points[i + 1], normal, i == 1, true, i + 2 == n);
        }
    }

    finish(): GpuMesh? {
        let count = this.data.length;
        if (count == 0) {
            return null;
        }
        let words = new Float32Array(count);
        for (let i = 0; i < count; i++) {
            words[i] = this.data[i];
        }
        let buffer = new GPUBuffer(BufferUsage.vertex, <u32>(count * 4), words.buffer, true);
        return new GpuMesh(buffer, <u32>(count / VERTEX_WORDS));
    }

    private vertex(p: Vec3, n: Vec3, ba: f32, bb: f32, bc: f32): void {
        let d = this.data;
        d.push(p.x);
        d.push(p.y);
        d.push(p.z);
        d.push(n.x);
        d.push(n.y);
        d.push(n.z);
        d.push(ba);
        d.push(bb);
        d.push(bc);
    }
}

function renderTexture(width: u32, height: u32, format: TextureFormat, samples: u32): GPUTextureView {
    let desc = new GPUTextureDesc();
    desc.width = width;
    desc.height = height;
    desc.format = format;
    desc.renderTarget = true;
    desc.sampleCount = samples;
    return new GPUTexture(desc).view();
}

function attribute(format: VertexFormat, slot: u32, offset: u32): GPUVertexAttribute {
    return new GPUVertexAttribute(format, slot, offset);
}

function bufferLayout(stride: u32, step: VertexStepMode, attributes: Array<GPUVertexAttribute>): GPUVertexBufferLayout {
    let l = new GPUVertexBufferLayout();
    l.stride = stride;
    l.stepMode = step;
    l.attributes = attributes;
    return l;
}

function depthState(write: bool): GPUDepthStencilDesc {
    let d = new GPUDepthStencilDesc();
    d.format = DEPTH_FORMAT;
    d.compare = CompareFunction.greater;
    d.write = write;
    return d;
}

// Counts how many of the n records fall under each key, into starts[key + 1],
// then copies the records into dst grouped by key, leaving starts[key] at
// each group's first record.
function groupByKey(keys: StaticArray<i32>, src: Float32Array, dst: Float32Array, n: i32, words: i32,
    keyCount: i32, starts: StaticArray<i32>): void {
    for (let k = 0; k <= keyCount; k++) {
        starts[k] = 0;
    }
    for (let i = 0; i < n; i++) {
        starts[keys[i] + 1]++;
    }
    for (let k = 0; k < keyCount; k++) {
        starts[k + 1] += starts[k];
    }
    for (let i = 0; i < n; i++) {
        let at = starts[keys[i]]++;
        let o = i * words;
        let t = at * words;
        for (let w = 0; w < words; w++) {
            dst[t + w] = src[o + w];
        }
    }
    // The copy advanced each start to the next one's; shift back.
    for (let k = keyCount; k > 0; k--) {
        starts[k] = starts[k - 1];
    }
    starts[0] = 0;
}

export class GpuDraw {
    // Device pixels per point.
    scale: f32 = 1;
    // Triangles drawn last frame.
    triangles: i32 = 0;
    dropped: i32 = 0;

    private canvas: GPUCanvas;
    private samples: u32;
    private maxSize: u32;
    private color: GPUTextureView? = null;
    private depth: GPUTextureView? = null;
    private width: u32 = 0;
    private height: u32 = 0;

    private flatPipeline: GPUPipeline;
    private roundPipeline: GPUPipeline;
    private shadowPipeline: GPUPipeline;
    private group: GPUBindGroup;
    private viewBuffer: GPUBuffer = new GPUBuffer(BufferUsage.uniform, <u32>(VIEW_WORDS * 4));
    private viewData: Float32Array = new Float32Array(VIEW_WORDS);

    private cube: GpuMesh;
    private roundVertices: GPUBuffer;
    private roundIndices: GPUBuffer;
    private roundIndexCount: u32 = 0;

    private frame: i32 = 0;
    private meshes: StaticArray<GpuMesh?> = new StaticArray<GpuMesh?>(MAX_MESHES);
    private meshCount: i32 = 0;

    private flatKeys: StaticArray<i32> = new StaticArray<i32>(MAX_FLAT);
    private flatStage: Float32Array = new Float32Array(MAX_FLAT * FLAT_WORDS);
    private flatData: Float32Array = new Float32Array(MAX_FLAT * FLAT_WORDS);
    private flatStarts: StaticArray<i32> = new StaticArray<i32>(2 * MAX_MESHES + 1);
    private flatBuffer: GPUBuffer = new GPUBuffer(BufferUsage.vertex, <u32>(MAX_FLAT * FLAT_WORDS * 4));
    private flats: i32 = 0;

    private roundKeys: StaticArray<i32> = new StaticArray<i32>(MAX_ROUND);
    private roundStage: Float32Array = new Float32Array(MAX_ROUND * ROUND_WORDS);
    private roundData: Float32Array = new Float32Array(MAX_ROUND * ROUND_WORDS);
    private roundStarts: StaticArray<i32> = new StaticArray<i32>(3);
    private roundBuffer: GPUBuffer = new GPUBuffer(BufferUsage.vertex, <u32>(MAX_ROUND * ROUND_WORDS * 4));
    private rounds: i32 = 0;

    private shadowData: Float32Array = new Float32Array(MAX_SHADOWS * SHADOW_WORDS);
    private shadowBuffer: GPUBuffer = new GPUBuffer(BufferUsage.vertex, <u32>(MAX_SHADOWS * SHADOW_WORDS * 4));
    private shadows: i32 = 0;

    private gridOn: bool = false;
    private gridX: f32 = 0;
    private gridZ: f32 = 0;
    private gridExtent: f32 = 16;

    private passDesc: GPUPassDesc = new GPUPassDesc();
    private attachment: GPUColorAttachment = new GPUColorAttachment();
    private pass: GPURenderPass? = null;
    private sampler: ImageSampler = ImageSampler(ImageWrap.clamp, ImageWrap.clamp, ImageFilter.bilinear);

    // Null when the host has no GPU canvas or the shader is missing.
    static create(context: Context): GpuDraw? {
        let canvas = context.gpuCanvas();
        let shader = context.shader(SHADER);
        if (canvas == null || shader == null) {
            return null;
        }
        let layout = GPUBindGroupLayout.fromShader(shader, 0);
        if (layout == null) {
            return null;
        }
        let features = context.features();
        let samples: u32 = features.maxSamples >= 4 ? 4 : features.maxSamples;
        if (samples < 1) {
            samples = 1;
        }
        return new GpuDraw(canvas, shader, layout, samples, features.maxTextureSize2D);
    }

    constructor(canvas: GPUCanvas, shader: GPUShaderModule, layout: GPUBindGroupLayout, samples: u32, maxSize: u32) {
        this.canvas = canvas;
        this.samples = samples;
        this.maxSize = maxSize;
        let format = <TextureFormat>canvas.format;

        let perVertex = bufferLayout(<u32>(VERTEX_WORDS * 4), VertexStepMode.vertex, [
            attribute(VertexFormat.float3, 0, 0),
            attribute(VertexFormat.float3, 1, 12),
            attribute(VertexFormat.float3, 2, 24),
        ]);
        let flatInstance = bufferLayout(<u32>(FLAT_WORDS * 4), VertexStepMode.instance, [
            attribute(VertexFormat.float4, 3, 0),
            attribute(VertexFormat.float3, 4, 16),
            attribute(VertexFormat.float3, 5, 28),
            attribute(VertexFormat.unorm8x4, 6, 40),
            attribute(VertexFormat.unorm8x4, 7, 44),
        ]);
        this.flatPipeline = this.pipeline(shader, layout, format, "vs_flat", "fs_flat", [perVertex, flatInstance], true);

        let roundVertex = bufferLayout(16, VertexStepMode.vertex, [
            attribute(VertexFormat.float3, 0, 0),
            attribute(VertexFormat.float1, 1, 12),
        ]);
        let roundInstance = bufferLayout(<u32>(ROUND_WORDS * 4), VertexStepMode.instance, [
            attribute(VertexFormat.float4, 2, 0),
            attribute(VertexFormat.float4, 3, 16),
            attribute(VertexFormat.float4, 4, 32),
            attribute(VertexFormat.unorm8x4, 5, 48),
            attribute(VertexFormat.unorm8x4, 6, 52),
        ]);
        this.roundPipeline = this.pipeline(shader, layout, format, "vs_round", "fs_round", [roundVertex, roundInstance], true);
        let shadowInstance = bufferLayout(<u32>(SHADOW_WORDS * 4), VertexStepMode.instance, [
            attribute(VertexFormat.float4, 0, 0),
            attribute(VertexFormat.float4, 1, 16),
        ]);
        this.shadowPipeline = this.pipeline(shader, layout, format, "vs_shadow", "fs_shadow", [shadowInstance], false);
        this.group = new GPUBindGroup(layout, [new GPUBindGroupUBO(0, this.viewBuffer)]);

        this.cube = makeCube();
        let round = makeRound();
        this.roundVertices = round.vertices;
        this.roundIndices = round.indices;
        this.roundIndexCount = round.count;

        this.attachment.loadOp = LoadOp.clear;
        this.attachment.clearA = 0;
        this.passDesc.color = [this.attachment];
        this.passDesc.depthLoadOp = LoadOp.clear;
        this.passDesc.depthStoreOp = StoreOp.discard;
        this.passDesc.depthClearValue = 0;
        this.passDesc.label = "solids";
    }

    private pipeline(shader: GPUShaderModule, layout: GPUBindGroupLayout, format: TextureFormat, vertex: string,
        fragment: string, buffers: Array<GPUVertexBufferLayout>, opaque: bool): GPUPipeline {
        let target = new GPUColorTargetDesc();
        target.format = format;
        if (!opaque) {
            let blend = new GPUBlendState();
            blend.srcColor = BlendFactor.one;
            blend.dstColor = BlendFactor.oneMinusSrcAlpha;
            blend.srcAlpha = BlendFactor.one;
            blend.dstAlpha = BlendFactor.oneMinusSrcAlpha;
            target.blend = blend;
        }
        let desc = new GPUPipelineDesc();
        desc.vertexModule = shader;
        desc.vertexEntryPoint = vertex;
        desc.fragmentModule = shader;
        desc.fragmentEntryPoint = fragment;
        desc.colorTargets = [target];
        desc.vertexBuffers = buffers;
        desc.bindGroupLayouts = [layout];
        desc.depthStencil = depthState(opaque);
        desc.sampleCount = this.samples;
        return new GPUPipeline(desc);
    }

    begin(): void {
        this.frame++;
        this.meshCount = 0;
        this.flats = 0;
        this.rounds = 0;
        this.shadows = 0;
        this.dropped = 0;
        this.gridOn = false;
    }

    // ---- recording ----------------------------------------------------------

    box(xf: Xf, halfExtents: Vec3, fill: u32, edge: u32, still: bool): void {
        this.flat(this.cube, xf, halfExtents, fill, edge, still);
    }

    flat(mesh: GpuMesh, xf: Xf, scale: Vec3, fill: u32, edge: u32, still: bool): void {
        if (this.flats >= MAX_FLAT) {
            this.dropped++;
            return;
        }
        if (mesh.frame != this.frame) {
            if (this.meshCount >= MAX_MESHES) {
                this.dropped++;
                return;
            }
            mesh.frame = this.frame;
            mesh.slot = this.meshCount;
            this.meshes[this.meshCount++] = mesh;
        }
        let i = this.flats++;
        this.flatKeys[i] = 2 * mesh.slot + (still ? 0 : 1);
        let d = this.flatStage;
        let o = i * FLAT_WORDS;
        d[o] = xf.q.x;
        d[o + 1] = xf.q.y;
        d[o + 2] = xf.q.z;
        d[o + 3] = xf.q.w;
        d[o + 4] = xf.p.x;
        d[o + 5] = xf.p.y;
        d[o + 6] = xf.p.z;
        d[o + 7] = scale.x;
        d[o + 8] = scale.y;
        d[o + 9] = scale.z;
        d[o + 10] = reinterpret<f32>(fill);
        d[o + 11] = reinterpret<f32>(edge);
    }

    // A capsule between two world points, or a sphere when they coincide;
    // q orients a sphere's rings.
    round(c1: Vec3, c2: Vec3, radius: f32, q: Quat, fill: u32, edge: u32, still: bool): void {
        if (this.rounds >= MAX_ROUND) {
            this.dropped++;
            return;
        }
        let i = this.rounds++;
        this.roundKeys[i] = still ? 0 : 1;
        let d = this.roundStage;
        let o = i * ROUND_WORDS;
        d[o] = c1.x;
        d[o + 1] = c1.y;
        d[o + 2] = c1.z;
        d[o + 3] = radius;
        d[o + 4] = c2.x;
        d[o + 5] = c2.y;
        d[o + 6] = c2.z;
        d[o + 7] = 0;
        d[o + 8] = q.x;
        d[o + 9] = q.y;
        d[o + 10] = q.z;
        d[o + 11] = q.w;
        d[o + 12] = reinterpret<f32>(fill);
        d[o + 13] = reinterpret<f32>(edge);
    }

    // A soft disc on the ground at p, lifted toward the eye by lift.
    shadow(p: Vec3, spread: f32, alpha: f32, lift: f32): void {
        if (this.shadows >= MAX_SHADOWS) {
            this.dropped++;
            return;
        }
        let d = this.shadowData;
        let o = this.shadows++ * SHADOW_WORDS;
        d[o] = p.x;
        d[o + 1] = p.y;
        d[o + 2] = p.z;
        d[o + 3] = spread;
        d[o + 4] = alpha;
        d[o + 5] = lift;
        d[o + 6] = 0;
        d[o + 7] = 0;
    }

    // Grid lines on the ground's faces around x, z, in the ground ink's clear alpha.
    grid(x: f32, z: f32, extent: f32): void {
        this.gridOn = true;
        this.gridX = x;
        this.gridZ = z;
        this.gridExtent = extent;
    }

    // ---- drawing ------------------------------------------------------------

    // Renders what was recorded and composites it over whatever the renderer
    // holds, at the camera's viewport.
    render(renderer: Renderer, camera: OrbitCamera, light: Vec3, minorInk: u32, majorInk: u32): void {
        let pw = this.pixels(camera.width);
        let ph = this.pixels(camera.height);
        let canvas = this.canvas;
        if (pw != this.width || ph != this.height) {
            if (!canvas.resize(pw, ph)) {
                return;
            }
            this.width = pw;
            this.height = ph;
            this.color = this.samples > 1 ? renderTexture(pw, ph, <TextureFormat>canvas.format, this.samples) : null;
            this.depth = renderTexture(pw, ph, DEPTH_FORMAT, this.samples);
        }
        this.writeView(camera, light, minorInk, majorInk);

        let flatKeyCount = 2 * this.meshCount;
        groupByKey(this.flatKeys, this.flatStage, this.flatData, this.flats, FLAT_WORDS, flatKeyCount, this.flatStarts);
        groupByKey(this.roundKeys, this.roundStage, this.roundData, this.rounds, ROUND_WORDS, 2, this.roundStarts);
        if (this.flats > 0) {
            this.flatBuffer.update(0, this.flatData.buffer, <u32>(this.flats * FLAT_WORDS * 4));
        }
        if (this.rounds > 0) {
            this.roundBuffer.update(0, this.roundData.buffer, <u32>(this.rounds * ROUND_WORDS * 4));
        }
        if (this.shadows > 0) {
            this.shadowBuffer.update(0, this.shadowData.buffer, <u32>(this.shadows * SHADOW_WORDS * 4));
        }

        let a = this.attachment;
        if (this.samples > 1) {
            a.view = this.color;
            a.resolveTarget = canvas.colorView();
            a.storeOp = StoreOp.discard;
        } else {
            a.view = null;
            a.resolveTarget = null;
            a.storeOp = StoreOp.store;
        }
        this.passDesc.depthView = this.depth;
        let pass = canvas.beginRenderPass(this.passDesc, this.pass);
        this.pass = pass;
        pass.setBindGroup(0, this.group);
        this.triangles = 0;
        this.drawFlats(pass, 0);
        this.drawRounds(pass, 0);
        if (this.shadows > 0) {
            pass.setPipeline(this.shadowPipeline);
            pass.setVertexBuffer(0, this.shadowBuffer);
            pass.draw(6, <u32>this.shadows);
        }
        this.drawFlats(pass, 1);
        this.drawRounds(pass, 1);
        pass.finish();

        renderer.save();
        renderer.transform(Mat2D(camera.width / <f32>pw, 0, 0, camera.height / <f32>ph, 0, 0));
        renderer.drawImage(canvas.image, this.sampler, BlendMode.srcOver, 1);
        renderer.restore();
    }

    private pixels(points: f32): u32 {
        let p = <u32>Mathf.max(1, Mathf.round(points * this.scale));
        return p < this.maxSize ? p : this.maxSize;
    }

    private drawFlats(pass: GPURenderPass, layer: i32): void {
        let starts = this.flatStarts;
        let bound = false;
        for (let s = 0; s < this.meshCount; s++) {
            let key = 2 * s + layer;
            let n = starts[key + 1] - starts[key];
            if (n <= 0) {
                continue;
            }
            if (!bound) {
                pass.setPipeline(this.flatPipeline);
                bound = true;
            }
            let mesh = this.meshes[s]!;
            pass.setVertexBuffer(0, mesh.buffer);
            pass.setVertexBuffer(1, this.flatBuffer, <u32>(starts[key] * FLAT_WORDS * 4));
            pass.draw(mesh.vertices, <u32>n);
            this.triangles += n * <i32>(mesh.vertices / 3);
        }
    }

    private drawRounds(pass: GPURenderPass, layer: i32): void {
        let starts = this.roundStarts;
        let n = starts[layer + 1] - starts[layer];
        if (n <= 0) {
            return;
        }
        pass.setPipeline(this.roundPipeline);
        pass.setVertexBuffer(0, this.roundVertices);
        pass.setVertexBuffer(1, this.roundBuffer, <u32>(starts[layer] * ROUND_WORDS * 4));
        pass.setIndexBuffer(this.roundIndices, <u32>IndexFormat.uint16);
        pass.drawIndexed(this.roundIndexCount, <u32>n);
        this.triangles += n * <i32>(this.roundIndexCount / 3);
    }

    private writeView(camera: OrbitCamera, light: Vec3, minorInk: u32, majorInk: u32): void {
        let d = this.viewData;
        let w = camera.width;
        let h = camera.height;
        d[0] = camera.eye.x;
        d[1] = camera.eye.y;
        d[2] = camera.eye.z;
        d[3] = camera.near;
        d[4] = camera.right.x;
        d[5] = camera.right.y;
        d[6] = camera.right.z;
        d[7] = this.scale;
        d[8] = camera.up.x;
        d[9] = camera.up.y;
        d[10] = camera.up.z;
        d[11] = 0;
        d[12] = camera.forward.x;
        d[13] = camera.forward.y;
        d[14] = camera.forward.z;
        d[15] = 0;
        d[16] = 2 * camera.focal / w;
        d[17] = 2 * camera.focal / h;
        d[18] = 2 * camera.cx / w - 1;
        d[19] = 1 - 2 * camera.cy / h;
        d[20] = light.x;
        d[21] = light.y;
        d[22] = light.z;
        d[23] = 0;
        d[24] = this.gridX;
        d[25] = this.gridZ;
        d[26] = 0;
        d[27] = this.gridOn ? this.gridExtent : 0;
        writeInk(d, 28, minorInk);
        writeInk(d, 32, majorInk);
        this.viewBuffer.update(0, d.buffer);
    }
}

function writeInk(d: Float32Array, o: i32, c: u32): void {
    d[o] = <f32>((c >> 16) & 0xff) / 255.0;
    d[o + 1] = <f32>((c >> 8) & 0xff) / 255.0;
    d[o + 2] = <f32>(c & 0xff) / 255.0;
    d[o + 3] = <f32>((c >> 24) & 0xff) / 255.0;
}

// A unit cube, half extent 1, for boxes scaled per instance.
function makeCube(): GpuMesh {
    let w = new MeshWriter();
    let corners = new StaticArray<Vec3>(4);
    for (let axis = 0; axis < 3; axis++) {
        for (let side = 0; side < 2; side++) {
            let s: f32 = side == 1 ? 1.0 : -1.0;
            for (let k = 0; k < 4; k++) {
                // Around the face counter clockwise seen from outside.
                let u: f32 = (k == 1 || k == 2) ? 1.0 : -1.0;
                let v: f32 = (k >= 2) ? 1.0 : -1.0;
                if (side == 0) {
                    u = -u;
                }
                corners[k] = axisPoint(axis, s, u, v);
            }
            let n = normalize(cross(sub(corners[1], corners[0]), sub(corners[2], corners[0])));
            w.polygon(corners, 4, n);
        }
    }
    return w.finish()!;
}

// The point with coordinate s on axis and u, v on the two axes after it.
function axisPoint(axis: i32, s: f32, u: f32, v: f32): Vec3 {
    if (axis == 0) {
        return Vec3(s, u, v);
    }
    if (axis == 1) {
        return Vec3(v, s, u);
    }
    return Vec3(u, v, s);
}

class RoundMesh {
    vertices: GPUBuffer;
    indices: GPUBuffer;
    count: u32;
    constructor(vertices: GPUBuffer, indices: GPUBuffer, count: u32) {
        this.vertices = vertices;
        this.indices = indices;
        this.count = count;
    }
}

// Two hemispheres of unit directions, each tagged with the cap it belongs
// to, joined at the equator by a band that stretches into a capsule's side.
function makeRound(): RoundMesh {
    let columns = ROUND_SEGMENTS + 1;
    // Rows from the first cap's pole to its equator, then the second cap's equator to its pole.
    let rows = 2 * (ROUND_RINGS + 1);
    let vertices = new Float32Array(rows * columns * 4);
    let o = 0;
    for (let row = 0; row < rows; row++) {
        let second = row > ROUND_RINGS;
        let ring = second ? row - 1 : row;
        let phi: f32 = -1.5707964 + 3.1415927 * <f32>ring / <f32>(2 * ROUND_RINGS);
        let y = Mathf.sin(phi);
        let r = Mathf.cos(phi);
        for (let c = 0; c < columns; c++) {
            let theta: f32 = 6.2831853 * <f32>c / <f32>ROUND_SEGMENTS;
            vertices[o] = r * Mathf.cos(theta);
            vertices[o + 1] = y;
            vertices[o + 2] = -r * Mathf.sin(theta);
            vertices[o + 3] = second ? 1.0 : 0.0;
            o += 4;
        }
    }
    let count = (rows - 1) * ROUND_SEGMENTS * 6;
    let indices = new Uint16Array(count);
    let k = 0;
    for (let row = 0; row + 1 < rows; row++) {
        for (let c = 0; c < ROUND_SEGMENTS; c++) {
            let a = <u16>(row * columns + c);
            let b = <u16>(row * columns + c + 1);
            let e = <u16>((row + 1) * columns + c);
            let f = <u16>((row + 1) * columns + c + 1);
            indices[k++] = a;
            indices[k++] = b;
            indices[k++] = f;
            indices[k++] = a;
            indices[k++] = f;
            indices[k++] = e;
        }
    }
    let vb = new GPUBuffer(BufferUsage.vertex, <u32>(vertices.length * 4), vertices.buffer, true);
    let ib = new GPUBuffer(BufferUsage.index, <u32>(count * 2), indices.buffer, true);
    return new RoundMesh(vb, ib, <u32>count);
}
