// Instanced GPU drawing of the demo's solids, with a reverse Z depth buffer,
// into a canvas DebugDraw composites under its vector overlays. Each hull,
// mesh or height field geometry is one draw for all its instances, every
// sphere and capsule one more. The same draws first fill a depth atlas of
// cascaded sun shadows, which every solid casts and receives. With the maps
// off, blob shadows stand in: static geometry draws first, then the blobs,
// which test depth but never write it, then everything that moves, so a blob
// never covers a body.

import { Context } from "rive/host";
import { Renderer } from "rive/renderer";
import { Mat2D } from "rive/mat2d";
import { BlendMode } from "rive/paint";
import { Image, ImageFilter, ImageSampler, ImageWrap } from "rive/image";
import {
    GPUBindGroup,
    GPUBindGroupLayout,
    GPUBindGroupSampler,
    GPUBindGroupTexture,
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
    GPUSampler,
    GPUShaderModule,
    GPUTexture,
    GPUTextureDesc,
    GPUTextureView,
    GPUTextureViewDesc,
    GPUVertexAttribute,
    GPUVertexBufferLayout,
} from "rive/gpu";
import {
    BlendFactor,
    BufferUsage,
    CompareFunction,
    Filter,
    IndexFormat,
    LoadOp,
    StoreOp,
    TextureAspect,
    TextureFormat,
    VertexFormat,
    VertexStepMode,
    WrapMode,
} from "rive/gpu_enums";
import { Vec3, Quat, Xf, add, cross, sub, scale, normalize, rotate, length, lerp3 } from "./vec";
import { OrbitCamera } from "./camera";
import { BLEND, CASCADES, ShadowCascades, ViewSpan } from "./shadows";

const DEPTH_FORMAT: TextureFormat = TextureFormat.depth32float;
const SHADER: string = "demo/scene3d";

const VERTEX_WORDS: i32 = 9;
const FLAT_WORDS: i32 = 12;
const ROUND_WORDS: i32 = 14;
const SHADOW_WORDS: i32 = 8;
const VIEW_WORDS: i32 = 132;
const CASTER_WORDS: i32 = 16;

// Shadow qualities H steps through: maps with a 7 texel tent filter, soft
// maps, half size maps with a 5 texel tent for weaker devices, and blobs.
const SHADOW_HIGH: i32 = 0;
const SHADOW_LOW: i32 = 2;
const SHADOW_OFF: i32 = 3;
const SHADOW_NAMES: StaticArray<string> = ["high", "soft", "low", "off"];
// The filter scene3d.wgsl runs for each quality.
const SHADOW_FILTERS: StaticArray<f32> = [2, 3, 1, 0];
// The atlas holds the cascades two by two.
const ATLAS_HIGH: u32 = 4096;
const ATLAS_LOW: u32 = 2048;
// Receivers move this many texels along their normal, and compare this many texels of depth nearer the sun.
const NORMAL_OFFSET: f32 = 1.0;
const DEPTH_BIAS: f32 = 1.0;
// Casters push sloped faces back by this much of their slope.
const SLOPE_BIAS: f32 = 1.0;
// The sun's angular radius as a tangent, for the soft penumbra; wider than the real sun so it reads.
const SUN_SIZE: f32 = 0.05;
// Shadows fade out over the last share of their reach.
const FADE: f32 = 0.15;
// What light a surface out of the sun keeps: a little warm.
const SHADOW_TINT_R: f32 = 1.05;
const SHADOW_TINT_G: f32 = 0.99;
const SHADOW_TINT_B: f32 = 0.9;

const MAX_FLAT: i32 = 8192;
const MAX_ROUND: i32 = 4096;
const MAX_SHADOWS: i32 = 4096;
// Geometries one frame can draw; each gets a static and a moving batch.
const MAX_MESHES: i32 = 256;

// The unit capsule, rings of a sphere split at the equator into two caps,
// in two levels of detail; rounds smaller on screen than DETAIL_PIXELS take the coarse one.
const FINE_SEGMENTS: i32 = 40;
const FINE_RINGS: i32 = 10;
const COARSE_SEGMENTS: i32 = 16;
const COARSE_RINGS: i32 = 4;
const DETAIL_PIXELS: f32 = 14;

// A geometry in a vertex buffer: position, normal and edge distances per
// vertex, three vertices per triangle.
export class GpuMesh {
    buffer: GPUBuffer;
    vertices: u32;
    // Local bounds, for fitting the shadow cascades.
    center: Vec3 = Vec3(0, 0, 0);
    half: Vec3 = Vec3(0, 0, 0);
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
        let mesh = new GpuMesh(buffer, <u32>(count / VERTEX_WORDS));
        let lo = Vec3(words[0], words[1], words[2]);
        let hi = lo;
        for (let i = 0; i < count; i += VERTEX_WORDS) {
            let p = Vec3(words[i], words[i + 1], words[i + 2]);
            lo = Vec3(Mathf.min(lo.x, p.x), Mathf.min(lo.y, p.y), Mathf.min(lo.z, p.z));
            hi = Vec3(Mathf.max(hi.x, p.x), Mathf.max(hi.y, p.y), Mathf.max(hi.z, p.z));
        }
        mesh.center = scale(add(lo, hi), 0.5);
        mesh.half = scale(sub(hi, lo), 0.5);
        return mesh;
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
    // Follows the canvas across resizes.
    private image: Image;
    private samples: u32;
    private maxSize: u32;
    private width: u32 = 0;
    private height: u32 = 0;

    private flatPipeline: GPUPipeline;
    private roundPipeline: GPUPipeline;
    private shadowPipeline: GPUPipeline;
    private flatCaster: GPUPipeline;
    private roundCaster: GPUPipeline;
    private layout: GPUBindGroupLayout;
    private group: GPUBindGroup;
    private viewBuffer: GPUBuffer = new GPUBuffer(BufferUsage.uniform, <u32>(VIEW_WORDS * 4));
    private viewData: Float32Array = new Float32Array(VIEW_WORDS);

    private cube: GpuMesh;
    private fineRound: RoundMesh = makeRound(FINE_SEGMENTS, FINE_RINGS);
    private coarseRound: RoundMesh = makeRound(COARSE_SEGMENTS, COARSE_RINGS);

    private camera: OrbitCamera = new OrbitCamera();
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
    private roundStarts: StaticArray<i32> = new StaticArray<i32>(5);
    private roundBuffer: GPUBuffer = new GPUBuffer(BufferUsage.vertex, <u32>(MAX_ROUND * ROUND_WORDS * 4));
    private rounds: i32 = 0;

    private shadowData: Float32Array = new Float32Array(MAX_SHADOWS * SHADOW_WORDS);
    private shadowBuffer: GPUBuffer = new GPUBuffer(BufferUsage.vertex, <u32>(MAX_SHADOWS * SHADOW_WORDS * 4));
    private shadows: i32 = 0;

    private span: ViewSpan = new ViewSpan();

    private gridOn: bool = false;
    private gridX: f32 = 0;
    private gridZ: f32 = 0;
    private gridExtent: f32 = 16;

    private passDesc: GPUPassDesc = new GPUPassDesc();
    private attachment: GPUColorAttachment = new GPUColorAttachment();
    private pass: GPURenderPass? = null;

    shadowQuality: i32 = SHADOW_HIGH;
    private cascades: ShadowCascades = new ShadowCascades();
    private casterBuffers: Array<GPUBuffer> = new Array<GPUBuffer>();
    private casterGroups: Array<GPUBindGroup> = new Array<GPUBindGroup>();
    private shadowSampler: GPUSampler = new GPUSampler(Filter.linear, Filter.linear, Filter.nearest, WrapMode.clampToEdge,
        WrapMode.clampToEdge, CompareFunction.less);
    // Bound where the atlas would be while the atlas is drawn into.
    private standIn: GPUTexture = new GPUTexture(depthDesc(1));
    private standInView: GPUTextureView;
    private atlas: GPUTexture? = null;
    private atlasSize: u32 = 0;
    private shadowDesc: GPUPassDesc = new GPUPassDesc();
    private shadowPass: GPURenderPass? = null;
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
        this.image = canvas.image;
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
        this.flatCaster = caster(shader, layout, "vs_flat_cast", [perVertex, flatInstance]);
        this.roundCaster = caster(shader, layout, "vs_round_cast", [roundVertex, roundInstance]);

        this.layout = layout;
        this.standInView = this.standIn.view(depthOnly());
        for (let c = 0; c < CASCADES; c++) {
            let buffer = new GPUBuffer(BufferUsage.uniform, <u32>(CASTER_WORDS * 4));
            this.casterBuffers.push(buffer);
            this.casterGroups.push(this.bindGroup(buffer, this.standInView));
        }
        this.group = this.bindGroup(this.casterBuffers[0], this.standInView);
        this.shadowDesc.depthLoadOp = LoadOp.clear;
        this.shadowDesc.depthStoreOp = StoreOp.store;
        this.shadowDesc.depthClearValue = 1;
        this.shadowDesc.label = "shadows";
        if (maxSize < ATLAS_HIGH) {
            this.shadowQuality = SHADOW_LOW;
        }
        if (isDefined(NO_SHADOWS)) {
            this.shadowQuality = SHADOW_OFF;
        }

        this.cube = makeCube();

        this.attachment.loadOp = LoadOp.clear;
        this.attachment.clearA = 0;
        this.passDesc.color = [this.attachment];
        this.passDesc.depthLoadOp = LoadOp.clear;
        this.passDesc.depthStoreOp = StoreOp.discard;
        this.passDesc.depthClearValue = 0;
        this.passDesc.label = "solids";
    }

    private bindGroup(casterBuffer: GPUBuffer, atlas: GPUTextureView): GPUBindGroup {
        return new GPUBindGroup(this.layout, [new GPUBindGroupUBO(0, this.viewBuffer), new GPUBindGroupUBO(1, casterBuffer)],
            [new GPUBindGroupTexture(2, atlas)], [new GPUBindGroupSampler(3, this.shadowSampler)]);
    }

    // True when the solids cast shadow maps, so no blobs are needed.
    get castsShadows(): bool {
        return this.shadowQuality != SHADOW_OFF;
    }

    // Steps to the next shadow quality and names it.
    cycleShadows(): string {
        let q = (this.shadowQuality + 1) % SHADOW_NAMES.length;
        if (q != SHADOW_OFF && q != SHADOW_LOW && this.maxSize < ATLAS_HIGH) {
            q = SHADOW_LOW;
        }
        this.shadowQuality = q;
        return SHADOW_NAMES[q];
    }

    // The atlas at the size the quality wants, made again when that changes.
    private prepareAtlas(): void {
        let size = this.shadowQuality == SHADOW_LOW ? ATLAS_LOW : ATLAS_HIGH;
        if (size == this.atlasSize) {
            return;
        }
        let old = this.atlas;
        if (old != null) {
            old.release();
        }
        let atlas = new GPUTexture(depthDesc(size));
        this.atlas = atlas;
        this.atlasSize = size;
        this.shadowDesc.depthView = atlas.view();
        this.group = this.bindGroup(this.casterBuffers[0], atlas.view(depthOnly()));
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

    begin(camera: OrbitCamera): void {
        this.camera = camera;
        this.frame++;
        this.meshCount = 0;
        this.flats = 0;
        this.rounds = 0;
        this.shadows = 0;
        this.dropped = 0;
        this.gridOn = false;
        this.span.reset(camera);
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
        if (this.castsShadows) {
            // Its bounds: the local box turned and scaled.
            let q = xf.q;
            let ry = Vec3(2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.w * q.x));
            let h = Vec3(mesh.half.x * scale.x, mesh.half.y * scale.y, mesh.half.z * scale.z);
            let center = add(xf.p, rotate(q, Vec3(mesh.center.x * scale.x, mesh.center.y * scale.y, mesh.center.z * scale.z)));
            let rise = Mathf.abs(ry.x * h.x) + Mathf.abs(ry.y * h.y) + Mathf.abs(ry.z * h.z);
            this.span.add(center, length(h), center.y - rise, center.y + rise);
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
    round(c1: Vec3, c2: Vec3, radius: f32, q: Quat, fill: u32, edge: u32, still: bool, rings: bool): void {
        if (this.rounds >= MAX_ROUND) {
            this.dropped++;
            return;
        }
        let cam = this.camera;
        let near = Mathf.min(cam.toCamera(c1).z, cam.toCamera(c2).z);
        let fine = cam.focal * radius * this.scale > DETAIL_PIXELS * Mathf.max(near, cam.near);
        if (this.castsShadows) {
            this.span.add(lerp3(c1, c2, 0.5), length(sub(c2, c1)) * 0.5 + radius, Mathf.min(c1.y, c2.y) - radius,
                Mathf.max(c1.y, c2.y) + radius);
        }
        let i = this.rounds++;
        this.roundKeys[i] = (still ? 0 : 2) + (fine ? 1 : 0);
        let d = this.roundStage;
        let o = i * ROUND_WORDS;
        d[o] = c1.x;
        d[o + 1] = c1.y;
        d[o + 2] = c1.z;
        d[o + 3] = radius;
        d[o + 4] = c2.x;
        d[o + 5] = c2.y;
        d[o + 6] = c2.z;
        d[o + 7] = rings ? 1.0 : 0.0;
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
            this.attach(canvas);
        }
        let maps = this.castsShadows && !this.span.empty;
        if (maps) {
            this.prepareAtlas();
            this.cascades.fit(camera, light, <i32>(this.atlasSize / 2), this.span);
        }
        this.writeView(camera, light, minorInk, majorInk, maps);

        let flatKeyCount = 2 * this.meshCount;
        groupByKey(this.flatKeys, this.flatStage, this.flatData, this.flats, FLAT_WORDS, flatKeyCount, this.flatStarts);
        groupByKey(this.roundKeys, this.roundStage, this.roundData, this.rounds, ROUND_WORDS, 4, this.roundStarts);
        if (this.flats > 0) {
            this.flatBuffer.update(0, this.flatData.buffer, <u32>(this.flats * FLAT_WORDS * 4));
        }
        if (this.rounds > 0) {
            this.roundBuffer.update(0, this.roundData.buffer, <u32>(this.rounds * ROUND_WORDS * 4));
        }
        if (this.shadows > 0) {
            this.shadowBuffer.update(0, this.shadowData.buffer, <u32>(this.shadows * SHADOW_WORDS * 4));
        }

        if (maps) {
            this.drawCasters(canvas);
        }

        let pass = canvas.beginRenderPass(this.passDesc, this.pass);
        this.pass = pass;
        pass.setBindGroup(0, this.group);
        this.triangles = 0;
        this.drawFlats(pass, 0, this.flatPipeline);
        this.drawRounds(pass, 0, this.roundPipeline);
        if (this.shadows > 0 && !maps) {
            pass.setPipeline(this.shadowPipeline);
            pass.setVertexBuffer(0, this.shadowBuffer);
            pass.draw(6, <u32>this.shadows);
        }
        this.drawFlats(pass, 1, this.flatPipeline);
        this.drawRounds(pass, 1, this.roundPipeline);
        pass.finish();

        renderer.save();
        renderer.transform(Mat2D(camera.width / <f32>pw, 0, 0, camera.height / <f32>ph, 0, 0));
        renderer.drawImage(this.image, this.sampler, BlendMode.srcOver, 1);
        renderer.restore();
    }

    // Targets sized to the canvas: multisampled color resolving into it, and depth.
    private attach(canvas: GPUCanvas): void {
        let a = this.attachment;
        if (this.samples > 1) {
            a.view = renderTexture(this.width, this.height, <TextureFormat>canvas.format, this.samples);
            a.resolveTarget = canvas.colorView();
            a.storeOp = StoreOp.discard;
        } else {
            a.view = null;
            a.storeOp = StoreOp.store;
        }
        this.passDesc.depthView = renderTexture(this.width, this.height, DEPTH_FORMAT, this.samples);
    }

    private pixels(points: f32): u32 {
        let p = <u32>Mathf.max(1, Mathf.round(points * this.scale));
        return p < this.maxSize ? p : this.maxSize;
    }

    // Every solid into each cascade's tile of the atlas.
    private drawCasters(canvas: GPUCanvas): void {
        let matrices = this.cascades.matrices.buffer;
        for (let c = 0; c < CASCADES; c++) {
            this.casterBuffers[c].update(0, matrices, <u32>(CASTER_WORDS * 4), <u32>(c * CASTER_WORDS * 4));
        }
        let pass = canvas.beginRenderPass(this.shadowDesc, this.shadowPass);
        this.shadowPass = pass;
        let tile = <f32>(this.atlasSize / 2);
        for (let c = 0; c < CASCADES; c++) {
            pass.setViewport(<f32>(c & 1) * tile, <f32>(c >> 1) * tile, tile, tile);
            pass.setBindGroup(0, this.casterGroups[c]);
            for (let layer = 0; layer < 2; layer++) {
                this.drawFlats(pass, layer, this.flatCaster);
                // Past the first cascade a texel spans more than the fine mesh's facets.
                this.drawRound(pass, 2 * layer, this.coarseRound, this.roundCaster);
                this.drawRound(pass, 2 * layer + 1, c == 0 ? this.fineRound : this.coarseRound, this.roundCaster);
            }
        }
        pass.finish();
    }

    private drawFlats(pass: GPURenderPass, layer: i32, pipeline: GPUPipeline): void {
        let starts = this.flatStarts;
        let bound = false;
        for (let s = 0; s < this.meshCount; s++) {
            let key = 2 * s + layer;
            let n = starts[key + 1] - starts[key];
            if (n <= 0) {
                continue;
            }
            if (!bound) {
                pass.setPipeline(pipeline);
                bound = true;
            }
            let mesh = this.meshes[s]!;
            pass.setVertexBuffer(0, mesh.buffer);
            pass.setVertexBuffer(1, this.flatBuffer, <u32>(starts[key] * FLAT_WORDS * 4));
            pass.draw(mesh.vertices, <u32>n);
            this.triangles += n * <i32>(mesh.vertices / 3);
        }
    }

    private drawRounds(pass: GPURenderPass, layer: i32, pipeline: GPUPipeline): void {
        this.drawRound(pass, 2 * layer, this.coarseRound, pipeline);
        this.drawRound(pass, 2 * layer + 1, this.fineRound, pipeline);
    }

    private drawRound(pass: GPURenderPass, key: i32, mesh: RoundMesh, pipeline: GPUPipeline): void {
        let starts = this.roundStarts;
        let n = starts[key + 1] - starts[key];
        if (n <= 0) {
            return;
        }
        pass.setPipeline(pipeline);
        pass.setVertexBuffer(0, mesh.vertices);
        pass.setVertexBuffer(1, this.roundBuffer, <u32>(starts[key] * ROUND_WORDS * 4));
        pass.setIndexBuffer(mesh.indices, <u32>IndexFormat.uint16);
        pass.drawIndexed(mesh.count, <u32>n);
        this.triangles += n * <i32>(mesh.count / 3);
    }

    private writeView(camera: OrbitCamera, light: Vec3, minorInk: u32, majorInk: u32, maps: bool): void {
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
        this.writeShadows(d, maps);
        this.viewBuffer.update(0, d.buffer);
    }

    private writeShadows(d: Float32Array, maps: bool): void {
        let cascades = this.cascades;
        let m = cascades.matrices;
        for (let k = 0; k < CASCADES * 16; k++) {
            d[36 + k] = m[k];
        }
        for (let c = 0; c < CASCADES; c++) {
            d[100 + c] = cascades.splits[c];
            d[104 + c] = cascades.texels[c];
            d[108 + c] = cascades.texelDepths[c];
        }
        d[112] = 1 / Mathf.max(<f32>this.atlasSize, 1);
        d[113] = maps ? SHADOW_FILTERS[this.shadowQuality] : 0;
        d[114] = NORMAL_OFFSET;
        d[115] = DEPTH_BIAS;
        d[116] = maps ? SHADOW_TINT_R : 1;
        d[117] = maps ? SHADOW_TINT_G : 1;
        d[118] = maps ? SHADOW_TINT_B : 1;
        d[119] = maps && isDefined(CSM_DEBUG) ? 1 : 0;
        d[120] = cascades.far * (1 - FADE);
        d[121] = BLEND;
        d[122] = SUN_SIZE;
        d[123] = cascades.near;
        writeVec(d, 124, cascades.right);
        writeVec(d, 128, cascades.up);
    }
}

function writeVec(d: Float32Array, o: i32, v: Vec3): void {
    d[o] = v.x;
    d[o + 1] = v.y;
    d[o + 2] = v.z;
    d[o + 3] = 0;
}

function depthDesc(size: u32): GPUTextureDesc {
    let desc = new GPUTextureDesc();
    desc.width = size;
    desc.height = size;
    desc.format = DEPTH_FORMAT;
    desc.renderTarget = true;
    return desc;
}

// A view the shader samples as texture_depth_2d.
function depthOnly(): GPUTextureViewDesc {
    let desc = new GPUTextureViewDesc();
    desc.aspect = TextureAspect.depthOnly;
    return desc;
}

// A depth only pipeline drawing into one cascade's tile, sloped faces pushed back so lit faces do not shadow themselves.
function caster(shader: GPUShaderModule, layout: GPUBindGroupLayout, vertex: string,
    buffers: Array<GPUVertexBufferLayout>): GPUPipeline {
    let depth = new GPUDepthStencilDesc();
    depth.format = DEPTH_FORMAT;
    depth.compare = CompareFunction.less;
    depth.write = true;
    depth.depthBiasSlopeScale = SLOPE_BIAS;
    let desc = new GPUPipelineDesc();
    desc.vertexModule = shader;
    desc.vertexEntryPoint = vertex;
    desc.vertexBuffers = buffers;
    desc.bindGroupLayouts = [layout];
    desc.depthStencil = depth;
    return new GPUPipeline(desc);
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
function makeRound(segments: i32, rings: i32): RoundMesh {
    let columns = segments + 1;
    // Rows from the first cap's pole to its equator, then the second cap's equator to its pole.
    let rows = 2 * (rings + 1);
    let vertices = new Float32Array(rows * columns * 4);
    let o = 0;
    for (let row = 0; row < rows; row++) {
        let second = row > rings;
        let ring = second ? row - 1 : row;
        let phi: f32 = -1.5707964 + 3.1415927 * <f32>ring / <f32>(2 * rings);
        let y = Mathf.sin(phi);
        let r = Mathf.cos(phi);
        for (let c = 0; c < columns; c++) {
            let theta: f32 = 6.2831853 * <f32>c / <f32>segments;
            vertices[o] = r * Mathf.cos(theta);
            vertices[o + 1] = y;
            vertices[o + 2] = -r * Mathf.sin(theta);
            vertices[o + 3] = second ? 1.0 : 0.0;
            o += 4;
        }
    }
    let count = (rows - 1) * segments * 6;
    let indices = new Uint16Array(count);
    let k = 0;
    for (let row = 0; row + 1 < rows; row++) {
        for (let c = 0; c < segments; c++) {
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
