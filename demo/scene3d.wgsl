// The physics demo's solids, drawn instanced with a reverse Z depth buffer,
// lit by a sun that casts cascaded shadow maps. The projection matches
// OrbitCamera's, so the vector overlays drawn on top line up. Edges are drawn
// in the fragment shader from each vertex's distance to the face outline, so
// the faceted look needs no line pass, and the grid is drawn on the ground's
// upward faces, so it follows uneven ground instead of cutting through it.
// Blob shadows stand in when the shadow maps are off.

struct View {
    eye: vec4f,      // eye position, near plane
    right: vec4f,    // camera right, device pixels per point
    up: vec4f,       // camera up
    forward: vec4f,  // camera forward
    proj: vec4f,     // camera x and y over depth to clip x and y, then the clip offset of the projection center
    light: vec4f,    // toward the sun
    grid: vec4f,     // grid center x and z, unused, half extent or 0 for none
    gridMinor: vec4f,
    gridMajor: vec4f,
    // World to each cascade's tile, x and y in clip space, depth away from the sun from 0 to 1.
    cascades: array<mat4x4f, 4>,
    // The view depth each cascade ends at.
    splits: vec4f,
    // World size of a texel per cascade, and the light depth it spans.
    texels: vec4f,
    texelDepths: vec4f,
    // Atlas texel in uv, filter (0 off, 1 tent 5, 2 tent 7, 3 soft), normal offset and depth bias in texels.
    shadow: vec4f,
    // 1 in x tints each cascade.
    debug: vec4f,
    // View depth shadows start fading at, blend share between cascades, the sun's angular radius as a
    // tangent, and the view depth the first cascade starts at.
    shadowFade: vec4f,
    // The light's axes across the sun.
    lightRight: vec4f,
    lightUp: vec4f,
}

struct Caster {
    matrix: mat4x4f,
}

@group(0) @binding(0) var<uniform> view: View;
// One cascade's matrix, for the passes that draw into the atlas.
@group(0) @binding(1) var<uniform> caster: Caster;
// The cascades two by two; the caster passes bind a stand in.
@group(0) @binding(2) var shadowAtlas: texture_depth_2d;
@group(0) @binding(3) var shadowSampler: sampler_comparison;

// Edge half width per face, in points; two faces share an inner edge.
const EDGE: f32 = 0.6;
// Ring and silhouette half width on round shapes, in points.
const LINE: f32 = 0.55;

fn project(p: vec3f) -> vec4f {
    let d = p - view.eye.xyz;
    let c = vec3f(dot(d, view.right.xyz), dot(d, view.up.xyz), dot(d, view.forward.xyz));
    return vec4f(c.x * view.proj.x + view.proj.z * c.z, c.y * view.proj.y + view.proj.w * c.z, view.eye.w, c.z);
}

fn qrot(q: vec4f, v: vec3f) -> vec3f {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

// Light from a slightly cool sky above, a dimmer bounce below and a slightly warm sun, balanced so a lit top
// face is neutral and as bright as the painter's, and shade keeps two thirds of it.
const SKY: vec3f = vec3f(0.65, 0.66, 0.68);
const BOUNCE: vec3f = vec3f(0.46, 0.46, 0.46);
const SUN: vec3f = vec3f(0.46, 0.44, 0.41);

// Hemisphere light plus Lambert from the sun, as much of it as reaches the surface.
fn lit(base: vec3f, n: vec3f, sun: f32) -> vec3f {
    let direct = max(dot(n, view.light.xyz), 0.0) * sun;
    return min(base * (mix(BOUNCE, SKY, 0.5 + 0.5 * n.y) + SUN * direct), vec3f(1.0));
}

// Colors arrive as 0xAARRGGBB words read as bytes, so blue first.
fn ink(c: vec4f) -> vec4f {
    return vec4f(c.z, c.y, c.x, c.w);
}

// Coverage of a line of half width w pixels at distance d pixels.
fn cover(d: f32, w: f32) -> f32 {
    return clamp(w + 0.5 - d, 0.0, 1.0);
}

// ---- shadows --------------------------------------------------------------

// A receiver as one cascade sees it: its atlas uv and light depth, how that
// depth changes per texel across the map, and the tile it stays inside.
struct Probe {
    uv: vec2f,
    z: f32,
    slope: vec2f,
    lo: vec2f,
    hi: vec2f,
}

// One bilinear comparison at atlas uv `at`, against the receiver's plane there pulled `lift` nearer the sun.
fn tapLifted(p: Probe, at: vec2f, lift: f32) -> f32 {
    let d = (at - p.uv) / view.shadow.x;
    return textureSampleCompareLevel(shadowAtlas, shadowSampler, clamp(at, p.lo, p.hi), p.z + dot(d, p.slope) - lift);
}

fn tap(p: Probe, at: vec2f) -> f32 {
    return tapLifted(p, at, 0.0);
}

// Tent filters from bilinear comparisons, after Castano's optimized PCF: 9
// taps for a 5 texel tent, 16 for a 7 texel one.
fn tent5(p: Probe) -> f32 {
    let texel = view.shadow.x;
    let at = p.uv / texel;
    let base = floor(at + 0.5);
    let st = at + 0.5 - base;
    let corner = (base - 0.5) * texel;
    let w0 = 4.0 - 3.0 * st;
    let w2 = 1.0 + 3.0 * st;
    let o0 = (3.0 - 2.0 * st) / w0 - 2.0;
    let o1 = (3.0 + st) / 7.0;
    let o2 = st / w2 + 2.0;
    var sum = 0.0;
    sum += w0.x * w0.y * tap(p, corner + vec2f(o0.x, o0.y) * texel);
    sum += 7.0 * w0.y * tap(p, corner + vec2f(o1.x, o0.y) * texel);
    sum += w2.x * w0.y * tap(p, corner + vec2f(o2.x, o0.y) * texel);
    sum += w0.x * 7.0 * tap(p, corner + vec2f(o0.x, o1.y) * texel);
    sum += 49.0 * tap(p, corner + vec2f(o1.x, o1.y) * texel);
    sum += w2.x * 7.0 * tap(p, corner + vec2f(o2.x, o1.y) * texel);
    sum += w0.x * w2.y * tap(p, corner + vec2f(o0.x, o2.y) * texel);
    sum += 7.0 * w2.y * tap(p, corner + vec2f(o1.x, o2.y) * texel);
    sum += w2.x * w2.y * tap(p, corner + vec2f(o2.x, o2.y) * texel);
    return sum / 144.0;
}

fn tent7Row(p: Probe, corner: vec2f, u: vec4f, w: vec4f, v: f32, wv: f32) -> f32 {
    let texel = view.shadow.x;
    var sum = 0.0;
    sum += w.x * tap(p, corner + vec2f(u.x, v) * texel);
    sum += w.y * tap(p, corner + vec2f(u.y, v) * texel);
    sum += w.z * tap(p, corner + vec2f(u.z, v) * texel);
    sum += w.w * tap(p, corner + vec2f(u.w, v) * texel);
    return sum * wv;
}

fn tent7(p: Probe) -> f32 {
    let texel = view.shadow.x;
    let at = p.uv / texel;
    let base = floor(at + 0.5);
    let st = at + 0.5 - base;
    let corner = (base - 0.5) * texel;
    let s = st.x;
    let t = st.y;
    let wu = vec4f(5.0 * s - 6.0, 11.0 * s - 28.0, -(11.0 * s + 17.0), -(5.0 * s + 1.0));
    let ou = vec4f((4.0 * s - 5.0) / wu.x - 3.0, (4.0 * s - 16.0) / wu.y - 1.0, -(7.0 * s + 5.0) / wu.z + 1.0, -s / wu.w + 3.0);
    let wv = vec4f(5.0 * t - 6.0, 11.0 * t - 28.0, -(11.0 * t + 17.0), -(5.0 * t + 1.0));
    let ov = vec4f((4.0 * t - 5.0) / wv.x - 3.0, (4.0 * t - 16.0) / wv.y - 1.0, -(7.0 * t + 5.0) / wv.z + 1.0, -t / wv.w + 3.0);
    var sum = tent7Row(p, corner, ou, wu, ov.x, wv.x);
    sum += tent7Row(p, corner, ou, wu, ov.y, wv.y);
    sum += tent7Row(p, corner, ou, wu, ov.z, wv.z);
    sum += tent7Row(p, corner, ou, wu, ov.w, wv.w);
    return sum / 2704.0;
}

const SOFT_TAPS: i32 = 16;
const SEARCH_TAPS: i32 = 8;
// Texels the blocker search reaches, and the penumbra widths it tells apart.
const SOFT_SEARCH: f32 = 10.0;
const SOFT_WIDTHS: vec3f = vec3f(3.0, 7.0, 14.0);

// A Vogel disc point of `count`, radius 1.
fn disc(i: i32, count: i32) -> vec2f {
    let r = sqrt((f32(i) + 0.5) / f32(count));
    let a = f32(i) * 2.3999632;
    return vec2f(cos(a), sin(a)) * r;
}

// Percentage closer soft shadows: how far blockers stand from the receiver
// sets how wide its penumbra grows. GLSL cannot read raw depth, so the
// search counts blockers at a few distances with comparisons instead of
// averaging their depths.
fn soft(p: Probe, texelDepth: f32) -> f32 {
    let texel = view.shadow.x;
    // Blocker gaps, in light depth, that cast each width.
    let gaps = SOFT_WIDTHS / view.shadowFade.z * texelDepth;
    var near = 0.0;
    var counts = vec3f(0.0);
    for (var i = 0; i < SEARCH_TAPS; i++) {
        let at = p.uv + disc(i, SEARCH_TAPS) * SOFT_SEARCH * texel;
        near += 1.0 - tapLifted(p, at, 0.0);
        counts += vec3f(1.0) - vec3f(tapLifted(p, at, gaps.x), tapLifted(p, at, gaps.y), tapLifted(p, at, gaps.z));
    }
    if (near < 0.01) {
        return 1.0;
    }
    // The share of blockers past each gap steps the width up toward it.
    let past = min(counts / near, vec3f(1.0));
    let width = 1.0 + past.x * (SOFT_WIDTHS.x - 1.0) + past.y * (SOFT_WIDTHS.y - SOFT_WIDTHS.x)
        + past.z * (SOFT_WIDTHS.z - SOFT_WIDTHS.y);
    var sum = 0.0;
    for (var i = 0; i < SOFT_TAPS; i++) {
        sum += tap(p, p.uv + disc(i, SOFT_TAPS) * width * texel);
    }
    return sum / f32(SOFT_TAPS);
}

// How much sun reaches `world` with normal `n` in cascade c.
fn cascadeSun(c: i32, world: vec3f, n: vec3f) -> f32 {
    let texel = view.texels[c];
    let texelDepth = view.texelDepths[c];
    let q = view.cascades[c] * vec4f(world + n * (view.shadow.z * texel), 1.0);
    let origin = vec2f(f32(c & 1), f32(c / 2)) * 0.5;
    let edge = vec2f(view.shadow.x);
    var p: Probe;
    p.uv = origin + (vec2f(q.x, -q.y) * 0.5 + 0.5) * 0.5;
    p.z = q.z - view.shadow.w * texelDepth;
    // The receiver's plane from its normal: depth per texel along the map's axes.
    let toward = max(dot(n, view.light.xyz), 0.1);
    let lean = clamp(vec2f(dot(n, view.lightRight.xyz), -dot(n, view.lightUp.xyz)) / toward, vec2f(-10.0), vec2f(10.0));
    p.slope = lean * texelDepth;
    p.lo = origin + edge;
    p.hi = origin + vec2f(0.5) - edge;
    let kind = view.shadow.y;
    if (kind > 2.5) {
        return soft(p, texelDepth);
    }
    if (kind > 1.5) {
        return tent7(p);
    }
    return tent5(p);
}

// Sun reaching `world`: the cascade its view depth falls in, crossfading
// into the next over the last stretch, fading out before the last ends.
fn sunAt(world: vec3f, n: vec3f) -> f32 {
    if (view.shadow.y < 0.5) {
        return 1.0;
    }
    let depth = dot(world - view.eye.xyz, view.forward.xyz);
    let splits = view.splits;
    if (depth >= splits.w) {
        return 1.0;
    }
    let c = i32(depth >= splits.x) + i32(depth >= splits.y) + i32(depth >= splits.z);
    var sun = cascadeSun(c, world, n);
    if (c < 3) {
        let end = splits[c];
        let begin = select(splits[max(c - 1, 0)], view.shadowFade.w, c == 0);
        let blend = smoothstep(end - (end - begin) * view.shadowFade.y, end, depth);
        if (blend > 0.0) {
            sun = mix(sun, cascadeSun(c + 1, world, n), blend);
        }
    }
    return mix(sun, 1.0, smoothstep(view.shadowFade.x, splits.w, depth));
}

// Each cascade's own tint, for the debug view.
fn cascadeTint(world: vec3f) -> vec3f {
    let depth = dot(world - view.eye.xyz, view.forward.xyz);
    let s = view.splits;
    let c = i32(depth >= s.x) + i32(depth >= s.y) + i32(depth >= s.z) + i32(depth >= s.w);
    var tint = vec3f(1.0);
    tint = select(tint, vec3f(1.0, 0.55, 0.5), c == 0);
    tint = select(tint, vec3f(0.55, 1.0, 0.55), c == 1);
    tint = select(tint, vec3f(0.55, 0.65, 1.0), c == 2);
    tint = select(tint, vec3f(1.0, 0.9, 0.45), c == 3);
    return mix(vec3f(1.0), tint, view.debug.x);
}

// ---- faceted solids: boxes, hulls, meshes, height fields ------------------

struct FlatIn {
    @location(0) position: vec3f,
    @location(1) normal: vec3f,
    // One per corner, or 1 for a component whose opposite edge is inside the face.
    @location(2) bary: vec3f,
    @location(3) rotation: vec4f,
    @location(4) origin: vec3f,
    @location(5) scale: vec3f,
    @location(6) fill: vec4f,
    @location(7) edge: vec4f,
}

struct FlatOut {
    @builtin(position) position: vec4f,
    @location(0) bary: vec3f,
    @location(1) @interpolate(flat) fill: vec4f,
    @location(2) @interpolate(flat) edge: vec4f,
    @location(3) world: vec3f,
    // How much grid the face shows: ground faces that look up.
    @location(4) @interpolate(flat) grid: f32,
    @location(5) @interpolate(flat) normal: vec3f,
}

fn flatWorld(i: FlatIn) -> vec3f {
    return i.origin + qrot(i.rotation, i.position * i.scale);
}

@vertex
fn vs_flat(i: FlatIn) -> FlatOut {
    let p = flatWorld(i);
    var n = normalize(qrot(i.rotation, i.normal / i.scale));
    // Meshes are one sided surfaces seen from either side.
    if (dot(n, view.eye.xyz - p) < 0.0) {
        n = -n;
    }
    let base = ink(i.fill);
    var o: FlatOut;
    o.position = project(p);
    o.bary = i.bary;
    o.fill = vec4f(base.rgb, 1.0);
    o.edge = ink(i.edge);
    o.world = p;
    // Ground arrives with a clear fill alpha.
    o.grid = select(0.0, smoothstep(0.5, 0.8, n.y), base.a < 0.5 && view.grid.w > 0.0);
    o.normal = n;
    return o;
}

@vertex
fn vs_flat_cast(i: FlatIn) -> @builtin(position) vec4f {
    return caster.matrix * vec4f(flatWorld(i), 1.0);
}

@fragment
fn fs_flat(i: FlatOut) -> @location(0) vec4f {
    let dx = dpdxFine(i.bary);
    let dy = dpdyFine(i.bary);
    let g = max(sqrt(dx * dx + dy * dy), vec3f(1e-6));
    let d = i.bary / g;
    let px = view.right.w;
    // Faces a few pixels across take the share of their area the edges cover instead, which does not shimmer.
    let across = 1.0 / max(g.x, max(g.y, g.z));
    let inner = max(1.0 - 3.0 * EDGE * px / across, 0.0);
    let sharp = clamp(across / (3.0 * px) - 1.0, 0.0, 1.0);
    let a = mix(1.0 - inner * inner, cover(min(d.x, min(d.y, d.z)), EDGE * px), sharp) * i.edge.a;
    let ground = mix(i.fill.rgb, gridOver(i.world.xz, i.fill.rgb), i.grid);
    let fill = lit(ground, i.normal, sunAt(i.world, i.normal)) * cascadeTint(i.world);
    return vec4f(mix(fill, i.edge.rgb, a), 1.0);
}

// Pixel distance to the nearest line of a grid with the given spacing.
fn gridDistance(p: vec2f, spacing: f32, perPixel: vec2f) -> f32 {
    let f = abs(fract(p / spacing - 0.5) - 0.5) * spacing / perPixel;
    return min(f.x, f.y);
}

// Grid lines every meter and every five over the ground's color, fading out from the grid's center.
fn gridOver(p: vec2f, under: vec3f) -> vec3f {
    let dx = dpdxFine(p);
    let dy = dpdyFine(p);
    let perPixel = max(sqrt(dx * dx + dy * dy), vec2f(1e-6));
    let px = view.right.w;
    // Spacing on screen in points; lines closer than a few points fade out before they shimmer.
    let pitch = 1.0 / (max(perPixel.x, perPixel.y) * px);
    let minor = cover(gridDistance(p, 1.0, perPixel), 0.5 * px) * clamp(pitch / 3.0 - 1.0, 0.0, 1.0);
    let major = cover(gridDistance(p, 5.0, perPixel), 0.5 * px) * clamp(pitch * 5.0 / 3.0 - 1.0, 0.0, 1.0);
    let off = abs(p - view.grid.xy) / max(view.grid.w, 1e-3);
    let reach = max(off.x, off.y);
    let fade = (1.0 - 0.5 * reach) * (1.0 - smoothstep(0.9, 1.0, reach));
    let c = mix(under, view.gridMinor.rgb, minor * fade);
    return mix(c, view.gridMajor.rgb, major * fade);
}

// ---- spheres and capsules -------------------------------------------------

struct RoundIn {
    // Unit direction from the center of the cap it belongs to, y along the axis.
    @location(0) direction: vec3f,
    // 0 for the first cap, 1 for the second.
    @location(1) end: f32,
    @location(2) first: vec4f,   // first center, radius
    @location(3) second: vec4f,  // second center, the same as the first for a sphere, 1 to draw rings
    @location(4) rotation: vec4f,
    @location(5) fill: vec4f,
    @location(6) edge: vec4f,
}

struct RoundOut {
    @builtin(position) position: vec4f,
    @location(0) normal: vec3f,
    @location(1) world: vec3f,
    // Signed distances to the two rings, in world units.
    @location(2) ring: vec2f,
    @location(3) @interpolate(flat) fill: vec4f,
    @location(4) @interpolate(flat) edge: vec4f,
    @location(5) @interpolate(flat) rings: f32,
}

// A round vertex on its body: where it is, its normal, and the frame its rings are drawn in.
struct RoundPoint {
    p: vec3f,
    n: vec3f,
    ring: vec2f,
}

fn roundPoint(i: RoundIn) -> RoundPoint {
    let c1 = i.first.xyz;
    let r = i.first.w;
    let span = i.second.xyz - c1;
    let reach = sqrt(dot(span, span));
    let capsule = reach > 1e-5;
    let axis = select(qrot(i.rotation, vec3f(0.0, 1.0, 0.0)), span / max(reach, 1e-5), capsule);
    var side = qrot(i.rotation, vec3f(1.0, 0.0, 0.0));
    side = select(side, qrot(i.rotation, vec3f(0.0, 0.0, 1.0)), abs(dot(side, axis)) > 0.9);
    let u = normalize(side - axis * dot(side, axis));
    let w = cross(u, axis);
    let n = i.direction.x * u + i.direction.y * axis + i.direction.z * w;
    let p = c1 + span * i.end + r * n;
    let s = dot(p - c1, axis);
    var o: RoundPoint;
    o.p = p;
    o.n = n;
    // A sphere shows two great circles for its spin, a capsule the seams of its caps.
    o.ring = select(vec2f(s, dot(p - c1, u)), vec2f(s, s - reach), capsule);
    return o;
}

@vertex
fn vs_round(i: RoundIn) -> RoundOut {
    let at = roundPoint(i);
    var o: RoundOut;
    o.position = project(at.p);
    o.normal = at.n;
    o.world = at.p;
    o.ring = at.ring;
    o.fill = ink(i.fill);
    o.edge = ink(i.edge);
    o.rings = i.second.w;
    return o;
}

@vertex
fn vs_round_cast(i: RoundIn) -> @builtin(position) vec4f {
    return caster.matrix * vec4f(roundPoint(i).p, 1.0);
}

@fragment
fn fs_round(i: RoundOut) -> @location(0) vec4f {
    let n = normalize(i.normal);
    let facing = dot(n, normalize(view.eye.xyz - i.world));
    let rdx = dpdxFine(i.ring);
    let rdy = dpdyFine(i.ring);
    let ring = abs(i.ring) / max(sqrt(rdx * rdx + rdy * rdy), vec2f(1e-6));
    let fdx = dpdxFine(facing);
    let fdy = dpdyFine(facing);
    let rim = facing / max(sqrt(fdx * fdx + fdy * fdy), 1e-6);
    let px = view.right.w;
    var c = lit(i.fill.rgb, n, sunAt(i.world, n)) * cascadeTint(i.world);
    let rings = cover(min(ring.x, ring.y), LINE * px) * 0.85 * i.rings;
    c = mix(c, i.fill.rgb * 0.45, rings);
    c = mix(c, i.edge.rgb, cover(rim, LINE * px) * i.edge.a);
    return vec4f(c, 1.0);
}

// ---- blob shadows, when the shadow maps are off ---------------------------

// A quad corner from a vertex index, two triangles; branchless, since some drivers miscompile indexed arrays here.
fn quad(v: u32) -> vec2f {
    let x = select(-1.0, 1.0, v == 1u || v == 2u || v == 4u);
    let y = select(-1.0, 1.0, v == 2u || v == 4u || v == 5u);
    return vec2f(x, y);
}

struct ShadowIn {
    @builtin(vertex_index) v: u32,
    @location(0) disc: vec4f,   // center on the ground, spread
    @location(1) shade: vec4f,  // opacity, lift toward the eye
}

struct ShadowOut {
    @builtin(position) position: vec4f,
    @location(0) local: vec2f,
    @location(1) @interpolate(flat) alpha: f32,
}

@vertex
fn vs_shadow(i: ShadowIn) -> ShadowOut {
    let q = quad(i.v) * 1.5;
    var p = i.disc.xyz + vec3f(q.x * i.disc.w, 0.006, q.y * i.disc.w);
    // Over uneven ground the flat disc dips under slopes; pull it toward the eye.
    p += normalize(view.eye.xyz - p) * i.shade.y;
    var o: ShadowOut;
    o.position = project(p);
    o.local = q;
    o.alpha = i.shade.x;
    return o;
}

@fragment
fn fs_shadow(i: ShadowOut) -> @location(0) vec4f {
    let a = i.alpha * (1.0 - smoothstep(0.55, 1.45, length(i.local)));
    return vec4f(0.0, 0.0, 0.0, a);
}
