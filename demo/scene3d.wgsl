// The physics demo's solids and blob shadows, drawn instanced with a reverse
// Z depth buffer. The projection matches OrbitCamera's, so the vector overlays
// drawn on top line up. Edges are drawn in the fragment shader from each
// vertex's distance to the face outline, so the faceted look needs no line
// pass, and the grid is drawn on the ground's upward faces, so it follows
// uneven ground instead of cutting through it.

struct View {
    eye: vec4f,      // eye position, near plane
    right: vec4f,    // camera right, device pixels per point
    up: vec4f,       // camera up
    forward: vec4f,  // camera forward
    proj: vec4f,     // camera x and y over depth to clip x and y, then the clip offset of the projection center
    light: vec4f,    // key light direction
    grid: vec4f,     // grid center x and z, unused, half extent or 0 for none
    gridMinor: vec4f,
    gridMajor: vec4f,
}

@group(0) @binding(0) var<uniform> view: View;

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

// Lambert from the key light plus a little sky, as the vector painter shades.
fn lit(base: vec3f, n: vec3f) -> vec3f {
    let s = 0.42 + 0.5 * max(dot(n, view.light.xyz), 0.0) + 0.12 * (0.5 + 0.5 * n.y);
    return min(base * s, vec3f(1.0));
}

// Colors arrive as 0xAARRGGBB words read as bytes, so blue first.
fn ink(c: vec4f) -> vec4f {
    return vec4f(c.z, c.y, c.x, c.w);
}

// Coverage of a line of half width w pixels at distance d pixels.
fn cover(d: f32, w: f32) -> f32 {
    return clamp(w + 0.5 - d, 0.0, 1.0);
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
}

@vertex
fn vs_flat(i: FlatIn) -> FlatOut {
    let p = i.origin + qrot(i.rotation, i.position * i.scale);
    var n = normalize(qrot(i.rotation, i.normal / i.scale));
    // Meshes are one sided surfaces seen from either side.
    if (dot(n, view.eye.xyz - p) < 0.0) {
        n = -n;
    }
    let base = ink(i.fill);
    var o: FlatOut;
    o.position = project(p);
    o.bary = i.bary;
    o.fill = vec4f(lit(base.rgb, n), 1.0);
    o.edge = ink(i.edge);
    o.world = p;
    // Ground arrives with a clear fill alpha.
    o.grid = select(0.0, smoothstep(0.5, 0.8, n.y), base.a < 0.5 && view.grid.w > 0.0);
    return o;
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
    let fill = mix(i.fill.rgb, gridOver(i.world.xz, i.fill.rgb), i.grid);
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
    @location(3) second: vec4f,  // second center, the same as the first for a sphere
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
}

@vertex
fn vs_round(i: RoundIn) -> RoundOut {
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
    var o: RoundOut;
    o.position = project(p);
    o.normal = n;
    o.world = p;
    // A sphere shows two great circles for its spin, a capsule the seams of its caps.
    o.ring = select(vec2f(s, dot(p - c1, u)), vec2f(s, s - reach), capsule);
    o.fill = ink(i.fill);
    o.edge = ink(i.edge);
    return o;
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
    var c = lit(i.fill.rgb, n);
    let rings = cover(min(ring.x, ring.y), LINE * px) * 0.85;
    c = mix(c, i.fill.rgb * 0.45, rings);
    c = mix(c, i.edge.rgb, cover(rim, LINE * px) * i.edge.a);
    return vec4f(c, 1.0);
}

// ---- blob shadows ---------------------------------------------------------

// A quad corner from a vertex index; a local array, since some drivers miscompile dynamic indexing of const arrays.
fn quad(v: u32) -> vec2f {
    var q = array<vec2f, 6>(
        vec2f(-1.0, -1.0), vec2f(1.0, -1.0), vec2f(1.0, 1.0),
        vec2f(-1.0, -1.0), vec2f(1.0, 1.0), vec2f(-1.0, 1.0));
    return q[v];
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
