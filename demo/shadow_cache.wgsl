// Copies one cascade's cached static caster depths into its tile of the
// shadow atlas, before the moving casters draw over them. The API has no
// texture copies, and GLSL cannot read a depth texture raw, so the cache is
// a float color texture written back as fragment depth.

@group(0) @binding(0) var cached: texture_2d<f32>;

// One triangle over the whole viewport, the cascade's tile.
@vertex
fn vs_blit(@builtin(vertex_index) v: u32) -> @builtin(position) vec4f {
    let x = f32(i32(v & 1u) * 4 - 1);
    let y = f32(i32(v >> 1u) * 4 - 1);
    return vec4f(x, y, 0.5, 1.0);
}

@fragment
fn fs_blit(@builtin(position) p: vec4f) -> @builtin(frag_depth) f32 {
    let size = vec2i(textureDimensions(cached));
    return textureLoad(cached, vec2i(p.xy) % size, 0).r;
}
