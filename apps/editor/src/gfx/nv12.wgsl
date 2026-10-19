@group(0) @binding(0) var src: texture_2d<f32>;

@vertex
fn vs(@builtin(vertex_index) i: u32) -> @builtin(position) vec4<f32> {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    return vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
}

const LUMA: vec3<f32> = vec3<f32>(0.299, 0.587, 0.114);

@fragment
fn fs_y(@builtin(position) p: vec4<f32>) -> @location(0) vec4<f32> {
    let c = textureLoad(src, vec2<i32>(p.xy), 0).rgb;
    return vec4<f32>(dot(c, LUMA), 0.0, 0.0, 1.0);
}

@fragment
fn fs_cbcr(@builtin(position) p: vec4<f32>) -> @location(0) vec4<f32> {
    let dim = vec2<i32>(textureDimensions(src)) - 1;
    let b = vec2<i32>(p.xy) * 2;
    let c = (textureLoad(src, min(b, dim), 0).rgb
        + textureLoad(src, min(b + vec2<i32>(1, 0), dim), 0).rgb
        + textureLoad(src, min(b + vec2<i32>(0, 1), dim), 0).rgb
        + textureLoad(src, min(b + vec2<i32>(1, 1), dim), 0).rgb) * 0.25;
    let y = dot(c, LUMA);
    return vec4<f32>((c.b - y) / 1.772 + 0.5, (c.r - y) / 1.402 + 0.5, 0.0, 1.0);
}
