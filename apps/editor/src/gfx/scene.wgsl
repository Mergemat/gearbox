struct Globals {
    view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    light: vec4<f32>,
    fog: vec4<f32>,
    sky: vec4<f32>,
    origin: vec4<f32>,
};

struct Inst {
    m: mat4x4<f32>,
    tint: vec4<f32>,
};

@group(0) @binding(0) var<uniform> g: Globals;
@group(0) @binding(1) var<storage, read> store: array<Inst>;
@group(0) @binding(2) var<storage, read> ids: array<u32>;
@group(1) @binding(0) var tex: texture_2d<f32>;
@group(1) @binding(1) var samp: sampler;

struct VIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec4<f32>,
    @location(2) color: vec4<f32>,
    @location(3) uv: vec2<f32>,
};

struct VOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) normal: vec4<f32>,
    @location(2) world: vec3<f32>,
    @location(3) uv: vec2<f32>,
};

fn to_linear(c: vec3<f32>) -> vec3<f32> {
    return pow(c, vec3<f32>(2.2));
}

@vertex
fn vs_mesh(v: VIn, @builtin(instance_index) ii: u32) -> VOut {
    let inst = store[ids[ii]];
    let m = inst.m;
    let a = m * vec4<f32>(v.pos, 1.0);
    let w = vec4<f32>(a.xyz - g.origin.xyz, 1.0);
    var o: VOut;
    o.clip = g.view_proj * w;
    o.color = vec4<f32>(to_linear(v.color.rgb * inst.tint.rgb), v.color.a * inst.tint.a);
    o.normal = vec4<f32>((m * vec4<f32>(v.normal.xyz, 0.0)).xyz, v.normal.w);
    o.world = w.xyz;
    o.uv = v.uv;
    return o;
}

fn shade(i: VOut, base: vec3<f32>) -> vec3<f32> {
    if (i.normal.w < -0.5) {
        return base;
    }
    var n: vec3<f32>;
    if (i.normal.w > 0.5) {
        n = normalize(i.normal.xyz);
    } else {
        n = normalize(cross(dpdx(i.world), dpdy(i.world)));
    }
    let to_eye = g.eye.xyz - i.world;
    if (dot(n, to_eye) < 0.0) {
        n = -n;
    }
    let key = max(dot(n, g.light.xyz), 0.0);
    let sky = 0.5 + 0.5 * n.y;
    return base * (0.22 + 0.16 * sky + 0.72 * key) * g.light.w;
}

fn fog(i: VOut, c: vec3<f32>) -> vec3<f32> {
    let d = length(i.world - g.eye.xyz);
    let f = clamp((d - g.fog.x) / (g.fog.y - g.fog.x), 0.0, g.fog.z);
    return mix(c, g.sky.rgb, f);
}

@fragment
fn fs_mesh(i: VOut) -> @location(0) vec4<f32> {
    let t = textureSample(tex, samp, i.uv);
    let base = i.color.rgb * t.rgb;
    return vec4<f32>(fog(i, shade(i, base)), i.color.a * t.a);
}

@fragment
fn fs_cutout(i: VOut) -> @location(0) vec4<f32> {
    let t = textureSample(tex, samp, i.uv);
    if (t.a < 0.5) {
        discard;
    }
    let base = i.color.rgb * t.rgb;
    return vec4<f32>(fog(i, shade(i, base)), 1.0);
}

fn light_fade(i: VOut) -> f32 {
    let d = length(i.world - g.eye.xyz);
    return (1.0 - clamp((d - g.fog.x) / (g.fog.y - g.fog.x), 0.0, 1.0)) * smoothstep(1.5, 8.0, d);
}

@fragment
fn fs_holo(i: VOut) -> @location(0) vec4<f32> {
    let t = textureSample(tex, samp, i.uv).rgb;
    let rim = smoothstep(0.45, 0.95, max(t.r, max(t.g, t.b)));
    var n: vec3<f32>;
    if (i.normal.w > 0.5) {
        n = normalize(i.normal.xyz);
    } else {
        n = normalize(cross(dpdx(i.world), dpdy(i.world)));
    }
    let v = normalize(g.eye.xyz - i.world);
    let fresnel = pow(1.0 - abs(dot(n, v)), 3.0);
    let y = i.world.y + g.origin.y;
    let scan = 0.78 + 0.22 * sin(y * 7.0 - g.fog.w * 1.6);
    let a = i.color.a * (0.07 + 0.55 * rim + 0.30 * fresnel) * scan * light_fade(i);
    return vec4<f32>(i.color.rgb, a);
}

@fragment
fn fs_glow(i: VOut) -> @location(0) vec4<f32> {
    let t = textureSample(tex, samp, i.uv).rgb;
    return vec4<f32>(i.color.rgb * t, i.color.a * light_fade(i));
}

struct LIn {
    @location(0) pos: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct LOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_line(v: LIn) -> LOut {
    var o: LOut;
    o.clip = g.view_proj * vec4<f32>(v.pos - g.origin.xyz, 1.0);
    o.color = vec4<f32>(to_linear(v.color.rgb), v.color.a);
    return o;
}

@fragment
fn fs_line(i: LOut) -> @location(0) vec4<f32> {
    return i.color;
}
