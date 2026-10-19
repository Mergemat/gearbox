use std::collections::VecDeque;
use std::time::Instant;

use crate::gfx::renderer::FrameTimes;

const KEEP: usize = 1200;

#[derive(Default)]
pub struct Series(VecDeque<f32>);

impl Series {
    pub fn push(&mut self, v: f32) {
        if self.0.len() == KEEP {
            self.0.pop_front();
        }
        self.0.push_back(v);
    }
    pub fn len(&self) -> usize {
        self.0.len()
    }
    pub fn pct(&self, p: f32) -> f32 {
        if self.0.is_empty() {
            return 0.0;
        }
        let mut v: Vec<f32> = self.0.iter().copied().collect();
        v.sort_by(f32::total_cmp);
        v[((v.len() - 1) as f32 * p).round() as usize]
    }
}

#[derive(Default)]
pub struct Stats {
    last_paint: Option<Instant>,
    pub interval: Series,
    pub encode: Series,
    pub wait: Series,
    pub gpu_scene: Series,
    pub gpu_composite: Series,
    pub readback: Series,
    pub paint: Series,
    pub input_latency: Series,
    pub ui_frame: Series,
    pub triangles: u64,
    pub pending_input: Option<Instant>,
}

impl Stats {
    pub fn painted(&mut self, start: Instant, t: &FrameTimes, animating: bool) {
        let now = Instant::now();
        if animating && let Some(p) = self.last_paint {
            self.interval.push((now - p).as_secs_f32() * 1e3);
        }
        self.last_paint = Some(now);
        self.paint.push((now - start).as_secs_f32() * 1e3);
        self.encode.push(t.encode_us / 1e3);
        self.wait.push(t.wait_us / 1e3);
        if let Some(g) = t.gpu_scene_us {
            self.gpu_scene.push(g / 1e3);
        }
        if let Some(g) = t.gpu_composite_us {
            self.gpu_composite.push(g / 1e3);
        }
        if t.readback_us > 0.0 {
            self.readback.push(t.readback_us / 1e3);
        }
        self.triangles = t.triangles;
        if let Some(i) = self.pending_input.take() {
            self.input_latency.push((now - i).as_secs_f32() * 1e3);
        }
    }

    pub fn input(&mut self) {
        self.pending_input.get_or_insert_with(Instant::now);
    }

    pub fn idle(&mut self) {
        self.last_paint = None;
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }

}
