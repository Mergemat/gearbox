use crate::{Vec3, numeric, math::{cvtt, maxss, minss}};
use crate::world_water::{Column, Entry, Grid, Layer, ModelBox, VoxelGrid};
use super::WaterManager;

pub const LAYER_MARGIN: f32 = 0.00001;

impl Grid {
    pub fn cell_at(&self, point: [f32; 2]) -> [i32; 2] {
        std::array::from_fn(|axis| {
            cvtt(numeric::floorf((point[axis] - self.origin[axis]) * self.inverse_cell[axis]))
        })
    }

    pub fn contains_cell(&self, cell: [i32; 2]) -> bool {
        cell[0] >= 0 && cell[1] >= 0
            && cell[0] < self.size[0] as i32 && cell[1] < self.size[1] as i32
    }

    fn index(&self, cell: [i32; 2]) -> usize {
        self.size[0].wrapping_mul(cell[1] as u32).wrapping_add(cell[0] as u32) as usize
    }

    pub fn column_at<'a>(&self, cell: [i32; 2], columns: &'a [Column], default: &'a Column)
        -> &'a Column
    {
        if self.contains_cell(cell) { &columns[self.index(cell)] } else { default }
    }

    pub fn head_at(&self, cell: [i32; 2], heads: &[u32]) -> u32 {
        if self.contains_cell(cell) { heads[self.index(cell)] } else { self.default_value }
    }

    pub(crate) fn entries_at<'a>(&self, point: [f32; 2], heads: &[u32], entries: &'a [Entry])
        -> impl Iterator<Item = u32> + 'a
    {
        let mut next = self.head_at(self.cell_at(point), heads);
        std::iter::from_fn(move || {
            let entry = entries.get(next as usize)?;
            next = entry.next;
            Some(entry.value)
        })
    }
}

impl Layer {
    pub fn span(&self, height: f32) -> [f32; 2] {
        [self.bottom as f32 * height - LAYER_MARGIN,
            self.top.wrapping_add(1) as f32 * height + LAYER_MARGIN]
    }
}

impl ModelBox {
    pub fn overlaps(&self, model: &VoxelGrid, point: Vec3, top: f32) -> bool {
        let [row_x, row_y, row_z] = self.inverse.rot.rows;
        let origin = self.inverse.pos;
        let local_y = point.x * row_y.x + point.y * row_y.y + point.z * row_y.z + origin.y;
        let local_x = point.y * row_x.y + point.x * row_x.x + point.z * row_x.z + origin.x;
        let local_z = point.x * row_z.x + point.y * row_z.y + point.z * row_z.z + origin.z;
        let local_top = (top - point.y) + local_y;
        model.grid.entries_at([local_x, local_z], &model.heads, &model.entries).any(|index| {
            let [bottom, upper] = model.layers[index as usize].span(model.height);
            !(local_top < bottom) && upper >= local_y
        })
    }

    pub fn surface(&self, model: &VoxelGrid) -> f32 {
        model.max_y.wrapping_add(1) as f32 * model.height + self.placement.pos.y
    }
}

impl WaterManager<'_> {
    pub fn query(&self, point: Vec3, height: f32, mut depth: Option<&mut f32>,
        mut material: Option<&mut u32>) -> bool
    {
        if self.water.queries_disabled || self.water.volumes.is_empty() {
            return false;
        }
        let horizontal = [point.x, point.z];
        let top = point.y + height;
        let mut hit = false;
        let mut best = f32::MIN;
        for volume in &self.water.volumes {
            let column = volume.grid.column_at(
                volume.grid.cell_at(horizontal), &volume.columns, &volume.default_column,
            );
            if column.surface > column.bottom && top > column.bottom && column.surface > point.y {
                if let Some(out) = depth { *out = column.surface; }
                if let Some(out) = material { *out = column.material; }
                return true;
            }
            for index in volume.box_grid.entries_at(horizontal, &volume.heads, &volume.entries) {
                let model_box = &volume.boxes[index as usize];
                let model = &self.water.models[model_box.model as usize];
                if model_box.overlaps(model, point, top) {
                    hit = true;
                    if let Some(out) = material.as_deref_mut() { *out = model.material; }
                    if depth.is_none() { return true; }
                    best = maxss(model_box.surface(model), best);
                }
            }
            if let Some(out) = depth.as_deref_mut() {
                *out = if hit { minss(top, best) } else { 0.0 };
            }
        }
        hit
    }
}
