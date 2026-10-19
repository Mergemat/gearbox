use crate::Iso;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Grid {
    pub cell: [f32; 2],
    pub inverse_cell: [f32; 2],
    pub origin: [f32; 2],
    pub size: [u32; 2],
    pub default_value: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Entry {
    pub value: u32,
    pub next: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Layer {
    pub bottom: u32,
    pub top: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Column {
    pub surface: f32,
    pub bottom: f32,
    pub material: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VoxelGrid {
    pub material: u32,
    pub layers: Vec<Layer>,
    pub grid: Grid,
    pub heads: Vec<u32>,
    pub entries: Vec<Entry>,
    pub column_count: u32,
    pub height: f32,
    pub max_y: u32,
    pub placement: Iso,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ModelBox {
    pub model: u32,
    pub placement: Iso,
    pub inverse: Iso,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Volume {
    pub grid: Grid,
    pub default_column: Column,
    pub columns: Vec<Column>,
    pub box_grid: Grid,
    pub boxes: Vec<ModelBox>,
    pub heads: Vec<u32>,
    pub entries: Vec<Entry>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Water {
    pub queries_disabled: bool,
    pub models: Vec<VoxelGrid>,
    pub volumes: Vec<Volume>,
}
