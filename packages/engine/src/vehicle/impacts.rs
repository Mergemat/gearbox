use super::{Vehicle, forces::common::TINY};
use crate::Vec3;

const UPPER_CONTACT_HEIGHT: f32 = 0.3;
const NO_REFERENCE: u8 = 255;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ImpactModel {
    pub references: [Vec3; 9],
    pub wear_thresholds: [f32; 9],
    pub wear_scales: [f32; 9],
    pub wear_delay_ms: u32,
    pub event_threshold: f32,
    pub driven_wheels: [bool; 4],
    pub ceiling_height: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Summary {
    pub material: u16,
    pub point: Vec3,
    pub normal: Vec3,
    pub velocity: Vec3,
}

pub fn two_nearest(points: &[Vec3], point: Vec3) -> ([u8; 2], [f32; 2]) {
    let (mut near, mut far) = (f32::MAX, f32::MAX);
    let mut indices = [NO_REFERENCE; 2];
    for (index, reference) in points.iter().enumerate() {
        let delta = point - *reference;
        let squared = delta.y * delta.y + delta.x * delta.x + delta.z * delta.z;
        if far > squared {
            if near > squared {
                far = near;
                indices[1] = indices[0];
                near = squared;
                indices[0] = index as u8;
            } else {
                far = squared;
                indices[1] = index as u8;
            }
        }
    }
    let sum = near + far;
    (indices, [1.0 - near / sum, 1.0 - far / sum])
}

pub fn add_saturate1(value: &mut f32, addition: f32) {
    let next = addition + *value;
    *value = if 1.0 < next { 1.0 } else { next };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Category {
    Body,
    Wheel,
    FreeWheel,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Accumulator {
    pub count: u32,
    pub point: Vec3,
    pub normal: Vec3,
    pub peak: f32,
    pub material: u8,
    pub gripping: u32,
}

impl Vehicle {
    pub fn accumulate_contact(
        &mut self,
        model: &ImpactModel,
        now: u32,
        contact: Summary,
        wheel: Option<usize>,
        gripping: bool,
        depth: bool,
    ) {
        let velocity = contact.velocity;
        let squared = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
        if TINY > squared {
            return;
        }
        let length = squared.sqrt();
        let normal = contact.normal;
        let strength = (velocity.y * normal.y + velocity.x * normal.x + velocity.z * normal.z).abs();
        let mut flags = super::flags::ImpactFlags(self.impact_flags);
        if depth {
            flags.insert(super::flags::ImpactFlags::DEPTH);
        }
        self.impact_flags = flags.0;
        let category = if wheel.is_none() || !(normal.y.abs() >= 0.25) {
            Category::Body
        } else if !model.driven_wheels[wheel.unwrap()] {
            Category::FreeWheel
        } else {
            Category::Wheel
        };
        let mut accumulator = self.impact(category);
        if strength > accumulator.peak {
            accumulator.peak = strength;
            accumulator.material = contact.material as u8;
            accumulator.gripping = gripping as u32;
        }
        accumulator.point = contact.point + accumulator.point;
        accumulator.normal = contact.normal + accumulator.normal;
        accumulator.count = accumulator.count.wrapping_add(1);
        self.set_impact(category, accumulator);
        if crate::surface::SOFT.contains(&(contact.material as u8)) {
            return;
        }
        if wheel.is_some() && !(contact.point.y > UPPER_CONTACT_HEIGHT) {
            return;
        }
        let (_, weights) = two_nearest(&model.references, contact.point);
        for weight in weights {
            if weight * length > model.event_threshold {
                self.recent_events[0] = now;
            }
        }
    }

    pub fn impact_wear(&mut self, model: &ImpactModel, now: u32, contact: Summary, start: u32) {
        let velocity = contact.velocity;
        let squared = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
        if TINY > squared || !(contact.point.y > UPPER_CONTACT_HEIGHT) {
            return;
        }
        let normal = contact.normal;
        let strength = (velocity.y * normal.y + velocity.x * normal.x + velocity.z * normal.z).abs();
        let (indices, weights) = two_nearest(&model.references, contact.point);
        let mut wear = self.wear;
        for (index, weight) in indices.into_iter().zip(weights) {
            let index = index as usize;
            if strength > model.wear_thresholds[index] {
                let amount = if start == u32::MAX || now < model.wear_delay_ms.wrapping_add(start) {
                    0.0
                } else {
                    strength * model.wear_scales[index]
                };
                add_saturate1(&mut wear[index], amount * weight);
            }
        }
        self.wear = wear;
    }

    pub fn impact(&self, category: Category) -> Accumulator {
        match category {
            Category::Body => Accumulator {
                count: self.body_impact_count,
                point: Vec3::new(
                    self.body_impact_point_x,
                    self.body_impact_point_y,
                    self.body_impact_point_z,
                ),
                normal: Vec3::new(
                    self.body_impact_normal_x,
                    self.body_impact_normal_y,
                    self.body_impact_normal_z,
                ),
                peak: self.body_impact_peak,
                material: self.body_impact_material,
                gripping: 0,
            },
            Category::Wheel => Accumulator {
                count: self.wheel_impact_count,
                point: Vec3::new(
                    self.wheel_impact_point_x,
                    self.wheel_impact_point_y,
                    self.wheel_impact_point_z,
                ),
                normal: Vec3::new(
                    self.wheel_impact_normal_x,
                    self.wheel_impact_normal_y,
                    self.wheel_impact_normal_z,
                ),
                peak: self.wheel_impact_peak,
                material: self.wheel_impact_material,
                gripping: self.wheel_impact_gripping,
            },
            Category::FreeWheel => Accumulator {
                count: self.free_wheel_impact_count,
                point: Vec3::new(
                    self.free_wheel_impact_point_x,
                    self.free_wheel_impact_point_y,
                    self.free_wheel_impact_point_z,
                ),
                normal: Vec3::new(
                    self.free_wheel_impact_normal_x,
                    self.free_wheel_impact_normal_y,
                    self.free_wheel_impact_normal_z,
                ),
                peak: self.free_wheel_impact_peak,
                material: self.free_wheel_impact_material,
                gripping: self.free_wheel_impact_gripping,
            },
        }
    }

    pub fn set_impact(&mut self, category: Category, value: Accumulator) {
        match category {
            Category::Body => {
                self.body_impact_count = value.count;
                (self.body_impact_point_x, self.body_impact_point_y, self.body_impact_point_z) =
                    (value.point.x, value.point.y, value.point.z);
                (self.body_impact_normal_x, self.body_impact_normal_y, self.body_impact_normal_z) =
                    (value.normal.x, value.normal.y, value.normal.z);
                self.body_impact_peak = value.peak;
                self.body_impact_material = value.material;
            }
            Category::Wheel => {
                self.wheel_impact_count = value.count;
                (self.wheel_impact_point_x, self.wheel_impact_point_y, self.wheel_impact_point_z) =
                    (value.point.x, value.point.y, value.point.z);
                (self.wheel_impact_normal_x, self.wheel_impact_normal_y, self.wheel_impact_normal_z) =
                    (value.normal.x, value.normal.y, value.normal.z);
                self.wheel_impact_peak = value.peak;
                self.wheel_impact_material = value.material;
                self.wheel_impact_gripping = value.gripping;
            }
            Category::FreeWheel => {
                self.free_wheel_impact_count = value.count;
                (
                    self.free_wheel_impact_point_x,
                    self.free_wheel_impact_point_y,
                    self.free_wheel_impact_point_z,
                ) = (value.point.x, value.point.y, value.point.z);
                (
                    self.free_wheel_impact_normal_x,
                    self.free_wheel_impact_normal_y,
                    self.free_wheel_impact_normal_z,
                ) = (value.normal.x, value.normal.y, value.normal.z);
                self.free_wheel_impact_peak = value.peak;
                self.free_wheel_impact_material = value.material;
                self.free_wheel_impact_gripping = value.gripping;
            }
        }
    }
}
