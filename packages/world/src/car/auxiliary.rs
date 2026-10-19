use anyhow::{Context, Result, ensure};
use gearbox_engine::{
    Curve,
    vehicle::{VisGeom, VisModel, VisShared, model::SurfaceMaterial},
};
use gearbox_packs::car_model::Aux;

#[derive(Clone, Debug, PartialEq)]
pub struct Auxiliary {
    pub visual: VisModel,
    pub shared: VisShared,
    pub geometry: VisGeom,
    pub groups: Vec<Vec<u32>>,
    pub materials: Vec<SurfaceMaterial>,
    pub material_slots: [u32; 80],
}

fn curve(source: &Curve) -> Result<Curve> {
    ensure!(source.flags & Curve::SPLINE == 0, "unsupported spline car auxiliary curve");
    Ok(source.clone())
}

pub(super) fn build(source: &Aux) -> Result<Auxiliary> {
    ensure!(source.curves.len() == 4, "unsupported car visual curve count");
    let visual = VisModel {
        wheel_impact_max: source.wheel_impact_max,
        wheel_impact_min: source.wheel_impact_min,
        body_impact_max: source.body_impact_max,
        body_impact_min: source.body_impact_min,
        ..VisModel::default()
    };
    let shared = shared(source)?;
    let geometry = VisGeom { impact_event_threshold: 20.0 };
    let (materials, material_slots) = materials(source)?;
    Ok(Auxiliary { visual, shared, geometry, groups: source.groups.clone(), materials, material_slots })
}

fn shared(source: &Aux) -> Result<VisShared> {
    for ids in &source.groups {
        ensure!(ids.iter().all(|&id| id < 80), "unsupported material group surface id");
    }
    Ok(VisShared {
        spring_drive_limit: 10.0,
        spring_drive_scale: 15.0,
        spring_value_limit: 0.3,
        spring_stiffness: 150.0,
        spring_damping: 15.0,
        impact_delays: source.times,
        wheel_response: curve(&source.curves[0])?,
        steering_response: curve(&source.curves[2])?,
    })
}

fn materials(source: &Aux) -> Result<(Vec<SurfaceMaterial>, [u32; 80])> {
    let mut materials = Vec::new();
    let mut slots = [None; 80];
    for material in &source.materials {
        ensure!((material.index as usize) < slots.len(), "unsupported car surface id {}", material.index);
        let value = SurfaceMaterial {
            thrust_by_speed: curve(&material.thrust_by_speed)?,
            steering_angle_by_speed: curve(&material.steering_angle_by_speed)?,
            steering_by_speed: curve(&material.steering_by_speed)?,
            grip_by_immersion: curve(&material.grip_by_immersion)?,
            thrust_by_alignment: curve(&material.thrust_by_alignment)?,
            ..material.clone()
        };
        slots[material.index as usize] = Some(materials.len() as u32);
        materials.push(value);
    }
    for (destination, parent) in [(56, 7), (58, 7), (57, 26), (59, 26), (60, 29), (61, 29)] {
        if slots[destination].is_none() {
            slots[destination] = slots[parent];
        }
    }
    let default = slots[0].context("car materials missing default surface zero")?;
    let mut ordered = Vec::new();
    let mut remapped = std::collections::HashMap::new();
    let slots = slots.map(|slot| {
        let slot = slot.unwrap_or(default);
        *remapped.entry(slot).or_insert_with(|| {
            let index = ordered.len() as u32;
            ordered.push(materials[slot as usize].clone());
            index
        })
    });
    Ok((ordered, slots))
}
