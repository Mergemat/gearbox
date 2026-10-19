use super::{Auxiliary, geometry};
use anyhow::{Result, ensure};
use gearbox_engine::dyna::constants::NULL_INDEX;
use gearbox_engine::{
    Aabb, BoxId, Mat3, ShapeId, State, Vec3, World,
    dyna::DynaModel,
    math::{maxss, minss},
    vehicle::{
        Tuning,
        model::{ForceGeometry, WheelDefinition},
        spawn,
    },
};
use gearbox_packs::car_model::CarModel;

const MIN_MASS: f32 = 10.0;

#[derive(Clone, Copy)]
enum DriveMode {
    Archived,
    All,
    Steering,
    Fixed,
}
impl DriveMode {
    fn from_code(code: u32) -> Result<Self> {
        match code {
            0 => Ok(Self::Archived),
            1 => Ok(Self::All),
            2 => Ok(Self::Steering),
            3 => Ok(Self::Fixed),
            _ => anyhow::bail!("unsupported car drive selector {code}"),
        }
    }
    fn driven(self, wheel: &gearbox_packs::car_model::Wheel) -> bool {
        match self {
            Self::Archived => wheel.driven != 0,
            Self::All => true,
            Self::Steering => wheel.steering != 0,
            Self::Fixed => wheel.steering == 0,
        }
    }
}

pub(super) struct Prepared {
    pub impacts: gearbox_engine::vehicle::impacts::ImpactModel,
    pub body: DynaModel,
    pub geometry: ForceGeometry,
    pub spawn: spawn::Assets,
    pub source_collision: ShapeId,
    pub template: ShapeId,
    pub water_shape: ShapeId,
}

pub(super) fn prepare(
    source: &CarModel,
    shape: &gearbox_engine::vehicle::CarShape,
    tuning: &Tuning,
    auxiliary: &Auxiliary,
    world: &mut World,
    state: &mut State,
) -> Result<Prepared> {
    ensure!(source.wheels.len() == 4, "unsupported non-four-wheel car model");
    ensure!(shape.manual_wheel_positions != 0, "unsupported automatic wheel placement");
    let mut wheels = wheels(source, shape, tuning)?;
    let (source_collision, template) = geometry::prepare(source, tuning, &mut wheels, world, state)?;
    let (center, half, average) = wheel_bounds(&wheels, shape.wheel_height);
    let body = body(tuning, center, half, average);
    let local_inverse_inertia = box_inertia(tuning.inertia_mass, tuning.inertia_half);
    let archived_bounds = shape.bounds;
    let water_box = if archived_bounds.half.x < 0.0 { Aabb::new(center, half) } else { archived_bounds };
    let hull_box = geometry::bounds(template, world, state);
    let water_shape = water_shape(shape.water_bounds, world);
    let impacts = impact_model(water_box, hull_box, &wheels, auxiliary.geometry.impact_event_threshold);
    let geometry = ForceGeometry {
        mass: tuning.mass,
        center_of_mass: body.center(),
        reference_center: body.center(),
        water_box,
        wheels,
        materials: auxiliary
            .material_slots
            .iter()
            .map(|&slot| auxiliary.materials[slot as usize].clone())
            .collect(),
        material14_alias: world.statics.material14_alias,
        thrust_scale: 1.0,
        steering_scale: 1.0,
        gravity_scale: 1.0,
        grip_scale: 1.0,
        local_inverse_inertia,
        axle_span: shape.front_axle - shape.rear_axle,
        effect_contacts: source.wheels.iter().map(|wheel| wheel.steering != 0).collect(),
    };
    let spawn = spawn_assets(tuning, &geometry);
    Ok(Prepared { impacts, body, geometry, spawn, source_collision, template, water_shape })
}

fn wheels(
    source: &CarModel,
    shape: &gearbox_engine::vehicle::CarShape,
    tuning: &Tuning,
) -> Result<Vec<WheelDefinition>> {
    let lateral = shape.lateral_offset;
    let height = shape.wheel_height;
    let front = shape.front_axle;
    let rear = shape.rear_axle;
    let front_width = shape.front_half_track;
    let rear_width = shape.rear_half_track;
    let positions = [
        (lateral + front_width, front),
        (lateral - front_width, front),
        (lateral - rear_width, rear),
        (lateral + rear_width, rear),
    ];
    let drive_mode = DriveMode::from_code(tuning.drive_selector)?;
    positions
        .into_iter()
        .enumerate()
        .map(|(index, (x, z))| {
            let radius = shape.wheel_radius[usize::from(index >= 2)];
            let width = shape.wheel_width[usize::from(index >= 2)];
            let definition = &source.wheels[index];
            let driven = drive_mode.driven(definition);
            Ok(WheelDefinition {
                steers: definition.steering != 0,
                radius,
                anchor: Vec3::new(x, height + radius, z),
                contact_flag: driven,
                contact_radius: width,
                transform_index: None,
            })
        })
        .collect()
}

pub fn box_inertia(mass: f32, half: Vec3) -> Mat3 {
    let doubled = Vec3::new(half.x + half.x, half.y + half.y, half.z + half.z);
    let scale = (1.0 / mass) * 12.0;
    let squared = Vec3::new(doubled.x * doubled.x, doubled.y * doubled.y, doubled.z * doubled.z);
    Mat3::from_array([
        scale / (squared.y + squared.z),
        0.0,
        0.0,
        0.0,
        scale / (squared.x + squared.z),
        0.0,
        0.0,
        0.0,
        scale / (squared.x + squared.y),
    ])
}

fn wheel_bounds(wheels: &[WheelDefinition], height: f32) -> (Vec3, Vec3, Vec3) {
    let mut low = [f32::MAX; 3];
    let mut high = [-f32::MAX; 3];
    let mut sum = [0.0; 3];
    for wheel in wheels {
        let position = wheel.anchor.to_array();
        let original = [position[0], height, position[2]];
        for axis in 0..3 {
            low[axis] = minss(position[axis], low[axis]);
            high[axis] = maxss(position[axis], high[axis]);
            sum[axis] += original[axis];
        }
    }
    let inverse = 1.0 / wheels.len() as f32;
    (
        Vec3::from_array(std::array::from_fn(|axis| (low[axis] + high[axis]) * 0.5)),
        Vec3::from_array(std::array::from_fn(|axis| (high[axis] - low[axis]) * 0.5)),
        Vec3::from_array(sum.map(|value| value * inverse)),
    )
}

fn body(tuning: &Tuning, center: Vec3, half: Vec3, average: Vec3) -> DynaModel {
    let mut mass = tuning.mass;
    let inertia_mass =
        if MIN_MASS > tuning.inertia_mass { tuning.fallback_mass } else { tuning.inertia_mass };
    if MIN_MASS > mass || MIN_MASS > tuning.inertia_mass {
        mass = tuning.fallback_mass;
    }
    let inertia = box_inertia(inertia_mass, tuning.inertia_half);
    DynaModel {
        mass,
        inverse_inertia_matrix: inertia,
        center_x: center.x,
        center_y: average.y + tuning.center_height_offset,
        center_z: half.z * tuning.longitudinal_balance + center.z,
        max_angular_velocity: tuning.max_angular_velocity,
        substep_distance: tuning.substep_distance,
        externally_simulated: 1,
        kind: 0,
        ..Default::default()
    }
}

fn water_shape(bounds: Aabb, world: &mut World) -> ShapeId {
    let result = ShapeId::Box(BoxId(world.box_shape.len() as u32));
    world.box_shape.push(gearbox_engine::collision::BoxShape { material: 0, bounds, ..Default::default() });
    result
}

fn spawn_assets(tuning: &Tuning, geometry: &ForceGeometry) -> spawn::Assets {
    spawn::Assets {
        wheels: geometry
            .wheels
            .iter()
            .map(|wheel| spawn::WheelModel {
                position: wheel.anchor,
                shape: wheel.transform_index.map_or(NULL_INDEX, |index| index as u32),
            })
            .collect(),
        water_bounds: geometry.water_box,
        travel: tuning.suspension_anchor_offset,
        water_travel: tuning.water_suspension_offset,
        airborne_height: tuning.suspension_rest_length,
        gravity_coefficient: tuning.ground_gravity,
        model_gravity: geometry.gravity_scale,
        inverse_inertia: geometry.local_inverse_inertia,
        effect_defaults: [tuning.effect_thrust, tuning.effect_speed, tuning.effect_gravity],
    }
}

fn impact_model(
    bounds: Aabb,
    hull_box: Aabb,
    wheels: &[WheelDefinition],
    event_threshold: f32,
) -> gearbox_engine::vehicle::impacts::ImpactModel {
    const CEILING_FRACTION: f32 = 0.3;
    const WEAR_DELAY_MS: u32 = 5500;
    let references = [
        Vec3::new(bounds.center.x - bounds.half.x, bounds.center.y, bounds.center.z - bounds.half.z),
        Vec3::new(bounds.center.x - bounds.half.x, bounds.center.y, bounds.center.z),
        Vec3::new(bounds.center.x - bounds.half.x, bounds.center.y, bounds.center.z + bounds.half.z),
        Vec3::new(bounds.center.x, bounds.center.y, bounds.center.z + bounds.half.z),
        Vec3::new(bounds.center.x + bounds.half.x, bounds.center.y, bounds.center.z + bounds.half.z),
        Vec3::new(bounds.center.x + bounds.half.x, bounds.center.y, bounds.center.z),
        Vec3::new(bounds.center.x + bounds.half.x, bounds.center.y, bounds.center.z - bounds.half.z),
        Vec3::new(bounds.center.x, bounds.center.y, bounds.center.z - bounds.half.z),
        Vec3::new(bounds.center.x, bounds.center.y + bounds.half.y, bounds.center.z),
    ];
    let mut wear_thresholds = [15.0; 9];
    wear_thresholds[8] = 1.0;
    gearbox_engine::vehicle::impacts::ImpactModel {
        references,
        wear_thresholds,
        wear_scales: [0.05; 9],
        wear_delay_ms: WEAR_DELAY_MS,
        event_threshold,
        driven_wheels: std::array::from_fn(|index| wheels[index].steers),
        ceiling_height: hull_box.half.y * CEILING_FRACTION + hull_box.center.y,
    }
}
