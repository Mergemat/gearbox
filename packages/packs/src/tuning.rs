use std::collections::{HashMap, HashSet};

use anyhow::{Context, Result, bail, ensure};
use gearbox_engine::vehicle::{Tuning, forces::common::PiecewiseResponse, tuning::GearTable};
use gearbox_engine::{Curve, Key, Vec3, water::drag::FluidTuning};
use gearbox_gbx::reader::{FACADE, Reader};

use crate::packs::PackSet;
use gearbox_engine::vehicle::model::SurfaceMaterial as Material;

pub const CARS: [(&str, &str); 4] = [
    ("CarSport", "Vehicles\\Cars\\CarSport\\TuningsSport.VehicleTunings.Gbx"),
    ("CarSnow", "Vehicles\\Cars\\CarSnow\\TuningsSnow.VehicleTunings.Gbx"),
    ("CarRally", "Vehicles\\Cars\\CarRally\\CarRally.VehicleTunings.Gbx"),
    ("CarDesert", "Vehicles\\Cars\\CarDesert\\DesertCar.VehicleTunings.Gbx"),
];

const TUNINGS: u32 = 0x090ec000;
const TUNING: u32 = 0x090ed000;
const TUNING_LIST: u32 = 0x090eb000;
const WATER_MODEL: u32 = 0x0915f000;
const MATERIALS: u32 = 0x01026000;
const SHARED_GRAVITY: u32 = 0x090ed005;
const MAX_ELEMENTS: usize = 65536;

pub struct CarTunings {
    pub tunings: Vec<Tuning>,
    pub active: usize,
    fluids: Vec<FluidTuning>,
    water: Vec<Option<usize>>,
    materials: Vec<Option<Vec<Material>>>,
}

impl CarTunings {
    pub fn fluid(&self, index: usize) -> Result<FluidTuning> {
        let owner = self.water.get(index).copied().flatten().context("missing car water model")?;
        Ok(self.fluids[owner].clone())
    }

    pub fn active_materials(&self) -> Option<&[Material]> {
        self.materials[self.active].as_deref()
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Target {
    Water,
    Materials,
    Other,
}

#[derive(Clone, Copy)]
enum Node {
    Tuning,
    Water(usize),
}

struct Archive<'a> {
    reader: Reader<'a>,
    tunings: Vec<Tuning>,
    fluids: Vec<FluidTuning>,
    water: Vec<Option<usize>>,
    materials: Vec<Option<Vec<Material>>>,
    current: usize,
    nodes: HashMap<u32, Option<Node>>,
    spans: HashMap<usize, usize>,
    starts: HashMap<u32, usize>,
    refs: HashSet<u32>,
    ids_started: bool,
    material_nodes: HashMap<u32, Vec<Material>>,
}

pub fn decode(buf: &[u8]) -> Result<CarTunings> {
    let layout = crate::nodes::layout(buf)?;
    ensure!(layout.head.class_id == TUNINGS, "not VehicleTunings");
    let mut archive = Archive {
        reader: Reader::new(buf),
        tunings: Vec::new(),
        fluids: Vec::new(),
        water: Vec::new(),
        materials: Vec::new(),
        current: 0,
        nodes: HashMap::new(),
        spans: layout.nodes.iter().filter_map(|n| n.end.map(|end| (n.start, end))).collect(),
        starts: layout.nodes.iter().map(|n| (n.index, n.start)).collect(),
        refs: layout.head.refs.iter().copied().collect(),
        ids_started: false,
        material_nodes: HashMap::new(),
    };
    archive.reader.skip(layout.head.body_start)?;
    archive.tuning_list()
}

pub fn load(packs: &PackSet, car: &str) -> Result<CarTunings> {
    let file = CARS.iter().find(|entry| entry.0 == car).with_context(|| format!("unknown car {car}"))?.1;
    decode(&packs.read_path(file)?).with_context(|| format!("loading car tunings {file}"))
}

impl Archive<'_> {
    fn tuning_list(mut self) -> Result<CarTunings> {
        ensure!(self.word()? == TUNINGS, "VehicleTunings chunk");
        ensure!(self.word()? == 10, "unsupported VehicleTunings version");
        let count = self.word()? as usize;
        ensure!((1..128).contains(&count), "tuning count {count}");
        for index in 0..count {
            let node = self.word()?;
            ensure!(
                !self.nodes.contains_key(&node) && !self.refs.contains(&node),
                "unsupported shared tuning"
            );
            ensure!(self.word()? == TUNING, "expected car tuning class");
            let (tuning, fluid) = constructed();
            self.tunings.push(tuning);
            self.fluids.push(fluid);
            self.water.push(Some(index));
            self.materials.push(None);
            self.current = index;
            self.nodes.insert(node, Some(Node::Tuning));
            self.chunks(TUNING)?;
        }
        let active = self.word()? as usize;
        ensure!(active < count, "active tuning {active} / {count}");
        ensure!(self.word()? == FACADE && self.reader.remaining() == 0, "VehicleTunings trailing data");
        Ok(CarTunings {
            tunings: self.tunings,
            active,
            fluids: self.fluids,
            water: self.water,
            materials: self.materials,
        })
    }

    fn chunks(&mut self, class: u32) -> Result<()> {
        loop {
            let id = self.word()?;
            if id == FACADE {
                return Ok(());
            }
            let position = self.reader.pos() - 4;
            ensure!(self.chunk(id, self.current)?, "unsupported tuning chunk {id:08x} at {position:x}");
            if id == SHARED_GRAVITY {
                let tuning = &mut self.tunings[self.current];
                tuning.air_gravity = tuning.ground_gravity;
            }
            ensure!(id & !0xfff == class || id & !0xfff == TUNING_LIST, "unexpected chunk class {id:08x}");
        }
    }

    fn node(&mut self, target: Target) -> Result<()> {
        let index = self.word()?;
        let node = if index == u32::MAX || self.refs.contains(&index) {
            None
        } else if let Some(&node) = self.nodes.get(&index) {
            node
        } else if self.starts.get(&index).is_some_and(|&start| start != self.reader.pos() + 4) {
            None
        } else {
            let node = self.new_node(index, target)?;
            self.nodes.insert(index, node);
            node
        };
        let current = self.current;
        match target {
            Target::Water => {
                self.water[current] = match node {
                    Some(Node::Water(owner)) => Some(owner),
                    Some(Node::Tuning) => bail!("unsupported tuning as water model"),
                    None => None,
                }
            }
            Target::Materials => {
                if let Some(materials) = self.material_nodes.get(&index) {
                    self.materials[current] = Some(materials.clone());
                }
            }
            Target::Other => {}
        }
        Ok(())
    }

    fn new_node(&mut self, index: u32, target: Target) -> Result<Option<Node>> {
        let class = self.word()?;
        let start = self.reader.pos();
        match class {
            WATER_MODEL => {
                self.chunks(class)?;
                Ok(Some(Node::Water(self.current)))
            }
            MATERIALS if target == Target::Materials => {
                let materials = crate::car_aux::read_materials_body(&mut self.reader)?;
                self.skip_to(start)?;
                self.material_nodes.insert(index, materials);
                Ok(None)
            }
            _ if self.spans.contains_key(&start) => {
                self.skip_to(start)?;
                Ok(None)
            }
            _ => bail!("unsupported tuning node class {class:08x}"),
        }
    }

    fn skip_to(&mut self, start: usize) -> Result<()> {
        let end = *self.spans.get(&start).context("tuning node end unavailable")?;
        self.reader.skip(end - self.reader.pos())
    }

    fn node_array(&mut self, versioned: bool) -> Result<()> {
        if versioned {
            self.version(10)?;
        }
        let count = self.word()? as usize;
        ensure!(count < MAX_ELEMENTS, "tuning node array length {count}");
        for _ in 0..count {
            self.node(Target::Other)?;
        }
        Ok(())
    }

    fn word(&mut self) -> Result<u32> {
        self.reader.u32()
    }
    fn float(&mut self) -> Result<f32> {
        Ok(f32::from_bits(self.reader.u32()?))
    }
    fn skip(&mut self) -> Result<()> {
        self.reader.u32().map(drop)
    }

    fn version(&mut self, expected: u32) -> Result<()> {
        let version = self.word()?;
        ensure!(version == expected, "unsupported tuning chunk version {version}");
        Ok(())
    }

    fn id(&mut self) -> Result<()> {
        if !self.ids_started {
            ensure!(self.word()? == 3, "identifier archive version");
            self.ids_started = true;
        }
        let id = self.word()?;
        if matches!(id & 0xc000_0000, 0x4000_0000 | 0x8000_0000) && id & 0x3fff_ffff == 0 {
            self.reader.string()?;
        }
        Ok(())
    }

    fn curve(&mut self) -> Result<Curve> {
        self.version(3)?;
        let count = self.word()? as usize;
        ensure!(count < MAX_ELEMENTS, "curve has {count} keys");
        let flags = u32::from(self.reader.u16()?);
        ensure!(flags & Curve::SPLINE == 0, "unsupported spline car tuning curve");
        let keys =
            (0..count).map(|_| Ok(Key { x: self.float()?, y: self.float()? })).collect::<Result<_>>()?;
        Ok(Curve::new(flags, keys))
    }

    fn skip_curve(&mut self) -> Result<()> {
        self.version(3)?;
        let count = self.word()? as usize;
        ensure!(count < MAX_ELEMENTS, "curve has {count} keys");
        self.reader.skip(2 + count * 8)
    }

    fn floats(&mut self) -> Result<Vec<f32>> {
        let count = self.word()? as usize;
        ensure!(count < MAX_ELEMENTS, "tuning array has {count} elements");
        (0..count).map(|_| self.float()).collect()
    }

    fn chunk(&mut self, id: u32, t: usize) -> Result<bool> {
        match id {
            0x090eb000 => {
                self.id()?;
                self.node(Target::Other)?;
                self.skip()?;
            }
            0x090eb001 => {
                self.skip()?;
            }
            0x090ed000 => {
                self.skip()?;
                self.tunings[t].steering_angle_constant = self.float()?;
                self.tunings[t].steering_angle_speed_scale = self.float()?;
                self.tunings[t].forward_speed_limit = self.float()?;
                self.tunings[t].suspension_stiffness = self.float()?;
                self.tunings[t].suspension_damping = self.float()?;
                self.tunings[t].suspension_grip_upper_length = self.float()?;
                self.tunings[t].suspension_grip_lower_length = self.float()?;
                self.tunings[t].suspension_rest_length = self.float()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed001 => {
                self.id()?;
            }
            0x090ed002 => {
                self.tunings[t].mass = self.float()?;
                self.tunings[t].longitudinal_balance = self.float()?;
                self.tunings[t].center_height_offset = self.float()?;
            }
            0x090ed005 => {
                self.tunings[t].ground_gravity = self.float()?;
            }
            0x090ed007 => {
                self.tunings[t].steering_rate = self.float()?;
            }
            0x090ed009 => {
                self.tunings[t].brake_constant = self.float()?;
            }
            0x090ed00b => {
                self.tunings[t].inertia_mass = self.float()?;
                self.tunings[t].inertia_half.x = self.float()?;
                self.tunings[t].inertia_half.y = self.float()?;
                self.tunings[t].inertia_half.z = self.float()?;
            }
            0x090ed00d => {
                self.tunings[t].wheel_surface = self.word()? as u8;
            }
            0x090ed00e => {
                self.tunings[t].suspension_kind = self.word()?;
            }
            0x090ed010 => {
                self.tunings[t].model_kind = self.word()?;
                self.tunings[t].lateral_torque_scale = self.float()?;
            }
            0x090ed013 => {
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed014 => {
                self.skip()?;
                self.skip()?;
            }
            0x090ed017 => {
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed018 => {
                self.tunings[t].air_linear_drag = self.float()?;
                self.tunings[t].angular_drag_linear = self.float()?;
                self.tunings[t].angular_drag_quadratic = self.float()?;
            }
            0x090ed01d => {
                self.tunings[t].air_gravity = self.float()?;
                self.tunings[t].thrust_pitch_offset = self.float()?;
            }
            0x090ed01f => {
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed020 => {
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed023 => {
                self.tunings[t].impulse_angular_blend = self.float()?;
                self.tunings[t].steering_full_response_speed = self.float()?;
            }
            0x090ed026 => {
                self.tunings[t].grip_slope_lower = self.float()?;
                self.tunings[t].grip_slope_upper = self.float()?;
                self.tunings[t].thrust_slope_lower = self.float()?;
                self.tunings[t].thrust_slope_upper = self.float()?;
            }
            0x090ed027 => {
                self.tunings[t].slipping_steering_scale = self.float()?;
                self.tunings[t].grip_excess_blend = self.float()?;
            }
            0x090ed02e => {
                self.tunings[t].steering_thrust_loss_scale = self.float()?;
                self.skip()?;
            }
            0x090ed031 => {
                self.tunings[t].speed_limit_force = self.float()?;
                self.tunings[t].longitudinal_force_feedback = self.float()?;
            }
            0x090ed033 => {
                self.tunings[t].max_angular_velocity = self.float()?;
            }
            0x090ed034 => {
                self.tunings[t].impulse_speed_squared_limit = self.float()?;
            }
            0x090ed035 => {
                self.skip()?;
            }
            0x090ed039 => {
                self.skip()?;
                self.skip()?;
            }
            0x090ed03a => {
                self.skip()?;
                self.skip()?;
            }
            0x090ed03b => {
                self.skip()?;
            }
            0x090ed03c => {
                self.skip()?;
            }
            0x090ed03e => {
                self.tunings[t].body_contact_drag_ms = self.word()?;
            }
            0x090ed041 => {
                self.skip()?;
                self.skip()?;
            }
            0x090ed042 => {
                self.skip()?;
            }
            0x090ed043 => {
                self.skip()?;
            }
            0x090ed044 => {
                self.tunings[t].slip_thrust_demand_scale = self.float()?;
            }
            0x090ed047 => {
                self.skip()?;
            }
            0x090ed04d => {
                self.tunings[t].force_feedback_threshold = self.float()?;
            }
            0x090ed04e => {
                self.tunings[t].force_feedback_pitch_scale = self.float()?;
                self.tunings[t].force_feedback_roll_scale = self.float()?;
            }
            0x090ed051 => {
                self.tunings[t].pivot_arm_grip_scale = self.float()?;
            }
            0x090ed053 => {
                self.tunings[t].pivot_radial_damping = self.float()?;
            }
            0x090ed056 => {
                self.tunings[t].rpm_pivot_rate = self.float()?;
                self.tunings[t].rpm_air_acceleration = self.float()?;
                self.tunings[t].rpm_air_deceleration = self.float()?;
            }
            0x090ed05c => {
                self.tunings[t].reverse_brake_slipping_cap = self.float()?;
                self.tunings[t].reverse_brake_gripping_cap = self.float()?;
            }
            0x090ed05f => {
                self.skip()?;
            }
            0x090ed062 => {
                self.tunings[t].special_grip_yaw_scale = self.float()?;
            }
            0x090ed063 => {
                self.tunings[t].special_grip_steering_scale = self.float()?;
            }
            0x090ed064 => {
                self.tunings[t].special_grip_yaw_damping = self.float()?;
            }
            0x090ed065 => {
                self.tunings[t].special_grip_lateral_scale = self.float()?;
            }
            0x090ed066 => {
                self.skip()?;
            }
            0x090ed06a => {
                self.tunings[t].maximum_speed = self.float()?;
                self.tunings[t].reverse_selection_speed = self.float()?;
            }
            0x090ed06e => {
                self.tunings[t].suspension_anchor_offset = self.float()?;
            }
            0x090ed072 => {
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].drive_selector = self.word()?;
            }
            0x090ed078 => {
                self.skip()?;
            }
            0x090ed07d => {
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed084 => {
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed085 => {
                self.tunings[t].pipe_side_contact_enabled = self.word()?;
                self.skip()?;
                self.tunings[t].contact_volume_bottom = self.float()?;
                self.tunings[t].contact_volume_top = self.float()?;
                self.tunings[t].contact_volume_half_width = self.float()?;
                self.tunings[t].contact_volume_half_depth = self.float()?;
            }
            0x090ed086 => {
                self.skip()?;
                self.tunings[t].contact_volume_inner_scale = self.float()?;
            }
            0x090ed088 => {
                self.tunings[t].substep_distance = self.float()?;
            }
            0x090ed089 => {
                self.version(10)?;
                self.tunings[t].adhesion_downforce = self.curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
            }
            0x090ed08a => {
                self.version(8)?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
            }
            0x090ed08b => {
                self.version(37)?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.tunings[t].volume_impact_duration = self.curve()?;
                self.tunings[t].impact_thrust_by_speed = self.curve()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.tunings[t].reverse_speed_limit = self.float()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.tunings[t].impact_thrust_limit_ms = self.word()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].adhesion_enabled = self.word()?;
                self.tunings[t].adhesion_force_scale = self.float()?;
                self.tunings[t].adhesion_gravity_scale = self.float()?;
                self.tunings[t].adhesion_by_horizontal_speed = self.curve()?;
                self.tunings[t].adhesion_hold_ms = self.word()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].fallback_mass = self.float()?;
                self.tunings[t].impact_yaw_enabled = self.word()?;
                self.tunings[t].impact_yaw_duration = self.curve()?;
                self.node(Target::Other)?;
            }
            0x090ed08c => {
                self.version(1)?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed08d => {
                self.version(6)?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed08e => {
                self.version(4)?;
                self.node_array(true)?;
                self.node_array(true)?;
                self.node(Target::Other)?;
                self.node(Target::Other)?;
                self.node_array(false)?;
            }
            0x090ed094 => {
                self.version(5)?;
                self.tunings[t].turbo_thrust = self.float()?;
                self.skip_curve()?;
                self.tunings[t].turbo_duration_ms = self.word()?;
                self.tunings[t].red_turbo_thrust = self.float()?;
                self.skip_curve()?;
                self.tunings[t].red_turbo_duration_ms = self.word()?;
                self.tunings[t].turbo_fade_ms = self.word()?;
                self.tunings[t].red_turbo_fade_ms = self.word()?;
                self.tunings[t].reactor_turbo_thrust = self.float()?;
                self.skip_curve()?;
                self.tunings[t].reactor_turbo_duration_ms = self.word()?;
                self.tunings[t].reactor_turbo_fade_ms = self.word()?;
                self.tunings[t].timed_turbo_period_ms = self.word()?;
            }
            0x090ed095 => {
                self.version(7)?;
                self.tunings[t].air_control_window = self.word()?;
                self.tunings[t].yaw_hold_threshold = self.float()?;
                self.tunings[t].roll_damping = self.curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].level_air_control = self.word()?;
                self.tunings[t].pitch_control = self.curve()?;
                self.skip_curve()?;
                self.tunings[t].yaw_control = self.curve()?;
                self.tunings[t].pitch_control_enabled = self.word()?;
                self.tunings[t].pitch_limit_enabled = self.word()?;
                self.skip()?;
                self.tunings[t].pitch_limit_torque = self.curve()?;
                self.tunings[t].pitch_limit_degrees = self.float()?;
                self.tunings[t].pitch_limit_scale = self.curve()?;
            }
            0x090ed096 => {
                self.version(5)?;
                self.tunings[t].maximum_rpm = self.float()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].rpm_air_acceleration = self.float()?;
                self.tunings[t].rpm_air_deceleration = self.float()?;
                self.tunings[t].rpm_loaded_deceleration = self.float()?;
                self.tunings[t].rpm_loaded_acceleration = self.float()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].rpm_launch_rate = self.float()?;
                self.tunings[t].rpm_coast_rate = self.float()?;
                self.tunings[t].forward_launch_upper_speed = self.float()?;
                self.tunings[t].reverse_launch_upper_speed = self.float()?;
                self.tunings[t].forward_launch_lower_speed = self.float()?;
                self.tunings[t].reverse_launch_lower_speed = self.float()?;
                self.tunings[t].gear_ratios = GearTable(self.floats()?);
                self.tunings[t].upshift_ratios = GearTable(self.floats()?);
                self.tunings[t].downshift_ratios = GearTable(self.floats()?);
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed097 => {
                self.version(2)?;
                self.tunings[t].body_friction = self.float()?;
                self.tunings[t].dirt_body_friction = self.float()?;
                self.tunings[t].dirt_body_restitution = self.float()?;
                self.tunings[t].body_restitution = self.float()?;
                self.tunings[t].contact_restitution = self.float()?;
                self.tunings[t].dirt_contact_restitution = self.float()?;
                self.tunings[t].floor_friction_disabled = self.word()?;
                self.node(Target::Materials)?;
            }
            0x090ed098 => {
                self.version(2)?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed099 => {
                self.version(0)?;
                self.node(Target::Other)?;
            }
            0x090ed09a => {
                self.version(0)?;
                self.node(Target::Other)?;
            }
            0x090ed09b => {
                self.version(5)?;
                self.tunings[t].jump_forward = self.float()?;
                self.tunings[t].jump_up = self.float()?;
                self.tunings[t].held_jump_forward = self.float()?;
                self.tunings[t].held_jump_up = self.float()?;
                self.tunings[t].repeat_jump_forward = self.float()?;
                self.tunings[t].repeat_jump_up = self.float()?;
                self.tunings[t].jump_rearm_ms = self.word()?;
                self.tunings[t].jump_repeat_ms = self.word()?;
                self.skip()?;
                self.tunings[t].control_response.keys[0] = self.float()?;
                self.tunings[t].control_response.keys[1] = self.float()?;
                self.tunings[t].control_response.keys[2] = self.float()?;
                self.tunings[t].control_response.keys[3] = self.float()?;
                self.tunings[t].control_response.keys[4] = self.float()?;
                self.tunings[t].control_response.values[0] = self.float()?;
                self.tunings[t].control_response.values[1] = self.float()?;
                self.tunings[t].control_response.values[2] = self.float()?;
                self.tunings[t].control_response.values[3] = self.float()?;
                self.tunings[t].control_response.alternate_values[0] = self.float()?;
                self.tunings[t].control_response.alternate_values[1] = self.float()?;
                self.tunings[t].control_response.alternate_values[2] = self.float()?;
                self.tunings[t].control_response.alternate_values[3] = self.float()?;
                self.tunings[t].control_speed_scale = self.float()?;
                self.tunings[t].effect_thrust = self.float()?;
                self.tunings[t].effect_gravity = self.float()?;
                self.tunings[t].effect_speed = self.float()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
            }
            0x090ed09c => {
                self.version(21)?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.node(Target::Other)?;
                self.node(Target::Other)?;
                self.node(Target::Other)?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.node(Target::Other)?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
            }
            0x090ed09d => {
                self.version(21)?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
                self.node(Target::Water)?;
            }
            0x0915f000 => {
                self.version(3)?;
                self.fluids[t].scale = self.float()?;
                self.fluids[t].reference_speed = self.float()?;
                self.fluids[t].reference_length = self.float()?;
                self.fluids[t].tangential = self.curve()?;
                self.skip_curve()?;
                self.fluids[t].facing = self.curve()?;
                self.fluids[t].facing_exponent = self.float()?;
                self.fluids[t].trailing = self.curve()?;
                self.fluids[t].trailing_exponent = self.float()?;
                self.node(Target::Other)?;
            }
            0x090ed09e => {
                self.version(8)?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed09f => {
                self.version(14)?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed0a0 => {
                self.version(2)?;
                self.tunings[t].bumper_offset = self.float()?;
                self.tunings[t].bumper_forward = self.float()?;
                self.tunings[t].bumper_normal = self.float()?;
                self.tunings[t].bumper_alt_forward = self.float()?;
                self.tunings[t].bumper_alt_normal = self.float()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
            }
            0x090ed0a1 => {
                self.version(2)?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
            }
            0x090ed0a2 => {
                self.version(2)?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
            }
            0x090ed0a3 => {
                self.version(13)?;
                self.tunings[t].effect_base_grip = self.float()?;
                self.tunings[t].effect_minimum_grip = self.float()?;
                self.tunings[t].effect_rise_ms = self.word()?;
                self.tunings[t].effect_ground_fall_ms = self.word()?;
                self.tunings[t].steering_grip_blend = self.float()?;
                self.tunings[t].forward_hold_speed = self.float()?;
                self.tunings[t].reverse_hold_speed = self.float()?;
                self.tunings[t].effect_coefficient_minimum = self.float()?;
                self.tunings[t].low_grip_threshold = self.float()?;
                self.tunings[t].boost_steering_grip_blend = self.float()?;
                self.node(Target::Other)?;
                self.tunings[t].drive_steering_degrees = self.float()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.node(Target::Other)?;
                self.node(Target::Other)?;
                self.tunings[t].drive_steering_drag = self.curve()?;
                self.tunings[t].drive_lateral_friction = self.curve()?;
                self.tunings[t].effect_air_fall_ms = self.word()?;
                self.tunings[t].special_grip_blend_ms = self.word()?;
                self.tunings[t].special_grip_blend_enabled = self.word()?;
                self.tunings[t].effect_activation_speed = self.float()?;
                self.tunings[t].effect_idle_fall_ms = self.word()?;
                self.tunings[t].special_grip_transition_ms = self.word()?;
                self.skip()?;
                self.tunings[t].special_material_slip = self.curve()?;
                self.tunings[t].material_slip = self.curve()?;
            }
            0x090ed0a4 => {
                self.version(18)?;
                self.tunings[t].level_power_base = self.float()?;
                self.tunings[t].level_power_minimum = self.float()?;
                self.tunings[t].slow_motion_duration_ms = self.word()?;
                self.tunings[t].adhesion_angle_enabled = self.word()?;
                self.tunings[t].adhesion_by_angle = self.curve()?;
                self.skip()?;
                self.tunings[t].adhesion_angle_degrees = self.float()?;
                self.skip()?;
                self.tunings[t].fragility_delay_ms = self.word()?;
                self.skip()?;
                self.tunings[t].boost_thrust_by_speed = self.curve()?;
                self.tunings[t].adhesion_by_forward_speed = self.curve()?;
                self.tunings[t].boost_thrust_by_time = self.curve()?;
                self.tunings[t].special_material_enabled = self.word()?;
                self.tunings[t].boost_drag_by_speed = self.curve()?;
                self.tunings[t].boost_drag_by_time = self.curve()?;
                self.skip()?;
                self.tunings[t].material_visual_steering = self.word()?;
                self.tunings[t].air_angular_drag = self.curve()?;
                self.skip()?;
                self.tunings[t].collision_scale = self.float()?;
                self.skip()?;
            }
            0x090ed0a5 => {
                self.version(2)?;
                self.skip()?;
                self.tunings[t].hover_side_enabled = self.word()?;
                self.tunings[t].hover_spring_strength = self.float()?;
                self.tunings[t].hover_spring_damping = self.float()?;
                self.tunings[t].hover_rest_distance = self.float()?;
                self.tunings[t].hover_down_reach = self.float()?;
                self.tunings[t].hover_side_reach = self.float()?;
                self.tunings[t].hover_side_speed_reach = self.float()?;
                self.tunings[t].hover_side_velocity = self.curve()?;
                self.tunings[t].hover_side_distance = self.curve()?;
            }
            0x090ed0a6 => {
                self.version(15)?;
                self.tunings[t].drive_yaw_by_alignment = self.curve()?;
                self.tunings[t].drive_yaw_by_rate = self.curve()?;
                self.tunings[t].drive_yaw_by_speed = self.curve()?;
                self.tunings[t].grip_meter_recovery_ms = self.word()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].low_grip_visual_rate = self.float()?;
                self.tunings[t].drive_lateral_force_by_side_speed = self.curve()?;
                self.tunings[t].drive_thrust_by_side_speed = self.curve()?;
                self.tunings[t].low_grip_visual_angle = self.float()?;
                self.tunings[t].drive_brake_by_speed = self.curve()?;
                self.tunings[t].grip_meter_minimum = self.float()?;
                self.tunings[t].drive_transition_thrust = self.curve()?;
                self.tunings[t].drive_thrust_by_forward_alignment = self.curve()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].drive_thrust_by_grip = self.curve()?;
                self.tunings[t].drive_thrust_by_side = self.curve()?;
                self.tunings[t].drive_longitudinal_drag = self.curve()?;
                self.tunings[t].drive_yaw_by_total_speed = self.curve()?;
                self.tunings[t].drift_thrust_recovery_ms = self.word()?;
                self.tunings[t].drift_release_ms = self.word()?;
                self.tunings[t].drive_thrust_by_slip = self.curve()?;
            }
            0x090ed0a7 => {
                self.version(19)?;
                self.tunings[t].turbo_boost_enabled = self.word()?;
                self.skip()?;
                self.tunings[t].turbo_boost_duration_ms = self.word()?;
                self.tunings[t].level_two_ground_force = self.float()?;
                self.tunings[t].ground_pull_strength = self.float()?;
                self.tunings[t].ground_pull_downforce = self.float()?;
                self.tunings[t].ground_pull_reach = self.float()?;
                self.tunings[t].ground_pull_by_distance = self.curve()?;
                self.tunings[t].ground_pull_by_speed = self.curve()?;
                self.tunings[t].effect_duration_ms = self.word()?;
                self.tunings[t].level_air_control_delay_ms = self.word()?;
                self.tunings[t].level_thrust = self.curve()?;
                self.skip()?;
                self.tunings[t].level_start_push = self.float()?;
                self.tunings[t].level_two_model_scale = self.float()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].level_start_push_ms = self.word()?;
                self.skip()?;
                self.node(Target::Other)?;
                self.skip()?;
                self.tunings[t].level_brake = self.curve()?;
                self.tunings[t].ground_push_by_distance = self.curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.tunings[t].level_two_air_force = self.float()?;
                self.tunings[t].ground_push_by_vertical_speed = self.curve()?;
                self.tunings[t].ground_push_by_forward_speed = self.curve()?;
                self.tunings[t].level_two_down_reach = self.float()?;
                self.tunings[t].level_two_inverted_force_scale = self.float()?;
                self.tunings[t].ground_push_by_time = self.curve()?;
                self.tunings[t].level_side_slip = self.curve()?;
                self.tunings[t].level_lateral_damping = self.curve()?;
                self.tunings[t].boost_visual_fade_ms = self.word()?;
                self.tunings[t].ground_roll_by_speed = self.curve()?;
                self.tunings[t].ground_roll_by_time = self.curve()?;
                self.tunings[t].hover_water_fraction = self.float()?;
                self.tunings[t].ground_pull_water_scale = self.float()?;
            }
            0x090ed0a8 => {
                self.version(10)?;
                self.tunings[t].direction_drag_enabled = self.word()?;
                self.tunings[t].direction_off_axis_drag = self.curve()?;
                self.tunings[t].meter_thrust = self.curve()?;
                self.tunings[t].meter_steering = self.curve()?;
                self.tunings[t].direction_vertical_drag = self.curve()?;
                self.tunings[t].meter_ground_drag = self.curve()?;
                self.tunings[t].contact_rearm_ms = self.word()?;
                self.tunings[t].sample_window_ms = self.word()?;
                self.tunings[t].sample_expected_scale = self.float()?;
                self.skip_curve()?;
                self.tunings[t].direction_speed_drag = self.curve()?;
                self.tunings[t].fragility_rate = self.float()?;
                self.tunings[t].fragility_minimum = self.float()?;
                self.tunings[t].wheel_sample_expected = self.curve()?;
            }
            0x090ed0a9 => {
                self.version(4)?;
                self.tunings[t].contact_impact_timer_enabled = self.word()?;
                self.tunings[t].contact_impact_limit_ms = self.word()?;
                self.tunings[t].contact_impact_duration_by_speed = self.curve()?;
                self.tunings[t].contact_impact_thrust_by_speed = self.curve()?;
                self.tunings[t].hover_forces_enabled = self.word()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
            }
            0x090ed0aa => {
                self.version(0)?;
                self.tunings[t].pipe_side_angle = self.float()?;
            }
            0x090ed0ab => {
                self.version(1)?;
                self.tunings[t].slow_start_enabled = self.word()?;
                self.tunings[t].slow_start_release_ms = self.word()?;
                self.tunings[t].slow_start_speed_limit = self.float()?;
                self.tunings[t].slow_start_boost = self.curve()?;
            }
            0x090ed0ac => {
                self.version(0)?;
                self.skip()?;
            }
            0x090ed0ad => {
                self.version(11)?;
                self.tunings[t].water_angular_drag = self.curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.node(Target::Other)?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].water_enabled = self.word()?;
                self.node(Target::Other)?;
                self.tunings[t].water_yaw_by_steering = self.curve()?;
                self.skip()?;
                self.skip()?;
                self.node(Target::Other)?;
                self.tunings[t].water_lift_by_heading = self.curve()?;
                self.node(Target::Other)?;
                self.tunings[t].water_lift_by_speed = self.curve()?;
                self.tunings[t].water_lift_by_vertical_speed = self.curve()?;
                self.tunings[t].water_wheel_drag = self.curve()?;
                self.tunings[t].water_lateral_force = self.curve()?;
                self.tunings[t].water_righting_enabled = self.word()?;
                self.tunings[t].immersion_rise_ms = self.word()?;
                self.tunings[t].water_transition_ms = self.word()?;
                self.tunings[t].water_righting_by_tilt = self.curve()?;
                self.tunings[t].water_exit_righting_by_rate = self.curve()?;
                self.tunings[t].wet_grip = self.curve()?;
                self.tunings[t].immersion_rise_rate = self.float()?;
                self.tunings[t].water_drag_immersion_power = self.float()?;
                self.tunings[t].wet_brake = self.curve()?;
                self.node(Target::Other)?;
                self.node(Target::Other)?;
                self.tunings[t].immersion_drain_by_speed = self.curve()?;
                self.tunings[t].buoyancy_fraction_power = self.float()?;
                self.tunings[t].water_exit_righting_ms = self.word()?;
            }
            0x090ed0ae => {
                self.version(2)?;
                self.tunings[t].special_contact_depth_enabled = self.word()?;
                self.tunings[t].special_contact_depth_by_speed = self.curve()?;
                self.skip()?;
            }
            0x090ed0af => {
                self.version(0)?;
                self.tunings[t].grip_by_speed = self.curve()?;
                self.tunings[t].slipping_grip_scale = self.float()?;
                self.tunings[t].coast_friction = self.float()?;
                self.tunings[t].suspension_recovery_rate = self.float()?;
                self.tunings[t].lateral_stiffness = self.float()?;
                self.tunings[t].grip_torque_by_speed = self.curve()?;
                self.tunings[t].body_contact_drag = self.curve()?;
                self.tunings[t].steering_thrust_loss = self.curve()?;
                self.tunings[t].grip_torque_by_normal = self.curve()?;
                self.tunings[t].yaw_hold_threshold = self.float()?;
                self.tunings[t].brake_speed_scale = self.float()?;
                self.tunings[t].brake_slipping_cap = self.float()?;
                self.tunings[t].brake_gripping_cap = self.float()?;
                self.tunings[t].coast_drag = self.float()?;
                self.tunings[t].steering_by_speed = self.curve()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].slip_thrust = self.curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.fluids[t].scale = self.float()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.tunings[t].grip_by_suspension = self.curve()?;
                self.tunings[t].pivot_maximum_speed = self.float()?;
                self.tunings[t].pivot_radius_by_speed = self.curve()?;
                self.tunings[t].side_speed_roll_torque = self.curve()?;
                self.tunings[t].force_feedback_mass = self.float()?;
                self.tunings[t].reverse_pivot_power_threshold = self.float()?;
                self.tunings[t].engine_thrust = self.curve()?;
                self.tunings[t].reverse_thrust = self.curve()?;
                self.tunings[t].pivot_lateral_force_by_radius = self.curve()?;
                self.tunings[t].pivot_lateral_stiffness = self.float()?;
                self.tunings[t].pivot_minimum_speed = self.float()?;
                self.tunings[t].pivot_angle_stiffness = self.float()?;
                self.tunings[t].pivot_radial_stiffness = self.float()?;
                self.tunings[t].pivot_minimum_radius = self.float()?;
                self.tunings[t].pivot_maximum_lateral_speed = self.float()?;
                self.tunings[t].pivot_return_yaw_damping = self.float()?;
                self.tunings[t].pivot_outer_yaw_damping = self.float()?;
                self.tunings[t].slipping_brake_scale = self.float()?;
                self.tunings[t].pivot_arm_ms = self.word()?;
                self.tunings[t].pivot_arm_thrust_scale = self.float()?;
                self.tunings[t].pivot_retire_thrust_scale = self.float()?;
                self.tunings[t].pivot_retire_ms = self.word()?;
                self.tunings[t].slipping_brake_grip_scale = self.float()?;
                self.skip()?;
                self.skip()?;
                self.skip()?;
                self.tunings[t].pivot_torque_by_forward_speed = self.curve()?;
                self.tunings[t].pivot_torque_by_side_speed = self.curve()?;
                self.tunings[t].pivot_normal_angle_limit = self.float()?;
                self.tunings[t].fixed_spin_speed = self.float()?;
                self.tunings[t].pivot_positive_angle_limit = self.float()?;
                self.tunings[t].pivot_negative_angle_limit = self.float()?;
                self.tunings[t].pivot_retire_extra_thrust = self.float()?;
                self.tunings[t].steering_angle_by_speed = self.curve()?;
                self.skip()?;
                self.skip()?;
                self.skip_curve()?;
                self.tunings[t].roll_stabilization_enabled = self.word()?;
                self.tunings[t].roll_stabilization_torque = self.curve()?;
                self.tunings[t].roll_stabilization_force = self.curve()?;
                self.tunings[t].roll_angular_damping = self.float()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
                self.skip_curve()?;
            }
            0x090ed0b0 => {
                self.version(1)?;
                self.skip()?;
                self.skip()?;
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}

fn constructed() -> (Tuning, FluidTuning) {
    let tuning = Tuning {
        effect_thrust: 1.5,
        effect_speed: 1.5,
        effect_gravity: 1.5,
        reactor_turbo_thrust: 3.0,
        hover_forces_enabled: 0,
        input_torque_scale: 10.0,
        drive_selector: 0,
        air_control_window: 0,
        yaw_hold_threshold: 3.1415927,
        level_air_control: 0,
        pitch_control_enabled: 0,
        pitch_limit_enabled: 1,
        suspension_kind: 2,
        suspension_stiffness: 0.0,
        suspension_damping: 0.0,
        suspension_rest_length: 0.0,
        suspension_grip_upper_length: 0.0,
        suspension_grip_lower_length: 0.0,
        suspension_anchor_offset: 0.0,
        suspension_recovery_rate: 5.0,
        mass: 800.0,
        inertia_half: Vec3::new(0.5, 0.5, 0.5),
        water_suspension_offset: 0.09,
        inertia_mass: 1.0,
        longitudinal_balance: 0.0,
        center_height_offset: 0.0,
        maximum_speed: 277.77777,
        max_angular_velocity: 100.0,
        substep_distance: 0.3,
        impulse_speed_squared_limit: 10000.0,
        impulse_y_blend: 1.0,
        impulse_angular_blend: 1.0,
        speed_limit_force: 10.0,
        air_linear_drag: 0.1,
        ground_gravity: 1.0,
        air_gravity: 1.0,
        angular_drag_linear: 0.5,
        angular_drag_quadratic: 1.0,
        turbo_thrust: 3.0,
        turbo_duration_ms: 1000,
        turbo_fade_ms: 1000,
        red_turbo_thrust: 3.0,
        red_turbo_duration_ms: 1000,
        red_turbo_fade_ms: 1000,
        reactor_turbo_duration_ms: 1000,
        reactor_turbo_fade_ms: 1000,
        timed_turbo_period_ms: 1000,
        wheel_surface: 9,
        contact_restitution: 0.0,
        dirt_contact_restitution: 0.0,
        body_friction: 0.8,
        dirt_body_friction: 0.4,
        dirt_body_restitution: 0.0,
        body_restitution: 0.0,
        floor_friction_disabled: 0,
        water_enabled: 0,
        water_righting_enabled: 0,
        water_thrust_reduction: 1,
        immersion_rise_ms: 200,
        water_transition_ms: 50,
        immersion_rise_rate: 0.33333334,
        water_drag_immersion_power: 3.0,
        buoyancy_fraction_power: 5.0,
        water_exit_righting_ms: 1500,
        special_contact_depth_enabled: 0,
        forward_speed_limit: 55.555557,
        reverse_speed_limit: 13.888889,
        reverse_selection_speed: 10.0,
        maximum_rpm: 11000.0,
        rpm_launch_rate: 10000.0,
        rpm_coast_rate: 4000.0,
        forward_launch_upper_speed: 10.0,
        forward_launch_lower_speed: -2.0,
        reverse_launch_upper_speed: 2.0,
        reverse_launch_lower_speed: -10.0,
        rpm_air_acceleration: 12000.0,
        rpm_air_deceleration: 8000.0,
        rpm_loaded_deceleration: 10000.0,
        rpm_loaded_acceleration: 7000.0,
        gear_ratios: GearTable(vec![338.0, 297.0, 207.0, 143.0, 100.0, 84.0, 56.0]),
        upshift_ratios: GearTable(vec![0.0, 0.8125, 0.8125, 0.8125, 0.8125, 0.8125, 1.0]),
        downshift_ratios: GearTable(vec![0.0, 0.0, 0.375, 0.53125, 0.5625, 0.625, 0.625]),
        rpm_pivot_rate: 5000.0,
        effect_base_grip: 0.2,
        effect_minimum_grip: 0.8,
        effect_activation_speed: 5.0,
        effect_rise_ms: 1650,
        effect_ground_fall_ms: 3300,
        effect_air_fall_ms: 6000,
        effect_idle_fall_ms: 10000,
        forward_hold_speed: 6.0,
        reverse_hold_speed: -6.0,
        low_grip_threshold: 0.5,
        effect_coefficient_minimum: 0.05,
        steering_grip_blend: 0.45,
        boost_steering_grip_blend: 0.75,
        drive_steering_degrees: 45.0,
        special_grip_blend_enabled: 0,
        special_grip_blend_ms: 0,
        special_grip_transition_ms: 0,
        grip_meter_recovery_ms: 500,
        grip_meter_minimum: 0.2,
        low_grip_visual_angle: 45.0,
        low_grip_visual_rate: 5.0,
        drift_thrust_recovery_ms: 400,
        drift_release_ms: 300,
        direction_drag_enabled: 0,
        contact_rearm_ms: 100,
        sample_window_ms: 100,
        sample_expected_scale: 20.0,
        fragility_rate: 0.33333334,
        fragility_minimum: 0.5,
        slow_start_enabled: 0,
        slow_start_speed_limit: 40.0,
        slow_start_release_ms: 2000,
        adhesion_enabled: 1,
        adhesion_angle_enabled: 0,
        adhesion_angle_degrees: 30.0,
        adhesion_force_scale: 9.81,
        adhesion_gravity_scale: 0.3333,
        adhesion_hold_ms: 400,
        fixed_spin_minimum_speed: 10.0,
        fixed_spin_maximum_speed: 80.0,
        collision_scale: 1.0,
        roll_stabilization_enabled: 0,
        roll_stabilization_ms: 1000,
        model_kind: 6,
        brake_constant: 20.0,
        brake_speed_scale: 0.0,
        brake_slipping_cap: 500.0,
        brake_gripping_cap: 500.0,
        coast_friction: 1.0,
        coast_drag: 0.0,
        longitudinal_force_feedback: 0.0,
        steering_angle_constant: 1.0,
        steering_angle_speed_scale: 0.5,
        steering_full_response_speed: 20.0,
        steering_thrust_loss_scale: 0.0,
        steering_rate: 20.0,
        lateral_torque_scale: 0.01,
        slipping_steering_scale: 0.0,
        lateral_stiffness: -1.0,
        slipping_grip_scale: 0.6,
        grip_excess_blend: 0.0,
        thrust_pitch_offset: 0.0,
        grip_slope_lower: 0.0,
        grip_slope_upper: 0.0,
        thrust_slope_lower: 0.0,
        thrust_slope_upper: 0.0,
        lateral_excess_blend: 1.0,
        material_visual_steering: 0,
        slip_thrust_scale: 1.0,
        body_contact_drag_ms: 500,
        slip_thrust_demand_scale: 1.0,
        force_feedback_mass: 5.0,
        reverse_pivot_power_threshold: 200.0,
        force_feedback_threshold: 0.01,
        force_feedback_pitch_scale: 0.1,
        force_feedback_roll_scale: 0.1,
        slipping_brake_scale: 0.5,
        slipping_brake_grip_scale: 0.2,
        reverse_brake_slipping_cap: 100.0,
        reverse_brake_gripping_cap: 50.0,
        pivot_maximum_speed: 10.0,
        pivot_minimum_speed: 3.0,
        pivot_lateral_stiffness: 1.0,
        pivot_angle_stiffness: 6.0,
        pivot_return_yaw_damping: 2.0,
        pivot_outer_yaw_damping: 0.5,
        pivot_lateral_yaw_damping: 0.5,
        pivot_radial_stiffness: 1.0,
        pivot_radial_damping: 0.2,
        pivot_minimum_radius: 20.0,
        pivot_maximum_lateral_speed: 10.0,
        pivot_normal_angle_limit: 0.35,
        pivot_positive_angle_limit: 1.7453,
        pivot_negative_angle_limit: 0.785,
        pivot_arm_ms: 1000,
        pivot_arm_thrust_scale: 0.5,
        pivot_arm_grip_scale: 0.01,
        pivot_retire_ms: 500,
        pivot_retire_thrust_scale: 4.0,
        pivot_retire_extra_thrust: 10.0,
        fixed_spin_speed: 150.0,
        special_grip_yaw_scale: 0.05,
        special_grip_steering_scale: 20.0,
        special_grip_yaw_damping: 15.0,
        special_grip_lateral_scale: 1.0,
        special_material_enabled: 0,
        fragility_delay_ms: 0,
        fallback_mass: 1500.0,
        hover_side_enabled: 0,
        hover_side_speed_reach: 0.25,
        hover_spring_strength: 20.0,
        hover_spring_damping: 1.5,
        hover_down_reach: 2.0,
        hover_rest_distance: 0.5,
        hover_side_reach: 1.0,
        impact_thrust_limit_ms: 500,
        impact_yaw_enabled: 0,
        contact_impact_timer_enabled: 0,
        contact_impact_limit_ms: 500,
        pipe_side_contact_enabled: 0,
        contact_volume_bottom: 0.2,
        contact_volume_top: 0.6,
        contact_volume_half_width: 0.2,
        contact_volume_half_depth: 1.7,
        contact_volume_inner_scale: 0.5,
        pipe_side_width_scale: 0.7,
        pipe_side_angle: 0.7853982,
        jump_forward: 25.0,
        jump_up: 15.0,
        held_jump_forward: 10.0,
        held_jump_up: 6.0,
        repeat_jump_forward: 0.5,
        repeat_jump_up: 0.2,
        jump_rearm_ms: 300,
        jump_repeat_ms: 150,
        control_response: PiecewiseResponse {
            keys: [25.0, 50.0, 125.0, 250.0, 350.0],
            values: [1.0, 0.5, 0.2, 0.1],
            alternate_values: [3.0, 1.5, 0.8, 0.4],
        },
        control_speed_scale: 100.0,
        bumper_offset: -0.015,
        bumper_forward: 25.0,
        bumper_normal: 20.0,
        bumper_alt_forward: 75.0,
        bumper_alt_normal: 20.0,
        support_decay: 0.05,
        turbo_boost_enabled: 0,
        level_two_ground_force: 10.0,
        level_two_air_force: 6.0,
        level_two_inverted_force_scale: 2.0,
        ground_pull_strength: 4.0,
        ground_pull_downforce: 0.5,
        turbo_boost_duration_ms: 6000,
        level_start_push_ms: 200,
        level_start_push: 2.0,
        ground_pull_reach: 10.0,
        level_two_down_reach: 10.0,
        hover_water_fraction: 0.35,
        ground_pull_water_scale: 3.0,
        level_two_model_scale: 2.5,
        effect_duration_ms: 30,
        level_air_control_delay_ms: 0,
        boost_visual_fade_ms: 1000,
        level_power_base: 0.57,
        slow_motion_duration_ms: 10000,
        level_power_minimum: 0.11111111,
        roll_damping: curve(&[[0.0, 1.0]]),
        pitch_limit_torque: curve(&[[0.0, 0.0]]),
        pitch_control: curve(&[[0.0, 0.0]]),
        yaw_control: curve(&[[0.0, 0.0]]),
        immersion_drain_by_speed: curve(&[[0.0, 1.0]]),
        water_lift_by_heading: curve(&[[0.0, 1.0]]),
        water_yaw_by_steering: curve(&[[0.0, 0.0]]),
        water_angular_drag: curve(&[[0.0, 0.0]]),
        water_lift_by_speed: curve(&[[0.0, 0.0]]),
        water_lift_by_vertical_speed: curve(&[[0.0, 0.0]]),
        water_wheel_drag: curve(&[[0.0, 0.0]]),
        water_lateral_force: curve(&[[0.0, 0.0]]),
        water_righting_by_tilt: curve(&[[0.0, 0.0]]),
        water_exit_righting_by_rate: curve(&[[0.0, 0.0]]),
        wet_grip: curve(&[[0.0, 1.0]]),
        wet_brake: curve(&[[0.0, 1.0]]),
        special_contact_depth_by_speed: curve(&[[0.0, 0.0]]),
        reverse_thrust: curve(&[[-100.0, 2.0], [-50.0, 5.0], [0.0, 10.0]]),
        special_material_slip: curve(&[[0.0, 0.0], [0.05, 0.6], [0.2, 0.85], [1.0, 1.0]]),
        material_slip: curve(&[[0.0, 0.0], [0.8, 0.3], [0.95, 0.7], [1.0, 1.0]]),
        drive_lateral_friction: curve(&[[0.0, 0.3], [100.0, 3.0], [300.0, 4.0]]),
        drive_steering_drag: curve(&[[0.0, 0.0]]),
        drive_yaw_by_speed: curve(&[[0.0, 7.0], [200.0, 6.0], [400.0, 5.0]]),
        drive_yaw_by_total_speed: curve(&[[0.0, 1.0]]),
        drive_yaw_by_alignment: curve(&[[0.0, 8.0], [0.5, 2.0], [1.0, 8.0]]),
        drive_yaw_by_rate: curve(&[[0.0, 0.0], [10.0, 1.0]]),
        drive_thrust_by_forward_alignment: curve(&[[0.0, 1.0]]),
        drive_lateral_force_by_side_speed: curve(&[[0.0, 0.0]]),
        drive_longitudinal_drag: curve(&[[0.0, 0.0]]),
        drive_brake_by_speed: curve(&[[0.0, 0.0]]),
        drive_transition_thrust: curve(&[[0.0, 0.0]]),
        drive_thrust_by_grip: curve(&[[0.0, 1.0]]),
        drive_thrust_by_side: curve(&[[0.0, 1.0]]),
        direction_speed_drag: curve(&[[20.0, 0.0], [50.0, 0.5], [100.0, 1.0], [200.0, 1.25], [300.0, 1.5]]),
        direction_off_axis_drag: curve(&[[10.0, 0.0], [25.0, 50.0], [100.0, 60.0], [250.0, 75.0]]),
        direction_vertical_drag: curve(&[
            [10.0, 0.0],
            [17.0, 2.0],
            [25.0, 15.0],
            [37.0, 32.0],
            [50.0, 36.0],
            [250.0, 44.0],
        ]),
        meter_thrust: curve(&[[0.0, 1.0], [0.25, 0.8], [0.5, 0.3], [1.0, 0.2]]),
        meter_steering: curve(&[[0.0, 1.0], [0.5, 0.25], [1.0, 0.1]]),
        meter_ground_drag: curve(&[[0.0, 0.0], [25.0, 2.0], [50.0, 4.0], [100.0, 10.0]]),
        adhesion_by_angle: curve(&[[0.0, 0.0]]),
        adhesion_by_forward_speed: curve(&[[0.0, 0.0]]),
        adhesion_by_horizontal_speed: curve(&[[20.0, 0.0], [100.0, 50.0], [300.0, 200.0]]),
        roll_stabilization_torque: curve(&[[0.0, 0.0], [40.0, 0.0], [60.0, 0.1], [90.0, 0.3]]),
        roll_stabilization_force: curve(&[[0.0, 0.0], [40.0, 0.0], [60.0, 0.1], [90.0, 0.3]]),
        body_contact_drag: curve(&[[0.0, 2.0]]),
        steering_thrust_loss: curve(&[[0.0, 0.0]]),
        steering_by_speed: curve(&[[0.0, 10.0]]),
        grip_by_speed: curve(&[[0.0, 30.0]]),
        grip_torque_by_speed: curve(&[[0.0, 0.0]]),
        grip_torque_by_normal: curve(&[[0.0, 1.0]]),
        pivot_lateral_force_by_radius: curve(&[[0.0, 0.0], [1.0, 30.0], [10.0, 30.0]]),
        pivot_torque_by_side_speed: curve(&[[0.0, 0.0], [10.0, 5.0], [20.0, 6.0]]),
        pivot_torque_by_forward_speed: curve(&[[0.0, 1.5], [10.0, 1.2], [50.0, 0.2]]),
        boost_drag_by_speed: curve(&[[0.0, 0.0], [50.0, 0.1], [150.0, 0.15], [200.0, 0.4]]),
        boost_drag_by_time: curve(&[[1.0, 0.1], [4.0, 1.0]]),
        boost_thrust_by_speed: curve(&[[50.0, 1.0], [200.0, 0.9], [400.0, 0.6]]),
        boost_thrust_by_time: curve(&[[1.0, 0.9], [2.0, 0.8], [3.0, 0.7], [4.0, 0.5]]),
        air_angular_drag: curve(&[[0.0, 0.0]]),
        adhesion_downforce: curve(&[[0.0, 0.0], [100.0, 2.0]]),
        hover_side_velocity: curve(&[[0.0, 0.0], [10.0, 25.0], [20.0, 75.0], [40.0, 150.0]]),
        volume_impact_duration: curve(&[[0.0, 0.0]]),
        impact_thrust_by_speed: curve(&[[0.0, 0.1]]),
        impact_yaw_duration: curve(&[[0.0, 0.0]]),
        contact_impact_thrust_by_speed: curve(&[[0.0, 0.0]]),
        contact_impact_duration_by_speed: curve(&[[0.0, 0.0]]),
        ground_pull_by_distance: curve(&[[0.0, 1.25], [1.0, 0.0]]),
        ground_pull_by_speed: curve(&[
            [-200.0, 50.0],
            [-100.0, 20.0],
            [-25.0, 1.0],
            [0.0, 0.0],
            [25.0, -1.5],
        ]),
        level_thrust: curve(&[
            [100.0, 8.0],
            [150.0, 4.0],
            [200.0, 3.0],
            [300.0, 2.5],
            [400.0, 1.5],
            [500.0, 1.0],
            [800.0, 0.5],
            [1000.0, 0.0],
        ]),
        level_side_slip: curve(&[[0.0, 1.0]]),
        level_lateral_damping: curve(&[
            [0.0, 0.0],
            [50.0, 0.05],
            [100.0, 0.15],
            [250.0, 0.175],
            [500.0, 0.2],
        ]),
        level_brake: curve(&[[-600.0, 0.0], [-200.0, 1.0], [-100.0, 2.0], [0.0, 3.0]]),
        ground_push_by_distance: curve(&[[0.0, 25.0], [1.0, 10.0]]),
        ground_push_by_vertical_speed: curve(&[[-50.0, 1.5], [0.0, 0.0]]),
        ground_push_by_forward_speed: curve(&[[300.0, 1.0], [400.0, 0.8], [600.0, 0.6], [800.0, 0.2]]),
        ground_push_by_time: curve(&[[700.0, 1.0], [800.0, 0.8], [900.0, 0.6], [1300.0, 0.2], [1500.0, 0.1]]),
        ground_roll_by_speed: curve(&[[0.0, 0.0], [100.0, 15.0], [500.0, 20.0]]),
        ground_roll_by_time: curve(&[
            [0.0, 0.0],
            [1000.0, 0.1],
            [1500.0, 0.25],
            [2000.0, 0.5],
            [2500.0, 1.0],
        ]),
        pitch_limit_scale: curve(&[[0.0, 0.0]]),
        engine_thrust: curve(&[
            [-100.0, 50.0],
            [0.0, 50.0],
            [100.0, 1.0],
            [200.0, 0.0],
            [300.0, 0.0],
            [400.0, 0.0],
        ]),
        wheel_sample_expected: curve(&[[0.0, 1.0], [0.25, 0.95], [0.5, 0.8], [0.75, 0.6], [1.0, 0.25]]),
        slow_start_boost: curve(&[[1000.0, 1.0], [2000.0, 4.0]]),
        slip_thrust: curve(&[[-100.0, 50.0], [0.0, 50.0], [100.0, 1.0]]),
        grip_by_suspension: curve(&[[0.0, 1.0], [0.5, 2.0], [1.0, 3.0]]),
        side_speed_roll_torque: curve(&[[0.0, 0.0], [1.0, 0.8], [10.0, 1.0], [20.0, 1.0], [30.0, 1.0]]),
        pivot_radius_by_speed: curve(&[[0.0, 1.0], [5.0, 2.0], [10.0, 3.0]]),
        hover_side_distance: curve(&[[0.0, 8.0], [0.5, 1.0], [1.0, 0.0]]),
        drive_thrust_by_side_speed: curve(&[[0.0, 0.0]]),
        drive_thrust_by_slip: curve(&[[0.0, 1.07], [1.0, 1.05]]),
        steering_angle_by_speed: curve(&[]),
        contact_angular_response: curve(&[[0.0, 1.0], [5.0, 0.25], [10.0, 0.1], [20.0, 0.05]]),
        support_by_speed: curve(&[[0.0, 1.0], [5.0, 0.97], [10.0, 0.9], [20.0, 0.85]]),
        pitch_limit_degrees: 5.0,
        roll_vertical_speed_limit: -10.0,
        roll_angular_damping: 0.1,
        roll_force_scale: 0.3,
        ..Default::default()
    };
    let fluid = FluidTuning {
        reference_speed: 10.0,
        scale: 0.0,
        facing: curve(&[[0.0, 0.0], [1.0, 6000.0]]),
        facing_exponent: 1.0,
        trailing: curve(&[[0.0, 0.0], [1.0, 1200.0]]),
        trailing_exponent: 1.0,
        reference_length: 1.0,
        tangential: curve(&[[0.0, 1.0], [1.0, 500.0]]),
    };
    (tuning, fluid)
}

fn curve(keys: &[[f32; 2]]) -> Curve {
    Curve::new(0, keys.iter().map(|&[x, y]| Key { x, y }).collect())
}
