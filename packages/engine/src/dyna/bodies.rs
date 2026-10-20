use super::{BodyIndex, BodyInertia, BodyState, Collision, Dyna, DynaBody, DynaModel, ForceAcc};
use super::{
    BodyOptions,
    constants::{ACTIVE_HANDLE, NULL_INDEX, SOLVER_EPSILON},
};
use crate::{DynamicItemId, Iso, Mat3, Vec3};

const HANDLE_RECORD_BYTES: u32 = 12;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BodyHandle(pub u32);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ForceModels {
    #[default]
    None,
    Vehicle,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Handles {
    pub slots: Vec<u32>,
    pub reverse: Vec<u32>,
    pub next: Vec<u32>,
    pub head: u32,
    pub used: u32,
}

impl Default for Handles {
    fn default() -> Self {
        Self { slots: Vec::new(), reverse: Vec::new(), next: Vec::new(), head: NULL_INDEX, used: 0 }
    }
}

impl Handles {
    pub fn slot(&self, handle: BodyHandle) -> u32 {
        assert_eq!(self.next[handle.0 as usize], ACTIVE_HANDLE, "inactive body handle");
        self.slots[handle.0 as usize]
    }

    pub fn free(&mut self, handle: BodyHandle) {
        let h = handle.0 as usize;
        let slot = self.slots[h];
        let last = self.used - 1;
        if slot < last {
            let moved = self.reverse[last as usize];
            self.slots[h] = last;
            self.reverse[slot as usize] = moved;
            self.slots[moved as usize] = slot;
            self.reverse[last as usize] = handle.0;
        }
        self.next[h] = self.head;
        self.head = handle.0;
        self.used -= 1;
    }

    pub fn take(&mut self) -> BodyHandle {
        if self.head == NULL_INDEX {
            let old = self.slots.len() as u32;
            let capacity = grow_capacity(old, old + 1, HANDLE_RECORD_BYTES);
            for i in old..capacity {
                self.slots.push(i);
                self.reverse.push(i);
                self.next.push(if i + 1 < capacity { i + 1 } else { NULL_INDEX });
            }
            self.head = old;
        }
        let h = self.head;
        self.head = self.next[h as usize];
        self.next[h as usize] = ACTIVE_HANDLE;
        self.used += 1;
        BodyHandle(h)
    }
}

pub fn grow_capacity(current: u32, wanted: u32, element_size: u32) -> u32 {
    let bytes = current.wrapping_mul(element_size);
    let step = if bytes < 0x100_0001 {
        if bytes > 0x40_0000 { current >> 2 } else { current >> 1 }
    } else {
        current >> 3
    };
    if (wanted.wrapping_sub(current) as i32) <= step as i32 { current.wrapping_add(step) } else { wanted }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Body {
    pub model: DynaModel,
    pub record: DynaBody,
    pub flags: BodyOptions,
    pub item: Option<DynamicItemId>,
    pub owner_item: Option<DynamicItemId>,
    pub state: BodyState,
    pub inertia: BodyInertia,
    pub index: BodyIndex,
    pub acc: ForceAcc,
    pub forces: ForceModels,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyParams {
    pub model: DynaModel,
    pub item: Option<DynamicItemId>,
    pub owner_item: Option<DynamicItemId>,
    pub has_pre_collision_callback: bool,
    pub forces: ForceModels,
    pub fixed: bool,
    pub kinematic: bool,
    pub special: bool,
    pub parent: Option<BodyHandle>,
}

impl DynaModel {
    pub fn is_externally_simulated(&self) -> bool {
        self.externally_simulated != 0
    }

    pub fn center(&self) -> Vec3 {
        Vec3::new(self.center_x, self.center_y, self.center_z)
    }

    pub fn inverse_inertia(&self) -> Vec3 {
        let matrix = self.inverse_inertia_matrix;
        Vec3::new(matrix.rows[0].x, matrix.rows[1].y, matrix.rows[2].z)
    }
}

impl BodyState {
    pub fn pose(&self) -> Iso {
        Iso::new(self.rot, self.pos)
    }

    pub fn set_pose(&mut self, pose: Iso) {
        self.quat = crate::Quat::from_mat3(&pose.rot);
        self.rot = pose.rot;
        self.pos = pose.pos;
    }

    pub fn kinematic(current: Iso, past: Iso, dt: f32) -> Self {
        let a = crate::Quat::from_mat3(&past.rot);
        let b = crate::Quat::from_mat3(&current.rot);
        let inv = 1.0 / dt;
        Self {
            rot: current.rot,
            pos: current.pos,
            quat: b,
            vel: Vec3::new(
                inv * (current.pos.x - past.pos.x),
                inv * (current.pos.y - past.pos.y),
                inv * (current.pos.z - past.pos.z),
            ),
            ang_vel: a.angular_velocity_to(b, dt),
        }
    }

    pub fn point_velocity(&self, p: Vec3) -> Vec3 {
        let r = p - self.pos;
        let w = self.ang_vel;
        Vec3::new(
            (w.y * r.z - w.z * r.y) + self.vel.x,
            (w.z * r.x - w.x * r.z) + self.vel.y,
            (w.x * r.y - w.y * r.x) + self.vel.z,
        )
    }

    pub fn clamp_angular_velocity(&mut self, max: f32) {
        if SOLVER_EPSILON < max {
            let w = self.ang_vel;
            let len2 = w.x * w.x + w.y * w.y + w.z * w.z;
            if max * max < len2 {
                let k = max / len2.sqrt();
                self.ang_vel = Vec3::new(w.x * k, k * w.y, k * w.z);
            }
        }
    }
}

impl Body {
    pub fn center_of_mass(&self) -> Vec3 {
        self.state.pose().transform_y_first(self.model.center())
    }

    pub fn velocity_at(&self, point: Vec3) -> Vec3 {
        let r = point - self.center_of_mass();
        let w = self.state.ang_vel;
        Vec3::new(
            (r.z * w.y - r.y * w.z) + self.state.vel.x,
            (r.x * w.z - r.z * w.x) + self.state.vel.y,
            (r.y * w.x - r.x * w.y) + self.state.vel.z,
        )
    }

    pub fn place_at_center(&mut self, negative: bool) {
        let mut center = self.model.center();
        if negative {
            center = -center;
        }
        self.state.pos = self.state.pose().transform_y_first(center);
    }

    pub fn wake(&self) {
        if self.index.index == u16::MAX && self.index.wake_disabled != 1 {
            todo!("waking a sleeping body");
        }
    }
}

impl Dyna {
    pub fn register(&mut self, params: BodyParams, pose: Iso, collision: &mut impl Collision) -> BodyHandle {
        let handle = self.handles.take();
        let mut state = BodyState::default();
        state.set_pose(pose);
        let record = DynaBody {
            coefficient: self.coefficient,
            has_pre_collision_callback: params.has_pre_collision_callback,
            parent: params.parent.map_or(NULL_INDEX, |h| h.0),
            ..DynaBody::default()
        };
        if let Some(parent) = params.parent {
            let slot = self.handles.slot(parent);
            self.bodies[slot as usize].record.parent = handle.0;
        }
        let index = self.active.len() as u16;
        let slot = self.bodies.len() as u32;
        self.active.push(slot);
        self.dynamic_registered |= params.special;
        for item in [params.item, params.owner_item].into_iter().flatten() {
            collision.set_body(item, handle);
        }
        let mut flags = BodyOptions::default();
        flags.set(BodyOptions::FIXED, params.fixed);
        flags.set(BodyOptions::SPECIAL, params.special);
        flags.set(BodyOptions::KINEMATIC, params.kinematic);
        self.bodies.push(Body {
            inertia: BodyInertia { diagonal: params.model.inverse_inertia(), inverse_mass: 0.0 },
            model: params.model,
            record,
            state,
            item: params.item,
            owner_item: params.owner_item,
            flags,
            index: BodyIndex { rest_since: NULL_INDEX, index, wake_disabled: 0 },
            forces: params.forces,
            acc: ForceAcc::default(),
        });
        handle
    }

    pub fn remove(&mut self, handle: BodyHandle, collision: &mut impl Collision) {
        self.pending_removals.push(handle);
        let slot = self.handles.slot(handle);
        super::solver::islands::deactivate(&mut self.bodies, &mut self.active, &[slot], false);
        let body = &mut self.bodies[slot as usize];
        for item in [body.item.take(), body.owner_item.take()].into_iter().flatten() {
            collision.set_body(item, BodyHandle(NULL_INDEX));
        }
        let parent = std::mem::replace(&mut body.record.parent, NULL_INDEX);
        body.flags.set(BodyOptions::NO_STATIC_COLLISION, true);
        if parent != NULL_INDEX {
            let slot = self.handles.slot(BodyHandle(parent));
            self.bodies[slot as usize].record.parent = NULL_INDEX;
        }
    }

    pub(crate) fn process_removals(&mut self) {
        for handle in std::mem::take(&mut self.pending_removals) {
            let slot = self.handles.slot(handle) as usize;
            let last = self.bodies.len() - 1;
            if slot < last {
                let index = self.bodies[last].index.index;
                if index != u16::MAX {
                    self.active[index as usize] = slot as u32;
                }
            }
            self.bodies.swap_remove(slot);
            self.handles.free(handle);
        }
    }

    pub fn body(&self, h: BodyHandle) -> &Body {
        &self.bodies[self.handles.slot(h) as usize]
    }
    pub fn body_mut(&mut self, h: BodyHandle) -> &mut Body {
        let slot = self.handles.slot(h);
        &mut self.bodies[slot as usize]
    }

    pub fn state(&self, h: BodyHandle) -> &BodyState {
        &self.body(h).state
    }

    pub fn add_force_at(&mut self, h: BodyHandle, f: Vec3, point: Vec3) {
        let b = self.body_mut(h);
        let r = point - b.center_of_mass();
        let a = &mut b.acc;
        a.force = Vec3::new(f.x + a.force.x, f.y + a.force.y, a.force.z + f.z);
        a.torque.x += r.y * f.z - r.z * f.y;
        a.torque.z += r.x * f.y - r.y * f.x;
        a.torque.y += r.z * f.x - r.x * f.z;
        b.wake();
    }

    pub fn add_force(&mut self, h: BodyHandle, force: Vec3) {
        let b = self.body_mut(h);
        b.acc.force += force;
        b.wake();
    }

    pub fn local_velocity(&self, h: BodyHandle) -> Vec3 {
        local(self.state(h).rot, self.state(h).vel)
    }

    pub fn local_angular_velocity(&self, h: BodyHandle) -> Vec3 {
        local(self.state(h).rot, self.state(h).ang_vel)
    }

    pub fn local_force(&self, h: BodyHandle) -> Vec3 {
        let b = self.body(h);
        local(b.state.rot, b.acc.force)
    }

    pub fn add_local_force(&mut self, h: BodyHandle, force: Vec3) {
        let force = self.state(h).rot * force;
        self.add_force(h, force);
    }

    pub fn add_local_force_at(&mut self, h: BodyHandle, force: Vec3, point: Vec3) {
        let state = self.state(h);
        let force = state.rot * force;
        let point = state.pose().transform_point(point);
        self.add_force_at(h, force, point);
    }

    pub fn add_local_torque(&mut self, h: BodyHandle, torque: Vec3) {
        let b = self.body_mut(h);
        b.acc.torque += b.state.rot.transform_y_first(torque);
    }

    pub fn set_local_torque(&mut self, h: BodyHandle, torque: Vec3) {
        let b = self.body_mut(h);
        b.acc.torque = b.state.rot.transform_y_first(torque);
    }

    pub fn set_local_force(&mut self, h: BodyHandle, force: Vec3) {
        let b = self.body_mut(h);
        b.acc.force = b.state.rot.transform_y_first(force);
        b.wake();
    }

    pub fn set_local_velocity(&mut self, h: BodyHandle, v: Vec3) {
        let b = self.body_mut(h);
        b.state.vel = b.state.rot.transform_y_first(v);
        b.wake();
    }

    pub fn set_local_angular_velocity(&mut self, h: BodyHandle, w: Vec3) {
        let b = self.body_mut(h);
        b.state.ang_vel = b.state.rot.transform_y_first(w);
        b.wake();
    }

    pub fn clear_acc(&mut self, h: BodyHandle) {
        self.body_mut(h).acc = ForceAcc::default();
    }

    pub fn is_dynamic_body(&self, h: BodyHandle) -> bool {
        let b = self.body(h);
        b.model.is_externally_simulated() && !b.flags.contains(BodyOptions::KINEMATIC)
    }

    pub fn set_pose(&mut self, h: BodyHandle, pose: Iso) {
        let b = self.body_mut(h);
        b.state.set_pose(pose);
        b.inertia.diagonal = b.model.inverse_inertia();
        b.wake();
    }

    pub fn set_state(&mut self, h: BodyHandle, state: BodyState) {
        let b = self.body_mut(h);
        b.state = state;
        b.wake();
    }

    pub fn set_velocity(&mut self, h: BodyHandle, velocity: Vec3) {
        let b = self.body_mut(h);
        b.state.vel = velocity;
        b.wake();
    }

    pub fn add_velocity(&mut self, h: BodyHandle, velocity: Vec3) {
        let b = self.body_mut(h);
        b.state.vel = add_velocity(b.state.vel, velocity);
        b.wake();
    }

    pub fn add_angular_velocity_unclamped(&mut self, h: BodyHandle, velocity: Vec3) {
        let b = self.body_mut(h);
        b.state.ang_vel = add_velocity(b.state.ang_vel, velocity);
        b.wake();
    }

    pub fn add_angular_velocity(&mut self, h: BodyHandle, velocity: Vec3) {
        let b = self.body_mut(h);
        b.state.ang_vel = add_velocity(b.state.ang_vel, velocity);
        b.state.clamp_angular_velocity(b.model.max_angular_velocity);
    }

    pub fn add_position(&mut self, h: BodyHandle, offset: Vec3) {
        let b = self.body_mut(h);
        b.state.pos = add_velocity(b.state.pos, offset);
        b.wake();
    }

    pub fn set_diagonal(&mut self, h: BodyHandle, matrix: Mat3) {
        self.body_mut(h).inertia.diagonal = Vec3::new(matrix.rows[0].x, matrix.rows[1].y, matrix.rows[2].z);
    }

    pub fn add_torque(&mut self, h: BodyHandle, torque: Vec3) {
        let b = self.body_mut(h);
        b.acc.torque = add_velocity(b.acc.torque, torque);
        b.wake();
    }

    pub fn local_point_velocity(&self, h: BodyHandle, p: Vec3) -> Vec3 {
        let b = self.body(h);
        let [a, c, d] = b.state.rot.rows;
        let t = b.state.pos;
        let world = Vec3::new(
            p.x * a.x + p.y * a.y + p.z * a.z + t.x,
            p.y * c.y + p.x * c.x + p.z * c.z + t.y,
            p.y * d.y + p.x * d.x + p.z * d.z + t.z,
        );
        let v = b.velocity_at(world);
        Vec3::new(
            v.x * a.x + v.y * c.x + v.z * d.x,
            v.x * a.y + v.y * c.y + v.z * d.y,
            v.y * c.z + v.x * a.z + v.z * d.z,
        )
    }

    pub fn contact_time_ms(
        &self,
        h: BodyHandle,
        now: i64,
        max: f32,
        point: Vec3,
        normal: Vec3,
        other: Iso,
    ) -> i64 {
        let b = self.body(h);
        let v = if self.centered { b.state.point_velocity(point) } else { b.velocity_at(point) };
        let local_velocity = other.rot.transpose_mul_vec(-v);
        let local_normal = other.rot.transpose_mul_vec(-normal);
        super::solver::closest_time_ms(now, max, local_normal, local_velocity)
    }

    pub fn apply_impulse(
        &mut self,
        h: BodyHandle,
        impulse: Vec3,
        point: Vec3,
        mass: f32,
        inverse_inertia: Mat3,
    ) {
        let b = self.body_mut(h);
        let inv = 1.0 / mass;
        let dv = Vec3::new(inv * impulse.x, inv * impulse.y, inv * impulse.z);
        b.state.vel = add_velocity(b.state.vel, dv);
        b.wake();
        let r = point - b.center_of_mass();
        let t = Vec3::new(
            r.y * impulse.z - r.z * impulse.y,
            r.z * impulse.x - r.x * impulse.z,
            r.x * impulse.y - r.y * impulse.x,
        );
        let k = inverse_inertia.rows;
        let w = Vec3::new(t.dot_yxz(k[0]), t.dot_yxz(k[1]), t.dot_yxz(k[2]));
        b.state.ang_vel = add_velocity(b.state.ang_vel, w);
        b.wake();
    }
}

fn local(r: Mat3, v: Vec3) -> Vec3 {
    let [a, b, c] = r.rows;
    Vec3::new(
        b.x * v.y + v.x * a.x + c.x * v.z,
        a.y * v.x + b.y * v.y + c.y * v.z,
        a.z * v.x + b.z * v.y + c.z * v.z,
    )
}

fn add_velocity(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x + b.x, b.y + a.y, b.z + a.z)
}
