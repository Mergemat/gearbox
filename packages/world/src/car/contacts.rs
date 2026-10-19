use gearbox_engine::vehicle::{impacts::ImpactModel, post};
use super::CarAssets;

const DAMAGE_THRESHOLD: f32 = 0.8;
const DETACH_DELAY_MS: [u32; 2] = [20000, 40000];

impl CarAssets {
    pub fn impacts(&self) -> ImpactModel {
        self.impacts.clone()
    }

    pub fn post_model(&self) -> post::Model {
        let aux = &self.auxiliary;
        let shared = &aux.shared;
        post::Model {
            wheel_impact_range: [aux.visual.wheel_impact_min, aux.visual.wheel_impact_max],
            body_impact_range: [aux.visual.body_impact_min, aux.visual.body_impact_max],
            impact_delays: shared.impact_delays,
            spring_stiffness: shared.spring_stiffness, spring_damping: shared.spring_damping,
            spring_drive_limit: shared.spring_drive_limit, spring_drive_scale: shared.spring_drive_scale,
            spring_value_limit: shared.spring_value_limit,
            layer_build_by_speed: shared.wheel_response.clone(),
            layer_clear_by_speed: shared.steering_response.clone(),
            layer_materials: (0..80).map(|id| aux.groups.get(2)
                .is_some_and(|group| group.contains(&id))).collect(),
            damage_threshold: DAMAGE_THRESHOLD,
            detach_delay: DETACH_DELAY_MS,
        }
    }
}
