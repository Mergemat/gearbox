use gearbox_search::pose::{Pose, PoseGoal, rotation_distance};

fn target() -> PoseGoal {
    PoseGoal {
        pose: Pose { position: [0.0; 3], rotation: [1.0, 0.0, 0.0, 0.0], velocity: [0.0; 3] },
        position_tolerance: [1.0, 2.0, 3.0],
        angle_tolerance: 0.1,
        velocity_tolerance: Some([0.5; 3]),
        position_weights: [1.0, 2.0, 3.0],
        angle_weight: 4.0,
        velocity_weights: [5.0; 3],
    }
}
#[test]
fn rotation_uses_shortest_so3_distance_including_quaternion_double_cover() {
    let identity = [1.0, 0.0, 0.0, 0.0];
    let ninety = [std::f32::consts::FRAC_1_SQRT_2, 0.0, std::f32::consts::FRAC_1_SQRT_2, 0.0];
    assert!((rotation_distance(identity, ninety) - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
    assert!((rotation_distance(identity, [0.0, 1.0, 0.0, 0.0]) - std::f64::consts::PI).abs() < 1e-9);
    assert!(rotation_distance(ninety, ninety.map(|v| -v)) < 1e-7);
    let h = 175f32.to_radians();
    assert!((rotation_distance(identity, [h.cos(), 0.0, h.sin(), 0.0]) - 10f64.to_radians()).abs() < 1e-6);
    assert!(rotation_distance(identity, [0.0; 4]).is_infinite());
}
#[test]
fn pose_tolerances_do_not_disappear_when_their_weights_are_zero() {
    let mut g = target();
    g.position_weights = [0.0; 3];
    g.velocity_weights = [0.0; 3];
    let mut p = g.pose;
    p.position = [1.01, 0.0, 0.0];
    assert!(!g.contains(p));
    p.position = [0.0; 3];
    p.velocity = [0.6, 0.0, 0.0];
    assert!(!g.contains(p));
    g.velocity_tolerance = None;
    assert!(g.contains(p));
    p.rotation = [0.0, 0.0, 1.0, 0.0];
    assert!(!g.contains(p));
}
#[test]
fn weighted_metric_matches_independently_computed_axis_and_angle_errors() {
    let g = target();
    let p = Pose { position: [2.0, 3.0, 4.0], rotation: [0.0, 0.0, 1.0, 0.0], velocity: [1.0, 2.0, 0.0] };
    let expected = 4.0 + 18.0 + 48.0 + 4.0 * std::f64::consts::PI.powi(2) + 25.0;
    assert!((g.error(p) - expected).abs() < 1e-9);
    let mut invalid = g;
    invalid.pose.rotation = [0.0; 4];
    assert!(invalid.validate().is_err());
}
