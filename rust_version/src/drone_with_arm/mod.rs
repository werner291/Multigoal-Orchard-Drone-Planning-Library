use parry3d::na::Isometry3;

struct DroneState {
    base_pose: Isometry3<f32>,
    arm_angles: [f32; 3],
}