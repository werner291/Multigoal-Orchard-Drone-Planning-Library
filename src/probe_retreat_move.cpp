
#include <ompl/geometric/PathSimplifier.h>
#include "probe_retreat_move.h"


moveit::core::RobotState
state_outside_tree(const moveit::core::RobotModelPtr &drone, const Apple &a, const Eigen::Vector3d &sphere_center,
                   double sphere_radius) {
    moveit::core::RobotState st(drone);

    Eigen::Vector3d default_facing(0.0, 1.0, 0.0);
    Eigen::Vector3d required_facing = (sphere_center - a.center).normalized();
    Eigen::Vector3d base_facing = (Eigen::Vector3d(required_facing.x(), required_facing.y(), 0.0)).normalized();

    double yaw = copysign(acos(default_facing.dot(base_facing)), default_facing.cross(base_facing).z());

    Eigen::Quaterniond qd(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    st.setVariablePositions({
                                    0.0, 0.0, 0.0,      // Position off the side of the tree
                                    qd.x(), qd.y(), qd.z(), qd.w(),// Identity rotation
                                    0.0, 0.0, 0.0, 0.0  // Arm straight out
                            });

    st.update(true);

    Eigen::Vector3d apple_on_sphere = (a.center - sphere_center).normalized() * sphere_radius + sphere_center;
    if (apple_on_sphere.z() < 0.5) apple_on_sphere.z() = 0.5;

    Eigen::Vector3d offset = (apple_on_sphere) - st.getGlobalLinkTransform("end_effector").translation();

    st.setVariablePosition(0, offset.x());
    st.setVariablePosition(1, offset.y());
    st.setVariablePosition(2, offset.z());

    st.update(true);

    return st;
}

std::vector<moveit::core::RobotState>
sphericalInterpolatedPath(const moveit::core::RobotState &ra, const moveit::core::RobotState &rb,
                          const Eigen::Vector3d &sphere_center) {

    Eigen::Vector3d ra_ray = (ra.getGlobalLinkTransform("base_link").translation() - sphere_center);
    Eigen::Vector3d rb_ray = (rb.getGlobalLinkTransform("base_link").translation() - sphere_center);

    Eigen::Vector3d normal = ra_ray.cross(rb_ray).normalized();
    double angle = acos(ra_ray.dot(rb_ray) / (ra_ray.norm() * rb_ray.norm()));
    const auto num_states = (size_t) (2.0 * angle);

    std::vector<moveit::core::RobotState> path;
    path.push_back(ra);

    for (size_t state_i = 0; state_i < num_states; state_i++) {

        double t = (double) (state_i+1) / (double) (num_states+2);

        moveit::core::RobotState ri(ra.getRobotModel());
        ra.interpolate(rb, t, ri);

        Eigen::Vector3d base_center = sphere_center + Eigen::AngleAxisd(angle * t, normal) * ra_ray;
        ri.setVariablePosition(0, base_center.x());
        ri.setVariablePosition(1, base_center.y());
        ri.setVariablePosition(2, std::max(base_center.z(),0.5));
        ri.update(true);

        path.push_back(ri);
    }

    path.push_back(rb);

    return path;
}
