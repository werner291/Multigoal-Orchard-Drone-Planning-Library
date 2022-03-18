#include "SphereShell.h"

#include <utility>

SphereShell::SphereShell(Eigen::Vector3d center, double radius) : center(std::move(center)), radius(radius) {

}

moveit::core::RobotState SphereShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) const {
    moveit::core::RobotState st(drone);

    Eigen::Vector3d default_facing(0.0, 1.0, 0.0);
    Eigen::Vector3d required_facing = (center - a.center).normalized();
    Eigen::Vector3d base_facing = (Eigen::Vector3d(required_facing.x(), required_facing.y(), 0.0)).normalized();

    double yaw = copysign(acos(default_facing.dot(base_facing)), default_facing.cross(base_facing).z());

    Eigen::Quaterniond qd(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    st.setVariablePositions({
                                    0.0, 0.0, 0.0,      // Position off the side of the tree
                                    qd.x(), qd.y(), qd.z(), qd.w(),// Identity rotation
                                    0.0, 0.0, 0.0, 0.0  // Arm straight out
                            });

    st.update(true);

    Eigen::Vector3d apple_on_sphere = (a.center - center).normalized() * radius + center;
    if (apple_on_sphere.z() < 0.5) apple_on_sphere.z() = 0.5;

    Eigen::Vector3d offset = (apple_on_sphere) - st.getGlobalLinkTransform("end_effector").translation();

    st.setVariablePosition(0, offset.x());
    st.setVariablePosition(1, offset.y());
    st.setVariablePosition(2, offset.z());

    st.update(true);

    return st;
}

std::vector<moveit::core::RobotState> SphereShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) const {

    Eigen::Vector3d ra_ray = a.center - center;
    Eigen::Vector3d rb_ray = b.center - center;

    Eigen::Vector3d normal = ra_ray.cross(rb_ray).normalized();
    double angle = acos(ra_ray.dot(rb_ray) / (ra_ray.norm() * rb_ray.norm()));
    const auto num_states = (size_t) (2.0 * angle);

    std::vector<moveit::core::RobotState> path;
    path.reserve(num_states + 1);

    for (size_t state_i = 0; state_i <= num_states; state_i++) {

        double t = (double) (state_i) / (double) (num_states);

        Eigen::Vector3d base_center = center + Eigen::AngleAxisd(angle * t, normal) * ra_ray;

        path.push_back(this->state_on_shell(drone, {base_center, {0.0, 0.0, 0.0}}));
    }

    return path;

}

