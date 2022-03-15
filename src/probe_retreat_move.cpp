
#include "probe_retreat_move.h"


ompl::geometric::PathGeometric plan_probe_retreat_slide(const std::vector<Apple>& apples_in_order,
                                                        const ompl::base::State* initial_state,
                                                        const ompl::base::SpaceInformationPtr& si,
                                                        const std::function<void(const Apple& apple, ompl::base::State*)>& state_outside_tree,
                                                        const std::function<std::optional<ompl::geometric::PathGeometric>(ompl::base::State*, ompl::base::State*)>& plan_state_to_state,
                                                        const std::function<std::optional<ompl::geometric::PathGeometric>(ompl::base::State*, const Apple& apple)>& plan_state_to_apple) {

    ompl::geometric::PathGeometric full_path(si, initial_state);

    for (const Apple& apple : apples_in_order) {

        ompl::base::ScopedState state_outside_tree_for_apple(si);
        state_outside_tree(apple, state_outside_tree_for_apple.get());

        const auto to_approach = plan_state_to_state(full_path.getStates().back(), state_outside_tree_for_apple.get());
        full_path.append(*to_approach);

        auto probing_path = plan_state_to_apple(full_path.getStates().back(), apple);

        if (to_approach.has_value() && probing_path.has_value()) {
            full_path.append(*probing_path);
            probing_path->reverse();
            full_path.append(*probing_path);
            assert(si->distance(state_outside_tree_for_apple.get(), probing_path->getStates().back()) < 0.01);
        }

    }

    return full_path;

}

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


    const size_t num_states = 20;

    std::vector<moveit::core::RobotState> path;
    path.push_back(ra);

    for (size_t state_i = 1; state_i < num_states; state_i++) {

        double t = (double) state_i / (double) num_states;

        moveit::core::RobotState ri(ra.getRobotModel());
        ra.interpolate(rb, t, ri);

        Eigen::Vector3d base_center = sphere_center + Eigen::AngleAxisd(angle * t, normal) * ra_ray;
        ri.setVariablePosition(0, base_center.x());
        ri.setVariablePosition(1, base_center.y());
        ri.setVariablePosition(2, base_center.z());
        ri.update(true);

        path.push_back(ri);
    }

    path.push_back(rb);

    return path;
}
