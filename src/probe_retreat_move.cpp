
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
        auto probing_path = plan_state_to_apple(full_path.getStates().back(), apple);

        if (to_approach.has_value() && probing_path.has_value()) {
            full_path.append(*to_approach);
            full_path.append(*probing_path);
            probing_path->reverse();
            full_path.append(*probing_path);
        }
    }

    return full_path;

}

moveit::core::RobotState
state_outside_tree(const moveit::core::RobotModelPtr& drone, const Apple &a, const Eigen::Vector3d &sphere_center) {
    moveit::core::RobotState st(drone);

    Eigen::Vector3d default_facing(0.0, 1.0, 0.0);
    Eigen::Vector3d required_facing = (a.center - sphere_center).normalized();
    Eigen::Vector3d base_facing = (Eigen::Vector3d(required_facing.x(), required_facing.y(), 0.0)).normalized();

    double yaw = copysign(acos(default_facing.dot(base_facing)), default_facing.cross(base_facing).z());

    Eigen::Quaterniond qd(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    st.setVariablePositions({
                                    0.0, 0.0, 0.0,      // Position off the side of the tree
                                    qd.x(), qd.y(), qd.z(), qd.w(),// Identity rotation
                                    0.0, 0.0, 0.0, 0.0  // Arm straight out
                            });

    st.update(true);

    Eigen::Vector3d apple_on_sphere = (a.center - sphere_center).normalized() * 4.0 + sphere_center;
    if (apple_on_sphere.z() < 0.5) apple_on_sphere.z() = 0.5;

    Eigen::Vector3d offset = (apple_on_sphere) - st.getGlobalLinkTransform("end_effector").translation();

    st.setVariablePosition(0, offset.x());
    st.setVariablePosition(1, offset.y());
    st.setVariablePosition(2, offset.z());

    st.update(true);

    return st;
}
