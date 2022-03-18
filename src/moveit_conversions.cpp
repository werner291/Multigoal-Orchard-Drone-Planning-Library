
#include <ompl/base/ScopedState.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include "moveit_conversions.h"

ompl::geometric::PathGeometric omplPathFromMoveitTrajectory(const std::vector<moveit::core::RobotState> &trajectory,
                                                            const ompl::base::SpaceInformationPtr &si) {
    ompl::geometric::PathGeometric result(si);
    
    ompl::base::ScopedState ss(si);
    
    for (const auto &item : trajectory) {
        const auto *state_space = si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>();
        state_space->copyToOMPLState(ss.get(), item);
        result.append(ss.get());
    }

    return result;
}

robot_trajectory::RobotTrajectory omplPathToRobotTrajectory(const moveit::core::RobotModelPtr &drone,
                                                            const std::shared_ptr<ompl_interface::ModelBasedStateSpace> &state_space,
                                                            ompl::geometric::PathGeometric &result_path) {

    robot_trajectory::RobotTrajectory traj(drone);

    double t = 0.0;
    for (const auto state : result_path.getStates()) {
        moveit::core::RobotState moveit_state(drone);
        state_space->copyToRobotState(moveit_state, state);
        traj.addSuffixWayPoint(moveit_state, t);
        t += 0.1;
    }

    return traj;
}
