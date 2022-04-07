
#include <ompl/base/ScopedState.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <boost/range/irange.hpp>
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

robot_trajectory::RobotTrajectory
omplPathToRobotTrajectory(const ompl_interface::ModelBasedStateSpace &state_space,
                          const ompl::geometric::PathGeometric &result_path) {

    robot_trajectory::RobotTrajectory traj(state_space.getRobotModel());

    double t = 0.0;
    for (size_t i : boost::irange<size_t>(0,result_path.getStateCount())) {
        moveit::core::RobotState moveit_state(state_space.getRobotModel());
        state_space.copyToRobotState(moveit_state, result_path.getState(i));
        traj.addSuffixWayPoint(moveit_state, t);
        t += 0.1;
    }

    return traj;
}
