
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
