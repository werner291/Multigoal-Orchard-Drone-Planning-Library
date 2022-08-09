#include "robot_path.h"

#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/transform.hpp>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include "utilities/general_utilities.h"

double RobotPath::length() const {
    return ranges::accumulate(pairwise(waypoints) | ranges::views::transform([&](auto pair) {
        return pair.first.distance(pair.second);
    }), 0.0);
}

void RobotPath::append(const RobotPath &other) {
    waypoints.insert(waypoints.end(), other.waypoints.begin(), other.waypoints.end());
}

RobotPath omplPathToRobotPath(const ompl::geometric::PathGeometric& ompl_path) {
    RobotPath path;

    auto state_space = ompl_path.getSpaceInformation()->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>();

    path.waypoints.reserve(ompl_path.getStateCount());

    for (size_t i = 0; i < ompl_path.getStateCount(); ++i) {
        auto state = ompl_path.getState(i);

        moveit::core::RobotState rs(state_space->getRobotModel());
        state_space->copyToRobotState(rs, state);
        rs.update(true);

        path.waypoints.push_back(rs);
    }

    return path;
}