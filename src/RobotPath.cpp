#include "RobotPath.h"

#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/transform.hpp>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include "utilities/general_utilities.h"

double RobotPath::length() const {
	// Sum up the distances between the waypoints.
    return ranges::accumulate(pairwise(waypoints) | ranges::views::transform([&](auto pair) {
        return pair.first.distance(pair.second);
    }), 0.0);
}

void RobotPath::append(const RobotPath &other) {
	// Append the other path to this one by adding the waypoints.
    waypoints.insert(waypoints.end(), other.waypoints.begin(), other.waypoints.end());
}

RobotPath omplPathToRobotPath(const ompl::geometric::PathGeometric& ompl_path) {

	// Create a RobotPath.
    RobotPath path;

	// Get the state space and convert it to a ModelBasedStateSpace.
    auto state_space = ompl_path.getSpaceInformation()->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>();

	// Get the number of waypoints in the OMPL path and reserve a vector of that size.
    path.waypoints.reserve(ompl_path.getStateCount());

	// Loop through the waypoints in the OMPL path.
    for (size_t i = 0; i < ompl_path.getStateCount(); ++i) {

		// Get the ith waypoint.
        auto state = ompl_path.getState(i);

		// Create a RobotState.
        moveit::core::RobotState rs(state_space->getRobotModel());
		// Convert the OMPL state to a RobotState.
        state_space->copyToRobotState(rs, state);
		// Update the FK.
        rs.update(true);

		// Add the RobotState to the RobotPath.
        path.waypoints.push_back(rs);
    }

	// Return the RobotPath.
    return path;
}

ompl::geometric::PathGeometric robotPathToOmplPath(const RobotPath &robot_path, const ompl::base::SpaceInformationPtr& si) {

	// Create an OMPL path.
	ompl::geometric::PathGeometric ompl_path(si);

	// Get the state space and convert it to a ModelBasedStateSpace.
	ompl::base::ScopedState<> state(si);

	// Loop through the waypoints in the RobotPath.
	for (const auto& moveit_state : robot_path.waypoints) {
		// Convert the RobotState to an OMPL state.
		si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToOMPLState(state.get(), moveit_state);
		// Add the OMPL state to the OMPL path.
		ompl_path.append(state.get());
	}

	return ompl_path;

}
