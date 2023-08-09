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

void RobotPath::split_long_segments(double max_segment_length) {

	for (size_t waypoint_i = 0; waypoint_i + 1 < waypoints.size(); ++waypoint_i) {

		const auto &start = waypoints[waypoint_i];
		const auto &end = waypoints[waypoint_i + 1];

		double distance = start.distance(end);

		if (distance > max_segment_length) {
			moveit::core::RobotState interpolated(start.getRobotModel());

			start.interpolate(end, 0.5, interpolated);

			interpolated.update();

			waypoints.insert(waypoints.begin() + (int) waypoint_i + 1, interpolated);

			waypoint_i -= 1; // TODO Inelegant and causes uneven spacing
		}

	}
}

void RobotPath::collapse_short_segments(double min_segment_length) {

	for (size_t waypoint_i = 0; waypoint_i + 1 < waypoints.size(); ++waypoint_i) {

		const auto &start = waypoints[waypoint_i];
		const auto &end = waypoints[waypoint_i + 1];

		double distance = start.distance(end);

		if (distance < min_segment_length) {
			moveit::core::RobotState interpolated(start.getRobotModel());

			start.interpolate(end, 0.5, interpolated);

			interpolated.update();

			waypoints.erase(waypoints.begin() + (int) waypoint_i + 1);
			waypoints[waypoint_i] = interpolated;

			waypoint_i -= 1; // TODO Inelegant and causes uneven spacing
		}

	}


}

void RobotPath::truncateUpTo(PathInterrupt interrupt) {

	// Ensure that the segment index is valid; i.e. that the path is long enough to be interrupted.
	assert(interrupt.segment_index + 1 < waypoints.size());

	// Get the start and end waypoints of the segment that is being interrupted.
	const auto &start = waypoints[interrupt.segment_index];
	auto &end = waypoints[interrupt.segment_index + 1];

	// Create a RobotState to hold the interpolated waypoint, and interpolate it.
	moveit::core::RobotState interpolated(start.getRobotModel());
	start.interpolate(end, interrupt.to_next_waypoint_interpolation, interpolated);

	// Assign the interpolated waypoint to the end waypoint.
	end = interpolated;

	// Erase all waypoints after the end waypoint, not including the end waypoint itself.
	waypoints.erase(waypoints.begin() + (long) interrupt.segment_index + 2, waypoints.end());

	assert(waypoints.size() == interrupt.segment_index + 2);

}

RobotPath omplPathToRobotPath(const ompl::geometric::PathGeometric &ompl_path) {

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

robot_trajectory::RobotTrajectory
robotPathToConstantSpeedRobotTrajectory(const RobotPath &robot_path, const double speed) {

	assert(!robot_path.waypoints.empty());

	// Create a RobotTrajectory.
	robot_trajectory::RobotTrajectory trajectory(robot_path.waypoints.front().getRobotModel());

	// Loop through the waypoints in the RobotPath.
	for (const auto &state: robot_path.waypoints) {

		// Distance from the previous waypoint (or 0.0 if this is the first waypoint).
		double distance = trajectory.getWayPointCount() == 0 ? 0.0 : state.distance(trajectory.getLastWayPoint());

		// Add the waypoint to the RobotTrajectory.
		trajectory.addSuffixWayPoint(state, distance / speed);

	}
	return std::move(trajectory);
}
