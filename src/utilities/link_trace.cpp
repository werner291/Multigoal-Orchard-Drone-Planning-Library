// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-5-23.
//

#include "link_trace.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

std::vector<Eigen::Vector3d> computeLinkTrace(const ompl_interface::ModelBasedStateSpace &ss,
											  const moveit::core::RobotModelPtr &robot,
											  const MultiGoalPlanner::PlanResult &path,
											  const std::string &link_name) {
	std::vector<Eigen::Vector3d> trace;

	// Loop over all segments in the path.
	for (const auto &segment: path.segments) {
		// For each segment, loop over all states.
		for (size_t state_i = 0; state_i < segment.path_.getStateCount(); ++state_i) {

			// Get the current state from the path segment.
			auto state = segment.path_.getState(state_i);

			// Create a RobotState object to hold the robot's state.
			moveit::core::RobotState robot_state(robot);

			// Copy the OMPL state to the MoveIt RobotState.
			ss.copyToRobotState(robot_state, state);

			// Get the global transform of the specified link and add its translation part to the trace.
			trace.emplace_back(robot_state.getGlobalLinkTransform(link_name).translation());
		}
	}

	return trace;
}

std::vector<Eigen::Vector3d> computeLinkTrace(const RobotPath &path, const std::string &link_name) {
	std::vector<Eigen::Vector3d> trace;

	for (const auto &state: path.waypoints) {
		trace.emplace_back(state.getGlobalLinkTransform(link_name).translation());
	}

	return trace;
}
