// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include "goal_events.h"
#include "moveit.h"
#include "../RobotPath.h"

namespace utilities {

	bool within_distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double distance) {
		return (a - b).norm() < distance;
	}

	std::optional<GoalSighting> find_earliest_discovery_event(RobotPath &traj,
															  const std::vector<Apple> &apples,
															  double discovery_max_distance,
															  const std::vector<DiscoveryStatus> &discovery_status) {

		for (size_t segment_i = 0; segment_i < traj.waypoints.size() - 1; segment_i += 1) {

			const moveit::core::RobotState &before = traj.waypoints[segment_i];
			const moveit::core::RobotState &after = traj.waypoints[segment_i + 1];

			moveit::core::RobotState state(before.getRobotModel());

			int num_steps = (int) (before.distance(after) / 0.1) + 1;

			for (int step_i = 0; step_i < num_steps; step_i += 1) {

				double t = (double) step_i / (double) num_steps;

				before.interpolate(after, t, state);

				Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

				for (const auto &[goal_id, apple]: ranges::views::enumerate(apples)) {
					if (within_distance(ee_pos, apple.center, discovery_max_distance) &&
						discovery_status[goal_id] != EXISTS_BUT_UNKNOWN_TO_ROBOT) {
						return GoalSighting{(int) goal_id, {segment_i, t}};
					}
				}

			}

		}

		return std::nullopt;
	}
}