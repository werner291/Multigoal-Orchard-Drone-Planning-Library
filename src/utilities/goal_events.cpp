// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/algorithm/count.hpp>
#include "goal_events.h"
#include "moveit.h"
#include "../RobotPath.h"

namespace utilities {

	bool within_distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double distance) {
		return (a - b).norm() < distance;
	}

	std::optional<GoalSighting> find_earliest_discovery_event(RobotPath &traj,
															  const std::vector<Apple> &apples,
															  CanSeeAppleFn can_see_apple,
															  const std::vector<DiscoveryStatus> &discovery_status) {

		for (size_t segment_i = 0; segment_i < traj.waypoints.size() - 1; segment_i += 1) {

			const moveit::core::RobotState &before = traj.waypoints[segment_i];
			const moveit::core::RobotState &after = traj.waypoints[segment_i + 1];

			moveit::core::RobotState state(before.getRobotModel());

			int num_steps = (int) (before.distance(after) / 0.1) + 1;

			for (int step_i = 0; step_i < num_steps; step_i += 1) {

				double t = (double) step_i / (double) num_steps;

				before.interpolate(after, t, state);

				state.update(true);

				for (const auto &[goal_id, apple]: ranges::views::enumerate(apples)) {

					bool has_unknown_information =
							discovery_status[goal_id] == DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT ||
							discovery_status[goal_id] == DiscoveryStatus::ROBOT_THINKS_EXISTS_BUT_DOESNT;

					if (has_unknown_information && can_see_apple(state, apple)) {
						return GoalSighting{(int) goal_id, {segment_i, t}};
					}
				}

			}

		}

		return std::nullopt;
	}

	DiscoveryStatusStats getDiscoveryStatusStats(const std::vector<utilities::DiscoveryStatus> &statuses) {
		size_t n_total = statuses.size();
		size_t n_visited = ranges::count(statuses, utilities::DiscoveryStatus::VISITED);
		size_t n_discoverable = ranges::count(statuses, utilities::DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT);
		size_t n_false = ranges::count(statuses, utilities::DiscoveryStatus::ROBOT_THINKS_EXISTS_BUT_DOESNT);
		size_t known_unvisited = ranges::count(statuses, utilities::DiscoveryStatus::KNOWN_TO_ROBOT);

		return {n_total, n_visited, n_discoverable, n_false, known_unvisited};
	}

	Json::Value toJson(const DiscoveryStatusStats &stats) {
		Json::Value root;
		root["total"] = static_cast<int>(stats.total);
		root["visited"] = static_cast<int>(stats.visited);
		root["discoverable"] = static_cast<int>(stats.discoverable);
		root["false"] = static_cast<int>(stats.false_positives);
		return root;
	}


}