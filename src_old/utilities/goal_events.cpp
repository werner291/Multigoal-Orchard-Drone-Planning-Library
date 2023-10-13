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

						GoalSightingType type =
								discovery_status[goal_id] == DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT
								? GoalSightingType::FOUND_NEW_GOAL : GoalSightingType::FOUND_FAKE_GOAL;

						return GoalSighting{.goal_id=(int) goal_id, .time={segment_i, t}, .type = type};
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

	Json::Value toJSON(const DiscoveryStatusStats &stats) {
		Json::Value json;
		json["total"] = (int) stats.total;
		json["visited"] = (int) stats.visited;
		json["discoverable"] = (int) stats.discoverable;
		json["false_positives"] = (int) stats.false_positives;
		json["known_unvisited"] = (int) stats.known_unvisited;
		return json;
	}

	Json::Value toJSON(const DiscoveryStatus &status) {
		switch (status) {
			case VISITED:
				return "VISITED";
			case KNOWN_TO_ROBOT:
				return "KNOWN_TO_ROBOT";
			case EXISTS_BUT_UNKNOWN_TO_ROBOT:
				return "EXISTS_BUT_UNKNOWN_TO_ROBOT";
			case ROBOT_THINKS_EXISTS_BUT_DOESNT:
				return "ROBOT_THINKS_EXISTS_BUT_DOESNT";
			case REMOVED:
				return "REMOVED";
		}

		throw std::runtime_error("Unknown DiscoveryStatus");
	}

	Json::Value toJSON(const GoalSightingType &type) {

		switch (type) {
			case FOUND_NEW_GOAL:
				return "FOUND_NEW_GOAL";
			case FOUND_FAKE_GOAL:
				return "FOUND_FAKE_GOAL";
		}

		throw std::runtime_error("Unknown GoalSightingType");
	}

	Json::Value toJSON(const GoalSighting &sighting) {

		Json::Value json;
		json["goal_id"] = sighting.goal_id;
		// We leave out the interrupt because it will make no sense to the user; it's used internally to tell
		// planners where to cut any paths they may have stored internally.
		json["discovery_type"] = toJSON(sighting.type);

		return json;

	}

	Json::Value toJSON(const PathEnd &end) {

		Json::Value json;

		for (const utilities::GoalId &goal_id: end.goals_reached) {
			json["goals_visited"].append(goal_id);
		}

		return json;

	}

	Json::Value toJSON(const GoalEvent &event) {

		if (std::holds_alternative<PathEnd>(event)) {

			Json::Value value = toJSON(std::get<PathEnd>(event));
			value["type"] = "PathEnd";
			return value;

		} else if (std::holds_alternative<GoalSighting>(event)) {

			Json::Value value = toJSON(std::get<GoalSighting>(event));
			value["type"] = "GoalSighting";
			return value;

		}

		throw std::runtime_error("Unknown GoalEvent type");

	}


}