// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include "goal_events.h"
#include "moveit.h"

namespace utilities {

	bool within_distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double distance) {
		return (a - b).norm() < distance;
	}

	auto indexes_within_distance(const Eigen::Vector3d &a, const std::vector<Apple> &apples, double distance) {
		return apples | ranges::views::enumerate | ranges::views::filter([a, distance](const auto &apple) {
			return within_distance(a, apple.second.center, distance);
		}) | ranges::views::transform([](const auto &apple) {
			return apple.first;
		});
	}

	std::vector<GoalEvent> goal_events(const robot_trajectory::RobotTrajectory &traj,
									   const std::vector<Apple> &apples,
									   double visit_max_distance,
									   double discovery_max_distance) {

		struct EventTimesForApple {
			double discovery_time = std::numeric_limits<double>::infinity();
			double visit_time = std::numeric_limits<double>::infinity();
		};

		std::vector<EventTimesForApple> event_times(apples.size());

		for (size_t waypoint_i = 0; waypoint_i < traj.getWayPointCount(); ++waypoint_i) {
			const auto &waypoint = traj.getWayPoint(waypoint_i);
			double t = traj.getWayPointDurationFromStart(waypoint_i);
			Eigen::Vector3d ee_pos = waypoint.getGlobalLinkTransform("end_effector").translation();

			for (auto goal_id: indexes_within_distance(ee_pos, apples, visit_max_distance)) {
				event_times[goal_id].visit_time = std::min(event_times[goal_id].visit_time, t);
			}
		}

		double t = 0.0;

		while (t < traj.getWayPointDurationFromStart(traj.getWayPointCount() - 1)) {

			moveit::core::RobotState state(traj.getRobotModel());

			setStateToTrajectoryPoint(state, t, traj);

			Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

			for (auto goal_id: indexes_within_distance(ee_pos, apples, discovery_max_distance)) {
				event_times[goal_id].discovery_time = std::min(event_times[goal_id].discovery_time, t);
			}

			t += 0.1; // Too big? Too small? Just right??? Maybe change based on end-effector distance traveled?

		}

		std::vector<GoalEvent> events;

		for (size_t goal_id = 0; goal_id < apples.size(); ++goal_id) {
			if (std::isfinite(event_times[goal_id].discovery_time)) {
				events.emplace_back(GoalSighting{(int) goal_id, event_times[goal_id].discovery_time});
			}
			if (std::isfinite(event_times[goal_id].visit_time)) {
				events.emplace_back(GoalVisit{(int) goal_id, event_times[goal_id].visit_time});
			}
		}

		std::sort(events.begin(), events.end(), [](const auto &a, const auto &b) {
			return std::visit([](const auto &a) { return a.time; }, a) <
				   std::visit([](const auto &b) { return b.time; }, b);
		});

		return events;
	}

	std::optional<RecomputationEvent> find_recomputation_event(const std::vector<utilities::GoalEvent> &events,
															   const std::vector<ompl::base::GoalPtr> &goals,
															   std::vector<DiscoveryStatus> &discovery_status) {

		RecomputationEvent re;

		for (const auto &event: events) {

			// Check if the current event is a GoalVisit
			if (std::holds_alternative<utilities::GoalVisit>(event)) {

				const auto &visit = std::get<utilities::GoalVisit>(event);

				// Mark the goal as visited
				discovery_status[visit.goal_id] = DiscoveryStatus::VISITED;

				// Add the visited goal to the recomputation event
				re.goal_changes.visited_goals.push_back(goals[visit.goal_id]);

				// Check if the current event is a GoalSighting
			} else if (std::holds_alternative<utilities::GoalSighting>(event)) {

				const auto &discover = std::get<utilities::GoalSighting>(event);

				// Check if the goal has not been seen before by the robot
				if (discovery_status[discover.goal_id] == DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT) {

					// Mark the goal as known to the robot
					discovery_status[discover.goal_id] = DiscoveryStatus::KNOWN_TO_ROBOT;

					// Add the newly discovered goal to the recomputation event
					re.goal_changes.new_goals.push_back(goals[discover.goal_id]);

					// Return the recomputation event
					return re;

				}
			}
		}

		// No recomputation event was found
		return std::nullopt;
	}

}