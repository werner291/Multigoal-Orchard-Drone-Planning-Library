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

		}

		std::vector<GoalEvent> events;

		for (size_t goal_id = 0; goal_id < apples.size(); ++goal_id) {
			if (std::is_finite(event_times[goal_id].discovery_time)) {
				events.emplace_back(GoalSighting{goal_id, event_times[goal_id].discovery_time});
			}
			if (std::is_finite(event_times[goal_id].visit_time)) {
				events.emplace_back(GoalVisit{goal_id, event_times[goal_id].visit_time});
			}
		}

		std::sort(events.begin(), events.end(), [](const auto &a, const auto &b) {
			return std::visit([](const auto &a) { return a.time; }, a) <
				   std::visit([](const auto &b) { return b.time; }, b);
		});

		return events;
	}

}