// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_GOAL_EVENTS_H
#define NEW_PLANNERS_GOAL_EVENTS_H

#include <variant>
#include <vector>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "../procedural_tree_generation.h"

namespace utilities {

	struct GoalVisit {
		int goal_id;
		double time;
	};

	struct GoalSighting {
		int goal_id;
		double time;
	};

	using GoalEvent = std::variant<GoalVisit, GoalSighting>;

	/**
	 * @brief Computes the goal events based on the robot trajectory, apples, visit and discovery distance limits.
	 *
	 * The function computes events for each apple in the given vector. An event can be a discovery event, indicating when the apple
	 * is first discovered by the robot, or a visit event, indicating when the robot visits the apple. The discovery and visit times are
	 * determined based on the robot trajectory and the given visit and discovery distance limits. If the apple is never discovered or
	 * visited, no event is generated for it. The function returns a vector of GoalEvent objects, which can be either a GoalSighting
	 * or a GoalVisit object, depending on the type of the event. The events are sorted by their time in ascending order.
	 *
	 * @param traj The robot trajectory.
	 * @param apples The vector of apples.
	 * @param visit_max_distance The maximum allowed distance for visiting an apple.
	 * @param discovery_max_distance The maximum allowed distance for discovering an apple.
	 * @return A vector of goal events, sorted by their time in ascending order.
	 */
	std::vector<GoalEvent> goal_events(const robot_trajectory::RobotTrajectory &traj,
									   const std::vector<Apple> &apples,
									   double visit_max_distance,
									   double discovery_max_distance);


}

#endif //NEW_PLANNERS_GOAL_EVENTS_H
