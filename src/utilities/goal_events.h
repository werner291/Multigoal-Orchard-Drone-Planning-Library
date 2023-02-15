// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_GOAL_EVENTS_H
#define NEW_PLANNERS_GOAL_EVENTS_H

#include <variant>
#include <vector>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "../procedural_tree_generation.h"
#include "../DynamicMultiGoalPlanner.h"

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


	struct RecomputationEvent {
		DynamicMultiGoalPlanner::GoalChanges goal_changes;
		double at_t;
	};

	enum DiscoveryStatus {
		VISITED, KNOWN_TO_ROBOT, EXISTS_BUT_UNKNOWN_TO_ROBOT
	};

	/**
	 * Scan through the events and find the first recomputation event,
	 * recording all the goals that were visited in the meantime as well.
	 *
	 * @param events            Events to scan through.
	 * @param goals             Goals to look up by ID.
	 * @param discovery_status  Discovery status of each goal (will be modified!)
	 * @return                  Recomputation event, or nullopt if no recomputation event is found.
	 */
	[[nodiscard]]
	std::optional<RecomputationEvent> find_recomputation_event(const std::vector<utilities::GoalEvent> &events,
															   const std::vector<ompl::base::GoalPtr> &goals,
															   std::vector<DiscoveryStatus> &discovery_status);
}

#endif //NEW_PLANNERS_GOAL_EVENTS_H
