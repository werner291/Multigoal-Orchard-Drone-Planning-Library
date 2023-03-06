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
#include "../RobotPath.h"
#include "../PathInterrupt.h"

namespace utilities {

	struct GoalVisit {
		int goal_id;
	};

	struct GoalSighting {
		int goal_id;
		PathInterrupt time;
	};

	using GoalEvent = std::variant<GoalVisit, GoalSighting>;

	enum DiscoveryStatus {
		// The apple exists, is known to the robot, and has successfully been reached.
		VISITED,
		// The apple exists, is known to the robot, but has yet to be reached
		KNOWN_TO_ROBOT,
		// The apple exists, but the robot doesn't know that.
		EXISTS_BUT_UNKNOWN_TO_ROBOT,
		// The apple does not exist, but the robot thinks it does, and may discover that it doesn't exist.
		ROBOT_THINKS_EXISTS_BUT_DOESNT,
		// The apple does not exist, and the robot knows that.
		REMOVED
	};

	using CanSeeAppleFn = std::function<bool(const moveit::core::RobotState& state, const Apple& apple)>;

	std::optional<GoalSighting> find_earliest_discovery_event(RobotPath &traj,
															  const std::vector<Apple> &apples,
															  CanSeeAppleFn can_see_apple,
															  const std::vector<DiscoveryStatus> &discovery_status);

}

#endif //NEW_PLANNERS_GOAL_EVENTS_H
