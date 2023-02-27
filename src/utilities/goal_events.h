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
		VISITED, KNOWN_TO_ROBOT, EXISTS_BUT_UNKNOWN_TO_ROBOT
	};

	using CanSeeAppleFn = std::function<bool(const moveit::core::RobotState& state, const Apple& apple)>;

	std::optional<GoalSighting> find_earliest_discovery_event(RobotPath &traj,
															  const std::vector<Apple> &apples,
															  CanSeeAppleFn can_see_apple,
															  const std::vector<DiscoveryStatus> &discovery_status);

}

#endif //NEW_PLANNERS_GOAL_EVENTS_H
