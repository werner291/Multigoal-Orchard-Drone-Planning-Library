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

	std::optional<GoalSighting>
	find_earliest_discovery_event(RobotPath &traj, const std::vector<Apple> &apples, double discovery_max_distance);

	enum DiscoveryStatus {
		VISITED, KNOWN_TO_ROBOT, EXISTS_BUT_UNKNOWN_TO_ROBOT
	};

}

#endif //NEW_PLANNERS_GOAL_EVENTS_H
