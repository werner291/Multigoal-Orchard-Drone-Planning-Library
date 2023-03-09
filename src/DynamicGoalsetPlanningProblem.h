// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_DYNAMICGOALSETPLANNINGPROBLEM_H
#define NEW_PLANNERS_DYNAMICGOALSETPLANNINGPROBLEM_H

#include <vector>
#include <moveit/robot_state/robot_state.h>

#include "utilities/goal_events.h"
#include "utilities/discoverability_specifications.h"

struct DynamicGoalsetPlanningProblem {

	moveit::core::RobotState start_state;
	AppleTreePlanningScene scene;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
	utilities::CanSeeAppleFn* can_see_apple;

};


#endif //NEW_PLANNERS_DYNAMICGOALSETPLANNINGPROBLEM_H
