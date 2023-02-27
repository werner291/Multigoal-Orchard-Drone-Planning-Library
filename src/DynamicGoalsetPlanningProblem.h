// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_DYNAMICGOALSETPLANNINGPROBLEM_H
#define NEW_PLANNERS_DYNAMICGOALSETPLANNINGPROBLEM_H

#include <vector>
#include <moveit/robot_state/robot_state.h>
#include "utilities/experiment_utils.h"
#include "utilities/goal_events.h"

struct DynamicGoalsetPlanningProblem {

	moveit::core::RobotState start_state;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
	utilities::CanSeeAppleFn can_see_apple;

	static std::vector<DynamicGoalsetPlanningProblem>
	genDynamicGoalsetPlanningProblems(const AppleTreePlanningScene &scene,
									  const moveit::core::RobotModelPtr &robot,
									  int reps);

};


#endif //NEW_PLANNERS_DYNAMICGOALSETPLANNINGPROBLEM_H
