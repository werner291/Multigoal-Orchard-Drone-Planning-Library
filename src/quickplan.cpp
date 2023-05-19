// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-5-23.
//

#include "quickplan.h"

RobotPath quickPlan(AppleTreePlanningScene &scene,
					const moveit::core::RobotState &start_state,
					const StaticPlannerAllocatorFn &planner_allocator) {

	auto ss = omplStateSpaceForDrone(start_state.getRobotModel());
	auto si = loadSpaceInformation(ss, scene);
	auto planner = planner_allocator(si);
	ompl::base::ScopedState<> start(ss);
	ss->copyToOMPLState(start.get(), start_state);
	auto ptc = ompl::base::plannerNonTerminatingCondition();
	auto rpath_moveit = omplPathToRobotPath(planner->plan(si, start.get(), appleGoalsFromScene(scene, si), scene, ptc)
													.combined());

	return rpath_moveit;

}

std::vector<ompl::base::GoalPtr>
appleGoalsFromScene(AppleTreePlanningScene &scene, ompl::base::SpaceInformationPtr &si) {

	std::vector<ompl::base::GoalPtr> goals;

	for (const auto &apple: scene.apples) {
		goals.push_back(std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center));
	}

	return goals;
}
