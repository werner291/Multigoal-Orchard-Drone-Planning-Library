// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-2-23.
//

#include "DynamicMultiGoalPlannerOmplToMoveitAdapter.h"

#include <utility>

DynamicMultiGoalPlannerOmplToMoveitAdapter::DynamicMultiGoalPlannerOmplToMoveitAdapter(const std::shared_ptr<DynamicMultiGoalPlanner> &planner,
																					   ompl::base::SpaceInformationPtr si,
																					   const std::shared_ptr<DroneStateSpace> &ss)
		: planner(planner), si(std::move(si)), ss(ss) {
}

std::optional<MoveitPathSegment>
DynamicMultiGoalPlannerOmplToMoveitAdapter::plan(const moveit::core::RobotState &start_state,
												 const AppleTreePlanningScene &planning_scene) {

	ompl::base::ScopedState start(si);

	// Convert the start state to an OMPL state.
	ss->copyToOMPLState(start.get(), start_state);

	auto goals = constructNewAppleGoals(si, planning_scene.apples);

	auto result = planner->plan(si, start.get(), goals, planning_scene);

	if (!result) {
		return std::nullopt;
	}

	// Convert the path to a MoveIt path.
	return {{.path = omplPathToRobotPath(result->path_), .goal_id = result->to_goal_id_}};

}

std::optional<MoveitPathSegment>
DynamicMultiGoalPlannerOmplToMoveitAdapter::replan_after_successful_visit(const moveit::core::RobotState &start_state,
																		  const AppleTreePlanningScene &planning_scene) {

	ompl::base::ScopedState start(si);

	// Convert the start state to an OMPL state.
	ss->copyToOMPLState(start.get(), start_state);

	auto result = planner->replan_after_successful_visit(si, start.get(), planning_scene);

	if (!result) {
		return std::nullopt;
	}

	// Convert the path to a MoveIt path.
	return {{.path = omplPathToRobotPath(result->path_), .goal_id = result->to_goal_id_}};

}

std::optional<MoveitPathSegment>
DynamicMultiGoalPlannerOmplToMoveitAdapter::replan_after_discovery(const moveit::core::RobotState &start_state,
																   const Apple &apple,
																   const PathInterrupt &interrupt,
																   const AppleTreePlanningScene &planning_scene) {

	ompl::base::ScopedState start(si);

	// Convert the start state to an OMPL state.
	ss->copyToOMPLState(start.get(), start_state);

	auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, APPLE_VISIT_MARGIN, apple.center);

	auto result = planner->replan_after_discovery(si, start.get(), goal, interrupt, planning_scene);

	if (!result) {
		return std::nullopt;
	}

	// Convert the path to a MoveIt path.
	return {{.path = omplPathToRobotPath(result->path_), .goal_id = result->to_goal_id_}};

}
