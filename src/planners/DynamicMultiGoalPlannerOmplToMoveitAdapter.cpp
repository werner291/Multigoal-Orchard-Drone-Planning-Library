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

std::optional<RobotPath>
DynamicMultiGoalPlannerOmplToMoveitAdapter::plan(const moveit::core::RobotState &start_state,
												 const AppleTreePlanningScene &planning_scene) {

	ompl::base::ScopedState start(si);

	// Convert the start state to an OMPL state.
	ss->copyToOMPLState(start.get(), start_state);

	std::vector<ompl::base::GoalPtr> goals;
	for (const auto &apple : planning_scene.apples) {
		apple_to_ompl_goal[apple] = std::make_shared<DroneEndEffectorNearTarget>(si, APPLE_VISIT_MARGIN, apple.center);
		goals.push_back(apple_to_ompl_goal[apple]);
	}

	auto result = planner->plan_initial(si, start.get(), goals, planning_scene, 0.5);

	if (!result) {
		return std::nullopt;
	}

	// Convert the path to a MoveIt path.
	return {omplPathToRobotPath(*result)};

}

std::optional<RobotPath>
DynamicMultiGoalPlannerOmplToMoveitAdapter::replan_after_successful_visit(const moveit::core::RobotState &start_state,
																		  const AppleTreePlanningScene &planning_scene) {

	ompl::base::ScopedState start(si);

	// Convert the start state to an OMPL state.
	ss->copyToOMPLState(start.get(), start_state);

	auto result = planner->replan_after_path_end(si, start.get(), planning_scene);

	if (!result) {
		return std::nullopt;
	}

	// Convert the path to a MoveIt path.
	return {omplPathToRobotPath(*result)};

}

std::optional<RobotPath>
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
	return {omplPathToRobotPath(*result)};

}

std::optional<RobotPath>
DynamicMultiGoalPlannerOmplToMoveitAdapter::replan_after_removal(const moveit::core::RobotState &start_state,
																 const Apple &apple,
																 const PathInterrupt &interrupt,
																 const AppleTreePlanningScene &planning_scene) {

	auto goal = apple_to_ompl_goal[apple];

	ompl::base::ScopedState start(si);

	// Convert the start state to an OMPL state.
	ss->copyToOMPLState(start.get(), start_state);

	auto result = planner->replan_after_removal(si, start.get(), goal, interrupt, planning_scene);

	if (!result) {
		return std::nullopt;
	}

	// Convert the path to a MoveIt path.
	return {omplPathToRobotPath(*result)};

}
