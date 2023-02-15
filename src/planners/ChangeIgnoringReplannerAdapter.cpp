// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "ChangeIgnoringReplannerAdapter.h"

DynamicMultiGoalPlanner::PlanResult ChangeIgnoringReplannerAdapter::plan(const ompl::base::SpaceInformationPtr &si,
																		 const ompl::base::State *start,
																		 const std::vector<ompl::base::GoalPtr> &goals,
																		 const AppleTreePlanningScene &planning_scene,
																		 ompl::base::PlannerTerminationCondition &ptc) {

	last_plan_result = static_planner->plan(si, start, goals, planning_scene, ptc);

	return *last_plan_result;
}

DynamicMultiGoalPlanner::PlanResult ChangeIgnoringReplannerAdapter::replan(const ompl::base::SpaceInformationPtr &si,
																		   const ompl::base::State *current_state,
																		   const DynamicMultiGoalPlanner::GoalChanges &goal_changes,
																		   const AppleTreePlanningScene &planning_scene,
																		   ompl::base::PlannerTerminationCondition &ptc) {
	return *last_plan_result;
}

ChangeIgnoringReplannerAdapter::ChangeIgnoringReplannerAdapter(const std::shared_ptr<MultiGoalPlanner> &staticPlanner)
		: static_planner(staticPlanner) {
}
