// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "ChangeIgnoringReplannerAdapter.h"
#include "../utilities/ompl_tools.h"

ChangeIgnoringReplannerAdapter::ChangeIgnoringReplannerAdapter(const std::shared_ptr<MultiGoalPlanner> &staticPlanner)
		: static_planner(staticPlanner) {
}

std::optional<DynamicMultiGoalPlanner::PathSegment>
ChangeIgnoringReplannerAdapter::plan(const ompl::base::SpaceInformationPtr &si,
									 const ompl::base::State *start,
									 const std::vector<ompl::base::GoalPtr> &goals,
									 const AppleTreePlanningScene &planning_scene) {
	auto ptc = ompl::base::plannerNonTerminatingCondition();
	static_plan = static_planner->plan(si, start, goals, planning_scene, ptc);

	return next_segment();
}


std::optional<DynamicMultiGoalPlanner::PathSegment>
ChangeIgnoringReplannerAdapter::replan_after_successful_visit(const ompl::base::SpaceInformationPtr &si,
															  const ompl::base::State *current_state,
															  const ompl::base::GoalPtr &visited_goal,
															  const AppleTreePlanningScene &planning_scene) {
	return next_segment();
}

std::optional<DynamicMultiGoalPlanner::PathSegment>
ChangeIgnoringReplannerAdapter::replan_after_discovery(const ompl::base::SpaceInformationPtr &si,
													   const ompl::base::State *current_state,
													   const ompl::base::GoalPtr &new_goal,
													   const PathInterrupt &at_interrupt,
													   const AppleTreePlanningScene &planning_scene) {

	return from_interrupt(at_interrupt);

}


std::optional<DynamicMultiGoalPlanner::PathSegment>
ChangeIgnoringReplannerAdapter::from_interrupt(const PathInterrupt &at_interrupt) {

	utilities::truncatePathToInterrupt(static_plan->segments[0].path_, at_interrupt);

	return static_plan->segments.front();
}

std::optional<DynamicMultiGoalPlanner::PathSegment>
ChangeIgnoringReplannerAdapter::replan_after_removal(const ompl::base::SpaceInformationPtr &si,
													 const ompl::base::State *current_state,
													 const ompl::base::GoalPtr &removed_goal,
													 const PathInterrupt &at_interrupt,
													 const AppleTreePlanningScene &planning_scene) {
	return from_interrupt(at_interrupt);
}

std::optional<DynamicMultiGoalPlanner::PathSegment> ChangeIgnoringReplannerAdapter::next_segment() {
	if (static_plan.has_value() && static_plan_index < static_plan->segments.size()) {
		return static_plan->segments[static_plan_index++];
	} else {
		return std::nullopt;
	}
}
