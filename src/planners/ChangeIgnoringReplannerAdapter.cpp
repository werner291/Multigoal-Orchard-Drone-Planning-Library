// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "ChangeIgnoringReplannerAdapter.h"
#include "../utilities/ompl_tools.h"

ChangeIgnoringReplannerAdapter::ChangeIgnoringReplannerAdapter(const std::shared_ptr<MultiGoalPlanner> &staticPlanner)
		: static_planner(staticPlanner) {
}

std::optional<DynamicMultiGoalPlanner::PathSegment>
ChangeIgnoringReplannerAdapter::plan_initial(const ompl::base::SpaceInformationPtr &si,
											 const ompl::base::State *start,
											 const std::vector<ompl::base::GoalPtr> &goals,
											 const AppleTreePlanningScene &planning_scene) {

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	static_plan = static_planner->plan(si, start, goals, planning_scene, ptc);

	if (!static_plan.has_value()) {
		std::cout << "No initial plan found." << std::endl;
		return std::nullopt;
	}

	if (static_plan->segments.empty()) {
		std::cout << "Initial plan is empty." << std::endl;
		return std::nullopt;
	}

	return static_plan->segments.front().path_;
}


std::optional<DynamicMultiGoalPlanner::PathSegment>
ChangeIgnoringReplannerAdapter::replan_after_path_end(const ompl::base::SpaceInformationPtr &si,
													  const ompl::base::State *current_state,
													  const AppleTreePlanningScene &planning_scene) {

	static_plan->segments.erase(static_plan->segments.begin());

	if (static_plan->segments.empty()) {
		return std::nullopt;
	}

	return static_plan->segments.front().path_;

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

	if (!static_plan.has_value()) {
		return std::nullopt;
	}

	utilities::truncatePathToInterrupt(static_plan->segments[0].path_, at_interrupt);

	return static_plan->segments.front().path_;
}

std::optional<DynamicMultiGoalPlanner::PathSegment>
ChangeIgnoringReplannerAdapter::replan_after_removal(const ompl::base::SpaceInformationPtr &si,
													 const ompl::base::State *current_state,
													 const ompl::base::GoalPtr &removed_goal,
													 const PathInterrupt &at_interrupt,
													 const AppleTreePlanningScene &planning_scene) {
	return from_interrupt(at_interrupt);
}
