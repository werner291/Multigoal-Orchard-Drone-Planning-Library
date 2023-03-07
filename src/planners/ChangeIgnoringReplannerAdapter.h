// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-2-23.
//

#ifndef NEW_PLANNERS_CHANGEIGNORINGREPLANNERADAPTER_H
#define NEW_PLANNERS_CHANGEIGNORINGREPLANNERADAPTER_H


#include "../DynamicMultiGoalPlanner.h"

/**
 * @class ChangeIgnoringReplannerAdapter
 * @brief Adapts a static multi-goal planner into a dynamic multi-goal planner by ignoring changes to the goal set
 *
 * This class adapts a static multi-goal planner to the dynamic multi-goal planner interface by ignoring changes to the
 * goal set. While this is, admittedly, a somewhat useless adapter, it is useful for testing the dynamic multi-goal
 * planner interface.
 *
 * Notably, since we assume a fixed set of obstacles, the path will remain valid, albeit that it may not be optimal
 * and will miss any new goals.
 */
class ChangeIgnoringReplannerAdapter : public DynamicMultiGoalPlanner {

	const std::shared_ptr<MultiGoalPlanner> static_planner;

	std::optional<MultiGoalPlanner::PlanResult> static_plan;

public:
	explicit ChangeIgnoringReplannerAdapter(const std::shared_ptr<MultiGoalPlanner> &staticPlanner);

	std::optional<DynamicMultiGoalPlanner::PathSegment> plan_initial(const ompl::base::SpaceInformationPtr &si,
																	 const ompl::base::State *start,
																	 const std::vector<ompl::base::GoalPtr> &goals,
																	 const AppleTreePlanningScene &planning_scene,
																	 double padding) override;

	std::optional<DynamicMultiGoalPlanner::PathSegment>
	replan_after_path_end(const ompl::base::SpaceInformationPtr &si,
						  const ompl::base::State *current_state,
						  const AppleTreePlanningScene &planning_scene) override;

	std::optional<DynamicMultiGoalPlanner::PathSegment> replan_after_discovery(const ompl::base::SpaceInformationPtr &si,
																		const ompl::base::State *current_state,
																		const ompl::base::GoalPtr &new_goal,
																		const PathInterrupt &at_interrupt,
																		const AppleTreePlanningScene &planning_scene) override;

	std::optional<DynamicMultiGoalPlanner::PathSegment> replan_after_removal(const ompl::base::SpaceInformationPtr &si,
																	  const ompl::base::State *current_state,
																	  const ompl::base::GoalPtr &removed_goal,
																	  const PathInterrupt &at_interrupt,
																	  const AppleTreePlanningScene &planning_scene) override;

	std::optional<DynamicMultiGoalPlanner::PathSegment> from_interrupt(const PathInterrupt &at_interrupt);
};


#endif //NEW_PLANNERS_CHANGEIGNORINGREPLANNERADAPTER_H
