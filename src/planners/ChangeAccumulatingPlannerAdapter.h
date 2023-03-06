//
// Created by werner on 6-3-23.
//

#ifndef NEW_PLANNERS_CHANGEACCUMULATINGPLANNERADAPTER_H
#define NEW_PLANNERS_CHANGEACCUMULATINGPLANNERADAPTER_H

#include "../DynamicMultiGoalPlanner.h"

/**
 * @class ChangeAccumulatingPlanner
 *
 * A dynamic goalset planner that relies on an underlying static MultiGoalPlanner
 * to plan the path.
 *
 * The planner starts out planning a path that visits the first goals given initially,
 * then returns this path segment-by-segment. When a new goal is added, the planner
 * adds this to a list of goals to visit, and plans a path to visit all these new goals
 * as a batch once the first path is finished.
 *
 */
class ChangeAccumulatingPlannerAdapter : public DynamicMultiGoalPlanner {

	const std::shared_ptr<MultiGoalPlanner> static_planner;

	std::optional<MultiGoalPlanner::PlanResult> static_plan;

	std::vector<ompl::base::GoalPtr> batch;

public:
	ChangeAccumulatingPlannerAdapter(const std::shared_ptr<MultiGoalPlanner> &staticPlanner);

	std::optional<DynamicMultiGoalPlanner::PathSegment> plan_initial(
			const ompl::base::SpaceInformationPtr &si,
			const ompl::base::State *start,
			const std::vector<ompl::base::GoalPtr> &goals,
			const AppleTreePlanningScene &planning_scene) override;

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


#endif //NEW_PLANNERS_CHANGEACCUMULATINGPLANNERADAPTER_H
