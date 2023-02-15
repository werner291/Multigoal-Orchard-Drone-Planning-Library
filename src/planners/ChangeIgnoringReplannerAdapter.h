// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-2-23.
//

#ifndef NEW_PLANNERS_CHANGEIGNORINGREPLANNERADAPTER_H
#define NEW_PLANNERS_CHANGEIGNORINGREPLANNERADAPTER_H


#include "../DynamicMultiGoalPlanner.h"

class ChangeIgnoringReplannerAdapter : public DynamicMultiGoalPlanner {

	const std::shared_ptr<MultiGoalPlanner> static_planner;

	std::optional<PlanResult> last_plan_result;

public:
	explicit ChangeIgnoringReplannerAdapter(const std::shared_ptr<MultiGoalPlanner> &staticPlanner);

	PlanResult plan(const ompl::base::SpaceInformationPtr &si,
					const ompl::base::State *start,
					const std::vector<ompl::base::GoalPtr> &goals,
					const AppleTreePlanningScene &planning_scene,
					ompl::base::PlannerTerminationCondition &ptc) override;

	PlanResult replan(const ompl::base::SpaceInformationPtr &si,
					  const ompl::base::State *current_state,
					  const GoalChanges &goal_changes,
					  const AppleTreePlanningScene &planning_scene,
					  ompl::base::PlannerTerminationCondition &ptc) override;

};


#endif //NEW_PLANNERS_CHANGEIGNORINGREPLANNERADAPTER_H
