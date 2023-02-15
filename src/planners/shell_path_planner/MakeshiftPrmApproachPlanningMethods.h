// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-2-23.
//

#ifndef NEW_PLANNERS_MAKESHIFTPRMAPPROACHPLANNINGMETHODS_H
#define NEW_PLANNERS_MAKESHIFTPRMAPPROACHPLANNINGMETHODS_H

#include "ApproachPlanning.h"
#include "../../SingleGoalPlannerMethods.h"

template<typename ShellPoint>
class MakeshiftPrmApproachPlanningMethods : public ApproachPlanningMethods<ShellPoint> {

	// Mutable because there isn't *supposed* to be any transfer of data in between calls.
	// Let's have a mutex just in case, we're limited by the collision detection mutex anyway,
	// so this won't cause a major speed penalty.
	mutable std::mutex mutex;
	mutable std::shared_ptr<SingleGoalPlannerMethods> single_goal_planner_methods;

	const ompl::base::SpaceInformationPtr si;

public:
	explicit MakeshiftPrmApproachPlanningMethods(ompl::base::SpaceInformationPtr si);

	std::optional<OmplApproachPath<ShellPoint>>
	approach_path(const ompl::base::State *start, const OmplShellSpace<ShellPoint> &shell) const override;

	std::optional<OmplApproachPath<ShellPoint>>
	approach_path(const ompl::base::GoalPtr &goal, const OmplShellSpace<ShellPoint> &shell) const override;

};


#endif //NEW_PLANNERS_MAKESHIFTPRMAPPROACHPLANNINGMETHODS_H
