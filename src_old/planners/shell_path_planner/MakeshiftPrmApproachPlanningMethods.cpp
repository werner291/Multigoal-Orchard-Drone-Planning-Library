// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "MakeshiftPrmApproachPlanningMethods.h"
#include "../../DronePathLengthObjective.h"
#include "../../shell_space/CuttingPlaneConvexHullShell.h"
#include "../../shell_space/CGALMeshShell.h"
#include "../../shell_space/CylinderShell.h"

template<typename ShellPoint>
MakeshiftPrmApproachPlanningMethods<ShellPoint>::MakeshiftPrmApproachPlanningMethods(ompl::base::SpaceInformationPtr si,
																					 double t_max)
		: si(si) {

	ompl::base::PlannerAllocator mkprm = [](const ompl::base::SpaceInformationPtr &si) {
		return std::make_shared<ompl::geometric::PRMstar>(si);
	};

	ompl::base::OptimizationObjectivePtr objective = std::make_shared<DronePathLengthObjective>(si);

	single_goal_planner_methods = std::make_shared<SingleGoalPlannerMethods>(t_max,
																			 si,
																			 objective,
																			 mkprm,
																			 true,
																			 true,
																			 true);

}

template<typename ShellPoint>
std::optional<OmplApproachPath<ShellPoint>>
MakeshiftPrmApproachPlanningMethods<ShellPoint>::approach_path(const ompl::base::State *start,
															   const OmplShellSpace<ShellPoint> &shell) const {

	std::scoped_lock lock(mutex);

	ompl::base::ScopedState<> shell_state(si);

	ShellPoint shellPoint = shell.pointNearState(start);

	shell.stateFromPoint(shellPoint, shell_state.get());

	if (!si->isValid(shell_state.get())) {
		std::cout << "Shell point is not valid" << std::endl;

		std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace())->printState(shell_state.get(), std::cout);

		return std::nullopt;
	}

	auto path = single_goal_planner_methods->state_to_state(shell_state.get(), start);

	if (path) {
		return OmplApproachPath<ShellPoint>{shellPoint, *path};
	} else {
		return std::nullopt;
	}
}

template<typename ShellPoint>
std::optional<OmplApproachPath<ShellPoint>>
MakeshiftPrmApproachPlanningMethods<ShellPoint>::approach_path(const ompl::base::GoalPtr &goal,
															   const OmplShellSpace<ShellPoint> &shell) const {

	std::scoped_lock lock(mutex);

	ompl::base::ScopedState<> shell_state(si);

	ShellPoint shellPoint = shell.pointNearGoal(goal.get());

	shell.stateFromPoint(shellPoint, shell_state.get());

	if (!si->isValid(shell_state.get())) {
		std::cout << "Shell point is not valid" << std::endl;

		std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace())->printState(shell_state.get(), std::cout);

		return std::nullopt;
	}

	auto path = single_goal_planner_methods->state_to_goal(shell_state.get(), goal);

	if (path) {
		return OmplApproachPath<ShellPoint>{shellPoint, *path};
	} else {
		return std::nullopt;
	}
}

// Explicit instantiation for Eigen::Vector3d.
template
class MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>;

// Explicit instantiation for ConvexHullPoint
template
class MakeshiftPrmApproachPlanningMethods<ConvexHullPoint>;

// Explicit instantiation for mgodpl::cgal_utils::CGALMeshPointAndNormal
template
class MakeshiftPrmApproachPlanningMethods<mgodpl::cgal_utils::CGALMeshPointAndNormal>;

// Explicit instantiation for CylinderShellPoint
template
class MakeshiftPrmApproachPlanningMethods<CylinderShellPoint>;