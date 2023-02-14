
#ifndef NEW_PLANNERS_APPROACHPLANNING_H
#define NEW_PLANNERS_APPROACHPLANNING_H

#include <optional>
#include <mutex>
#include <memory>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "../../DronePathLengthObjective.h"
#include "../../shell_space/OmplShellSpace.h"
#include "../../SingleGoalPlannerMethods.h"

template<typename ShellPoint>
struct OmplApproachPath {
	ShellPoint shell_point;
	ompl::geometric::PathGeometric robot_path;
};

template<typename ShellPoint>
struct InitialApproachPath {
	ShellPoint shell_point;
	ompl::geometric::PathGeometric robot_path;
};

template<typename ShellPoint>
class ApproachPlanningMethods {

public:

	virtual std::optional<InitialApproachPath<ShellPoint>> initial_approach_path(const ompl::base::State *start, const OmplShellSpace<ShellPoint>& shell) const = 0;

	virtual std::optional<OmplApproachPath<ShellPoint>>
	approach_path(const ompl::base::GoalPtr &goal, const OmplShellSpace<ShellPoint> &shell) const = 0;

	virtual ~ApproachPlanningMethods() = default;
};

template<typename ShellPoint>
class MakeshiftPrmApproachPlanningMethods : public ApproachPlanningMethods<ShellPoint> {

	// Mutable because there isn't *supposed* to be any transfer of data in between calls.
	// Let's have a mutex just in case, we're limited by the collision detection mutex anyway,
	// so this won't cause a major speed penalty.
	mutable std::mutex mutex;
	mutable std::shared_ptr<SingleGoalPlannerMethods> single_goal_planner_methods;

public:
	explicit MakeshiftPrmApproachPlanningMethods(ompl::base::SpaceInformationPtr si) {

		ompl::base::PlannerAllocator mkprm = [](const ompl::base::SpaceInformationPtr & si) {
			return std::make_shared<ompl::geometric::PRMstar>(si);
		};

		ompl::base::OptimizationObjectivePtr objective = std::make_shared<DronePathLengthObjective>(si);

		single_goal_planner_methods = std::make_shared<SingleGoalPlannerMethods>(1.0,si,objective,mkprm,true,true,true);

	}

	std::optional<InitialApproachPath<ShellPoint>> initial_approach_path(
			const ompl::base::State *start,
			const OmplShellSpace<ShellPoint> &shell) const override {

		std::scoped_lock lock(mutex);

		ompl::base::ScopedState<> shell_state(shell.getSpaceInformation());

		ShellPoint shellPoint = shell.pointNearState(start);

		shell.stateFromPoint(shellPoint, shell_state.get());

		auto path = single_goal_planner_methods->state_to_state(start, shell_state.get());
		
		if (path) {
			return InitialApproachPath<ShellPoint>{shellPoint, *path};
		} else {
			return std::nullopt;
		}
	}

	std::optional<OmplApproachPath<ShellPoint>>
	approach_path(const ompl::base::GoalPtr &goal, const OmplShellSpace<ShellPoint> &shell) const override {

		std::scoped_lock lock(mutex);

		ompl::base::ScopedState<> shell_state(shell.getSpaceInformation());

		ShellPoint shellPoint = shell.pointNearGoal(goal.get());

		shell.stateFromPoint(shellPoint, shell_state.get());

		auto path = single_goal_planner_methods->state_to_goal(shell_state.get(), goal);

		if (path) {
			return OmplApproachPath<ShellPoint>{shellPoint, *path};
		} else {
			return std::nullopt;
		}
	}

};

#endif //NEW_PLANNERS_APPROACHPLANNING_H
