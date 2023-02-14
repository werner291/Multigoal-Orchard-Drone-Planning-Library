
#ifndef NEW_PLANNERS_SHELLPATHPLANNER_H
#define NEW_PLANNERS_SHELLPATHPLANNER_H

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>
#include "MultiGoalPlanner.h"
#include "../shell_space/OmplShellSpace.h"
#include "../DistanceHeuristics.h"
#include "../planning_scene_diff_message.h"
#include "../DronePathLengthObjective.h"
#include "../utilities/general_utilities.h"
#include "../utilities/traveling_salesman.h"
#include "../probe_retreat_move.h"
#include "../ExperimentVisualTools.h"
#include "shell_path_planner/Construction.h"
#include "shell_path_planner/ApproachPlanning.h"
#include "shell_path_planner/OrderingMethod.h"


template<typename ShellPoint>
class ShellPathPlanner : public MultiGoalPlanner {

public:
	MkOmplShellFn<ShellPoint> shell_builder;
	const std::unique_ptr<ApproachPlanningMethods<ShellPoint>> methods;
	const bool optimize_segments;

	ShellPathPlanner(MkOmplShellFn<ShellPoint> shellBuilder,
					 std::unique_ptr<ApproachPlanningMethods<ShellPoint>> methods,
					 bool optimizeSegments);

	MultiGoalPlanner::PlanResult plan(const ompl::base::SpaceInformationPtr &si,
									  const ompl::base::State *start,
									  const std::vector<ompl::base::GoalPtr> &goals,
									  const AppleTreePlanningScene &planning_scene,
									  ompl::base::PlannerTerminationCondition &ptc) override;

	[[nodiscard]] ompl::geometric::PathGeometric buildGoalToGoal(const ompl::base::SpaceInformationPtr &si,
																 const OmplShellSpace<ShellPoint> &shell,
																 const std::vector<OmplApproachPath<ShellPoint>> &approach_paths,
																 size_t i) const;

	[[nodiscard]] ompl::geometric::PathGeometric buildInitialApproach(const ompl::base::SpaceInformationPtr &si,
																	  const OmplShellSpace<ShellPoint> &shell,
																	  const std::optional<InitialApproachPath<ShellPoint>> &initial_approach,
																	  const std::vector<OmplApproachPath<ShellPoint>> &approach_paths) const;

	std::vector<OmplApproachPath<ShellPoint>>
	mkApproachPaths(const std::vector<ompl::base::GoalPtr> &goals, const OmplShellSpace<ShellPoint> &shell) const;

	[[nodiscard]] Json::Value parameters() const override {
		Json::Value result;
//
//		result["shell_builder_params"] = shell_builder->parameters();
//		result["ptp"] = methods->parameters();
		result["optimize_segments"] = optimize_segments;

		return result;
	}

	[[nodiscard]] std::string name() const {
		return "ShellPathPlanner";
	}

};


#endif //NEW_PLANNERS_SHELLPATHPLANNER_H
