
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
void orderWithOrTools(const InitialApproachPath<ShellPoint> &start,
					  std::vector<OmplApproachPath<ShellPoint>> &approaches,
					  const OmplShellSpace<ShellPoint> &shell) {

	auto ordering = tsp_open_end(
			[&](auto i) {
				return shell.predict_path_length(start.shell_point, approaches[i].shell_point);
			},
			[&](auto i, auto j) {
				return shell.predict_path_length(approaches[i].shell_point,approaches[j].shell_point);
			},
			approaches.size()
	);

	std::vector<OmplApproachPath<ShellPoint>> ordered_approaches;
	for (auto [i, j] : ranges::views::enumerate(ordering)) {
		ordered_approaches.push_back(approaches[j]);
	}
	approaches = ordered_approaches;

}

template<typename ShellPoint>
class ShellPathPlanner : public MultiGoalPlanner {

public:
	MkOmplShellFn<ShellPoint> shell_builder;
	const std::unique_ptr<ApproachPlanningMethods<ShellPoint>> methods;
	const bool optimize_segments;

	ShellPathPlanner(MkOmplShellFn<ShellPoint> shellBuilder,
					 std::unique_ptr<ApproachPlanningMethods<ShellPoint>> methods,
					 bool optimizeSegments)
			: shell_builder(shellBuilder), methods(std::move(methods)), optimize_segments(optimizeSegments){
	}

	MultiGoalPlanner::PlanResult plan(
			const ompl::base::SpaceInformationPtr &si,
			const ompl::base::State *start,
			const std::vector<ompl::base::GoalPtr> &goals,
			const AppleTreePlanningScene &planning_scene,
			ompl::base::PlannerTerminationCondition& ptc) override {

		auto shell = shell_builder(planning_scene, si);

		PlanResult result {{}};

		std::optional<InitialApproachPath<ShellPoint>> initial_approach = methods->initial_approach_path(start, *shell);

		if (!initial_approach) {
			return result;
		}

		std::vector<OmplApproachPath<ShellPoint>> approach_paths = mkApproachPaths(goals, *shell);

		if (approach_paths.empty()) {
			return result;
		}

		orderWithOrTools(*initial_approach, approach_paths, *shell);

		{
			ompl::geometric::PathGeometric path = buildInitialApproach(si, *shell, initial_approach, approach_paths);
			result.segments.push_back(MultiGoalPlanner::PathSegment{SIZE_MAX, path});
		}

		for (size_t i = 0; i+1 < approach_paths.size(); i++) {
			ompl::geometric::PathGeometric path = buildGoalToGoal(si, *shell, approach_paths, i);
			result.segments.push_back(MultiGoalPlanner::PathSegment{SIZE_MAX, path});
		}

		return result;
	}

	[[nodiscard]] ompl::geometric::PathGeometric buildGoalToGoal(const ompl::base::SpaceInformationPtr &si,
																 const OmplShellSpace<ShellPoint> &shell,
																 const std::vector<OmplApproachPath<ShellPoint>> &approach_paths,
																 size_t i) const {
		ompl::geometric::PathGeometric path(si);
		path.append(approach_paths[i].robot_path);
		path.reverse();
		shell.shellPath(approach_paths[i].shell_point, approach_paths[i+1].shell_point);
		path.append(approach_paths[i+1].robot_path);

		if (optimize_segments) {
			path = optimize(path, {nullptr}, si);
		}
		return path;
	}

	[[nodiscard]] ompl::geometric::PathGeometric buildInitialApproach(const ompl::base::SpaceInformationPtr &si,
																	  const OmplShellSpace<ShellPoint> &shell,
																	  const std::optional<InitialApproachPath<ShellPoint>> &initial_approach,
																	  const std::vector<OmplApproachPath<ShellPoint>> &approach_paths) const {
		ompl::geometric::PathGeometric path(si);
		path.append(initial_approach->robot_path);
		shell.shellPath(initial_approach->shell_point, approach_paths.front().shell_point);
		path.append(approach_paths.front().robot_path);

		if (optimize_segments) {
			path = optimize(path, {nullptr}, si);
		}
		return path;
	}

	std::vector<OmplApproachPath<ShellPoint>>
	mkApproachPaths(const std::vector<ompl::base::GoalPtr> &goals, const OmplShellSpace<ShellPoint> &shell) const {
		std::vector<OmplApproachPath<ShellPoint>> approach_paths;

		for (auto &goal : goals) {
			auto approach_path = methods->approach_path(goal, shell);
			if (approach_path) {
				approach_paths.push_back(*approach_path);
			}
		}
		return approach_paths;
	}

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

//class PaddedSphereShellAroundLeavesBuilder : public ShellPathPlanner<Eigen::Vector3d>::ShellBuilder {
//
//	double padding;
//public:
//	PaddedSphereShellAroundLeavesBuilder(double padding = 0.1);
//
//public:
//	std::shared_ptr<OMPLShellSpaceWrapper<Eigen::Vector3d>>
//	buildShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) const override;
//
//	[[nodiscard]] Json::Value parameters() const override;
//
//};

#endif //NEW_PLANNERS_SHELLPATHPLANNER_H
