#include "ShellPathPlanner.h"

/**
 * @brief Orders the approaches using the TSP solver from OR-Tools.
 *
 * @tparam ShellPoint 		The type of the shell point.
 * @param start 			The start point.
 * @param approaches 		The approaches to order.
 * @param shell 			The shell space.
 */
template<typename ShellPoint>
void orderWithOrTools(const InitialApproachPath<ShellPoint> &start,
					  std::vector<OmplApproachPath<ShellPoint>> &approaches,
					  const OmplShellSpace<ShellPoint> &shell) {

	auto ordering = tsp_open_end([&](auto i) {
		return shell.predict_path_length(start.shell_point, approaches[i].shell_point);
	}, [&](auto i, auto j) {
		return shell.predict_path_length(approaches[i].shell_point, approaches[j].shell_point);
	}, approaches.size());

	std::vector<OmplApproachPath<ShellPoint>> ordered_approaches;
	for (auto [i, j]: ranges::views::enumerate(ordering)) {
		ordered_approaches.push_back(approaches[j]);
	}
	approaches = ordered_approaches;

}

template<typename ShellPoint>
ShellPathPlanner<ShellPoint>::ShellPathPlanner(MkOmplShellFn<ShellPoint> shellBuilder,
											   std::unique_ptr<ApproachPlanningMethods<ShellPoint>> methods,
											   bool optimizeSegments)
		: shell_builder(shellBuilder), methods(std::move(methods)), optimize_segments(optimizeSegments) {
}

template<typename ShellPoint>
MultiGoalPlanner::PlanResult ShellPathPlanner<ShellPoint>::plan(const ompl::base::SpaceInformationPtr &si,
																const ompl::base::State *start,
																const std::vector<ompl::base::GoalPtr> &goals,
																const AppleTreePlanningScene &planning_scene,
																ompl::base::PlannerTerminationCondition &ptc) {

	auto shell = shell_builder(planning_scene, si);

	PlanResult result{{}};

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

	for (size_t i = 0; i + 1 < approach_paths.size(); i++) {
		ompl::geometric::PathGeometric path = buildGoalToGoal(si, *shell, approach_paths, i);
		result.segments.push_back(MultiGoalPlanner::PathSegment{SIZE_MAX, path});
	}

	return result;
}

template<typename ShellPoint>
std::vector<OmplApproachPath<ShellPoint>>
ShellPathPlanner<ShellPoint>::mkApproachPaths(const std::vector<ompl::base::GoalPtr> &goals,
											  const OmplShellSpace<ShellPoint> &shell) const {
	std::vector<OmplApproachPath<ShellPoint>> approach_paths;

	for (const auto &[i, goal]: ranges::views::enumerate(goals)) {

		std::cout << "Planning approach for goal " << i << std::endl;

		auto approach_path = methods->approach_path(goal, shell);
		if (approach_path) {
			approach_paths.push_back(*approach_path);
		}
	}
	return approach_paths;
}

template<typename ShellPoint>
ompl::geometric::PathGeometric
ShellPathPlanner<ShellPoint>::buildInitialApproach(const ompl::base::SpaceInformationPtr &si,
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

template<typename ShellPoint>
ompl::geometric::PathGeometric ShellPathPlanner<ShellPoint>::buildGoalToGoal(const ompl::base::SpaceInformationPtr &si,
																			 const OmplShellSpace<ShellPoint> &shell,
																			 const std::vector<OmplApproachPath<ShellPoint>> &approach_paths,
																			 size_t i) const {
	ompl::geometric::PathGeometric path(si);
	path.append(approach_paths[i].robot_path);
	path.reverse();
	shell.shellPath(approach_paths[i].shell_point, approach_paths[i + 1].shell_point);
	path.append(approach_paths[i + 1].robot_path);

	if (optimize_segments) {
		path = optimize(path, {nullptr}, si);
	}
	return path;
}

template
class ShellPathPlanner<Eigen::Vector3d>;