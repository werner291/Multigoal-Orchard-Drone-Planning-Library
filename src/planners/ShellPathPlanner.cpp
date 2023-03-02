#include <range/v3/view/filter.hpp>
#include "ShellPathPlanner.h"
#include "../shell_space/CuttingPlaneConvexHullShell.h"

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

	// Build the shell space based on the planning scene.
	// In most cases, this is a sphere or a convex hull or something around the obstacles.
	auto shell = shell_builder(planning_scene, si);

	// Allocate the result.
	PlanResult result{{}};

	// Compute the initial approach path from the start state to the shell.
	std::optional<OmplApproachPath<ShellPoint>> initial_approach = methods->approach_path(start, *shell);

	// If the initial approach path computation failed, return an empty result.
	if (!initial_approach) {
		return result;
	}

	std::vector<std::pair<size_t, OmplApproachPath<ShellPoint>>> approach_paths;

	for (size_t i = 0; i < goals.size(); ++i) {
		std::cout << "Planning approach for goal " << i << std::endl;
		auto approach_path_opt = methods->approach_path(goals[i], *shell);
		if (approach_path_opt.has_value()) {
			assert(approach_path_opt->robot_path.getStateCount() > 0);
			approach_paths.emplace_back(i, approach_path_opt.value());
		}
	}

	// If there are no reachable goals, return an empty result.
	if (approach_paths.empty()) {
		return result;
	}

	std::cout << "Planned approach paths: " << approach_paths.size() << std::endl;

	// Determine the order in which to visit the goals.
	{
		// Use the TSP solver from OR-Tools to find the optimal ordering of the approach paths.
		auto ordering = tsp_open_end([&](auto i) {
			return shell->predict_path_length(initial_approach->shell_point, approach_paths[i].second.shell_point);
		}, [&](auto i, auto j) {
			return shell->predict_path_length(approach_paths[i].second.shell_point, approach_paths[j].second.shell_point);
		}, approach_paths.size());

		// Reorder the approach paths according to the optimal ordering.
		std::vector<std::pair<size_t,OmplApproachPath<ShellPoint>>> ordered_approaches;
		for (auto j: ordering) {
			ordered_approaches.push_back(approach_paths[j]);
		}
		// TODO: This line has actually crashed the program before, but it suddenly stopped for no clear reason.
		//  I suspect it was corrupted by the compiler somehow, but I suppose we'll never know.
		approach_paths = ordered_approaches;
	}

	// Build the initial approach path from the start state to the first goal.
	ompl::geometric::PathGeometric path = buildInitialApproach(si,
															   *shell,
															   initial_approach,
															   approach_paths.front().second);

	result.segments.push_back(MultiGoalPlanner::PathSegment{approach_paths.front().first, path});

	// Build the paths between each pair of consecutive goals.
	for (size_t i = 0; i + 1 < approach_paths.size(); i++) {
		path = buildGoalToGoal(si, *shell, approach_paths[i].second, approach_paths[i+1].second);
		assert(path.getStateCount() > 0); // TODO remove this once we find out why it sometimes fails
		result.segments.push_back(MultiGoalPlanner::PathSegment{approach_paths[i+1].first, path});
	}

	return result;
}


template<typename ShellPoint>
ompl::geometric::PathGeometric
ShellPathPlanner<ShellPoint>::buildInitialApproach(const ompl::base::SpaceInformationPtr &si,
												   const OmplShellSpace<ShellPoint> &shell,
												   const std::optional<OmplApproachPath<ShellPoint>> &initial_approach,
												   const OmplApproachPath<ShellPoint>& approach_to_first_goal) const {

	// Build the initial approach path from the start state to the first goal.
	// To do so, we will build a concatenation of several paths.

	// First, we will reverse the initial approach path, since by default it goes from the shell to the start state,
	// but we want to go from the start state to the shell.
	ompl::geometric::PathGeometric path(si);
	path.append(initial_approach->robot_path);
	path.reverse();

	// Then, append the shell path from the shell point of the initial approach path to the shell point of the first goal.
	shell.shellPath(initial_approach->shell_point, approach_to_first_goal.shell_point);

	// Finally, append the approach path of the first goal.
	path.append(approach_to_first_goal.robot_path);

	// Locally optimize the path if necessary.
	if (optimize_segments) {
		path = optimize(path, {nullptr}, si);
	}

	// Return the path.
	return path;
}

template<typename ShellPoint>
ompl::geometric::PathGeometric ShellPathPlanner<ShellPoint>::buildGoalToGoal(const ompl::base::SpaceInformationPtr &si,
																			 const OmplShellSpace<ShellPoint> &shell,
																			 const OmplApproachPath<ShellPoint> &approach_path_1,
																			 const OmplApproachPath<ShellPoint> &approach_path_2) const {

	// Build the path from the i-th goal to the (i+1)-th goal.

	// We will do this by building a concatenation of several paths.
	ompl::geometric::PathGeometric path(si);

	// First, the reverse of the approach path of the i-th goal, since by default it goes from the shell to the goal,
	// but we want to go from the goal to the shell.
	path.append(approach_path_1.robot_path);
	path.reverse();

	// Then, the shell path from the shell point of the i-th goal to the shell point of the (i+1)-th goal.
	shell.shellPath(approach_path_1.shell_point, approach_path_2.shell_point);

	// Finally, the approach path of the (i+1)-th goal.
	path.append(approach_path_2.robot_path);

	// Locally optimize the path if necessary.
	if (optimize_segments) {
		path = optimize(path, {nullptr}, si);
	}

	// Return the path.
	return path;
}

// Explicitly instantiate the planner for the shell point type Eigen::Vector3d (used with the sphere).
template
class ShellPathPlanner<Eigen::Vector3d>;

template
class ShellPathPlanner<ConvexHullPoint>;