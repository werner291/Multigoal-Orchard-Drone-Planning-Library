// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#ifndef MGODPL_SHELLPATHPLANNER_H
#define MGODPL_SHELLPATHPLANNER_H

#include <cstddef>
#include <vector>
#include <functional>
#include <optional>

class ShellPoint;
namespace mgodpl {

	template<typename ShellSpace>
	struct ShellSpaceTraits;

	template<typename Path>
	struct PathTraits;

	/**
	 * @brief A segment of a path leading to a goal identifier.
	 *
	 * @tparam Path 		The type of the path.
	 * @tparam GoalId 	The type of the goal identifier. (Default: std::size_t, a vector index)
	 */
	template<typename Path, typename GoalId = std::size_t>
	struct PathToGoal {
		Path path; //< The path. The last state of the path must fulfill the goal condition.
		GoalId goal; //< The goal identifier, referring to the index of the goal in the list of goals given to the planner.
	};

	/**
	 * @brief The result of a planning query.
	 * @tparam Path 		The type of the path.
	 * @tparam GoalId 	The type of the goal identifier. (Default: std::size_t, a vector index)
	 *
	 * The result consists of a set of path segments from the start state to each of the goals.
	 */
	template<typename Path, typename GoalId = std::size_t>
	struct PlanResult {
		std::vector<PathToGoal<Path, GoalId>> segments;
	};

	/**
	 * @brief A path that starts at an arbitrary configuration and ends at a shell point.
	 *
	 * @tparam Path 			The type of the path.
	 * @tparam ShellPoint 		The type of the shell point.
	 */
	template<typename Path, typename ShellPoint>
	struct ApproachPath {
		Path path; //< A path, assumed to start at a shell point and end at some goal implicit from the context.
		ShellPoint shell_point; //< The shell point.
	};

	/**
	 * @brief A function that plans an approach path from a shell space to a goal, or nullopt if the path cannot be planned.
	 */
	template<typename Path, typename Shell, typename Goal> using PlanShellRetractionPathFn =
			std::function<std::optional<ApproachPath<Path, typename ShellSpaceTraits<Shell>::ShellPoint>>(const Shell &, Goal)>;

	/**
	 * @brief a pair of approach planning functions that, given a shell space, plan an approach path from a shell point to either a goal or a specific configuration.
	 *
	 * @tparam Path
	 * @tparam Shell
	 * @tparam Goal
	 */
	template<typename Path, typename Shell, typename Goal>
	struct ApproachPlanningFns {

		PlanShellRetractionPathFn<Path, Shell, Goal> plan_shell_retraction_path;

		PlanShellRetractionPathFn<Path, Shell, typename PathTraits<Path>::Configuration> plan_shell_retraction_path_from_configuration;

	};

	/**
	 * @brief A function that optimizes a path, keeping the start and end states fixed.
	 */
	template<typename Path> using LocalOptFn = std::function<Path(Path)>;

	/**
	 * @brief an ApproachPath paired with a goal.
	 *
	 * @tparam Path 			The type of the path.
	 * @tparam ShellPoint 		The type of the shell point.
	 * @tparam Goal 			The type of the goal.
	 */
	template<typename Path, typename ShellPoint, typename Goal>
	struct ApproachToGoal {

		ApproachPath<Path, ShellPoint> approach_path;

		Goal goal;
	};

	/**
	 * @brief Plan a set of approach paths to a set of goals.
	 *
	 * @tparam Path 			The type of the path.
	 * @tparam Shell 			The type of the shell.
	 * @tparam Goal 			The type of the goal.
	 *
	 * @param goals 					The goals.
	 * @param approach_planning_fns 	The approach planning functions.
	 * @param shell 					The shell.
	 * @return 	A vector of approach paths, one for each goal if the path could be planned, or an empty vector if the path could not be planned.
	 * 			Each approach path is paired with the index of the goal it leads to.
	 */
	template<typename Path, typename Shell, typename Goal>
	std::vector<ApproachToGoal<Path, typename ShellSpaceTraits<Shell>::ShellPoint, size_t>>
	plan_approach_paths(const std::vector<Goal> &goals,
						const ApproachPlanningFns<Path, Shell, Goal> &approach_planning_fns,
						const Shell &shell) {

		std::vector<ApproachToGoal<Path, typename ShellSpaceTraits<Shell>::ShellPoint, Goal>> result;

		for (const auto &goal: goals) {
			auto approach_path = approach_planning_fns.plan_shell_retraction_path(shell, goal);
			if (approach_path) {
				result.push_back({*approach_path, goal});
			}
		}

		return result;

	}


	/**
	 * @brief Compute the visitation order of a set of points, given a shell space and a start point.
	 *
	 * @tparam Shell 			The type of the shell.
	 *
	 * @param shell 			The shell.
	 * @param start 			The start point.
	 * @param points 			The points to visit.
	 * @return std::vector<size_t> 	The visitation order, as a vector of indices into the points vector.
	 */
	template<typename Shell>
	std::vector<size_t> compute_visitation_order(const Shell &shell,
												 const typename ShellSpaceTraits<Shell>::ShellPoint &start,
												 const std::vector<typename ShellSpaceTraits<Shell>::ShellPoint> &points) {

		// Combine into a single vector of shell points to feed it to the compute_distance_matrix function.
		std::vector<ShellPoint> shell_points{start};
		shell_points.insert(shell_points.end(), points.begin(), points.end());

		// Compute the distance matrix in batch form, as this is far more efficient than computing it one by one.
		auto distance_matrix = ShellSpaceTraits<Shell>::compute_distance_matrix(shell, shell_points);

		// Step 3.3: Compute the visitation order by calling out to the given algorithm.
		auto ordering = tsp_open_end([&](auto i) {
			// Use the shell path length as the heuristic.
			return distance_matrix[0][i + 1];
		}, [&](auto i, auto j) {
			// Use the shell path length as the heuristic.
			return distance_matrix[i + 1][j + 1];
		}, points.size());

		// Return the ordering. Note that the lambdas in tsp_open_end already take into account the fact that the
		// first point is the start point, so we don't need to do anything special here.
		return ordering;
	}


	template<typename Path, typename ShellSpace>
	PlanResult<Path> compose_paths(const LocalOptFn<Path> &local_opt_fn,
								   ShellSpace shell,
								   ApproachToGoal<Path, typename ShellSpaceTraits<ShellSpace>::ShellPoint, typename PathTraits<Path>::Configuration> initial_approach_path,
								   std::vector<ApproachToGoal<Path, typename ShellSpaceTraits<ShellSpace>::ShellPoint, size_t> > approach_paths,
								   const std::vector<size_t> &ordering) {// Step 4: Stitch the paths together into a result.
		PlanResult<Path> result;

		/*
		 * Possible optimization: Could std::move the paths out of the approach_paths vector
		 * and the initial approach path, since we don't need them anymore.
		 * However, this requires us to carefully manage the lifetimes of the paths,
		 * which is a bit tricky, so we don't do it for now as this isn't a hot path.
		 */

		// Step 4.1: Stitch the initial approach path to the first approach path, via the shell.
		{

			// Compute the shell path between the shell points of the initial approach path and the first approach path.
			auto sp1 = initial_approach_path.approach_path.shell_point;
			auto sp2 = approach_paths[ordering[0]].approach_path.shell_point;
			const auto& shell_path = ShellSpaceTraits<ShellSpace>::shell_path(sp1, sp2);

			// Reverse the initial approach path (noting that, since it's an approach path,
			// it starts at the shell and ends at the initial configuration, and therefore
			// we need to reverse it to get the path from the initial configuration to the shell).
			auto i_rev = PathTraits<Path>::reverse(initial_approach_path.approach_path.path);

			// Look up the first approach path.
			const auto approach = approach_paths[ordering[0]].approach_path.path;

			// Concatenate the paths together and local-optimize them.
			Path initial_path = local_opt_fn(PathTraits<Path>::concatenate({std::move(i_rev), shell_path, approach}));

			// Add the path to the result.
			result.paths.push_back({initial_path, approach_paths[ordering[0]].goal});
		}

		// Step 4.2: Stitch the remaining approach paths together.
		for (size_t i = 0; i + 1 < approach_paths.size(); ++i) {

			// Reverse the last approach path, to get back to the shell.
			auto retreat = PathTraits<Path>::reverse(approach_paths[ordering[i]].approach_path.path);

			// Compute the shell path between the shell points of the last approach path and the next approach path.
			auto sp1 = approach_paths[ordering[i]].approach_path.shell_point;
			auto sp2 = approach_paths[ordering[i + 1]].approach_path.shell_point;
			const auto& shell_path = ShellSpaceTraits<ShellSpace>::shell_path(sp1, sp2);

			// Look up the next approach path.
			auto approach = approach_paths[ordering[i + 1]].approach_path.path;

			// Concatenate the paths together and local-optimize them.
			Path goal_to_goal = local_opt_fn(PathTraits<Path>::concatenate({std::move(retreat), shell_path, approach}));

			// Add the path to the result.
			result.paths.push_back({goal_to_goal, approach_paths[ordering[i + 1]].goal});
		}

		// Return the result.
		return result;
	}

	/**
	 * @brief Plan a path while attempting to visit all goals. Implementation is based on the paper
	 *        "A fast two-stage approach for multi-goal path planning in a fruit tree" by Kroneman et al (ICRA 2023).
	 *
	 * @tparam Goal 			Type representing the goal.
	 * @tparam Path 			Type representing the path.
	 * @tparam ShellSpace		Type representing the shell space.
	 *
	 * @param start 					The start configuration.
	 * @param goals 					The set of goals to be visited.
	 * @param approach_planning_fns 	The approach planning functions.
	 * @param local_opt_fn 				The local optimization function.
	 * @param shell 					The shell space.
	 * @return The planned result containing the path segments leading to each goal.
	 */
	template<typename Goal, typename Path, typename ShellSpace>
	PlanResult<Path, Goal> plan(const typename PathTraits<Path>::Configuration &start,
								const std::vector<Goal> &goals,
								const ApproachPlanningFns<Path, ShellSpace, Goal> &approach_planning_fns,
								const LocalOptFn<Path> &local_opt_fn,
								ShellSpace shell) {

		// Step 1: Plan a path from the start to the shell.
		auto initial_approach_path = approach_planning_fns.plan_shell_retraction_path(shell, start);

		// Step 2: Plan a path from the shell to each goal.
		auto approach_paths = plan_approach_paths(goals, approach_planning_fns, shell);

		// If the approach paths are empty, return an empty result.
		if (approach_paths.empty()) {
			return {};
		}

		// Step 3: Compute the visitation order.

		// Step 3.1: Extract the shell points from the approach paths, including the initial approach path.
		std::vector<ShellPoint> shell_points{
				initial_approach_path.approach_path.shell_point // The shell point of the initial approach path.
		};

		for (const auto &approach_path: approach_paths) {
			shell_points.push_back(approach_path.approach_path.shell_point);
		}

		// Step 3.2: Compute the visitation order by calling out to the given algorithm.
		auto ordering = compute_visitation_order(shell, initial_approach_path.approach_path.shell_point, shell_points);

		return compose_paths(local_opt_fn, shell, initial_approach_path, approach_paths, ordering);

	}

} // mgodpl

#endif //MGODPL_SHELLPATHPLANNER_H
