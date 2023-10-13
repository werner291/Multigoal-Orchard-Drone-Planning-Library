// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file approach_planning.h
 *
 * @brief This file contains traits and functions for approach planning, i.e. planning
 * from a shell configuration space to a goal region.
 */

#ifndef MGODPL_APPROACH_PLANNING_H
#define MGODPL_APPROACH_PLANNING_H

#include <optional>
#include <functional>
#include "ShellConfigurationSpace.h"
#include "goal_region.h"

namespace mgodpl::approach_planning {

	using namespace mgodpl::shell_configuration_space;
	using namespace mgodpl::goal_region;

	/**
	 * @brief Find a pair of closest configurations between a shell and a goal region.
	 */
	template<typename ShellSpace, typename GoalRegion>
	std::pair<typename shell_configuration_space::internal_point_t<ShellSpace>::type,
			typename configuration_space::configuration_t<GoalRegion>::type> find_closest_configurations(
			const ShellSpace& shell_configuration_space,
			const GoalRegion& goal_region);

	/**
	 * @brief Find a pair of closest configurations between a shell and a goal region.
	 */
	template<typename ShellSpace, typename GoalRegion>
	typename shell_configuration_space::internal_point_t<ShellSpace>::type find_closest_shellpoint(
			const ShellSpace& shell_configuration_space,
			const GoalRegion& goal_region);

	/**
	 * Type definition for a batch approach planner function.
	 *
	 * This function type represents a batch approach planner that takes a vector of goal regions and a shell space,
	 * and produces a vector of optional robot paths. Each entry in the vector corresponds to an element in the scene.
	 * If an element is reachable, the entry contains a robot path from the current robot state to the element;
	 * otherwise, the entry is nullopt.
	 *
	 * @tparam ShellSpace The type representing the shell space around the elements.
	 * @tparam GoalRegion The type representing a goal region for an element.
	 * @param goal_regions A vector of goal regions, each representing an element in the scene.
	 * @param shell_space A shell space around the elements.
	 * @return A vector of optional robot paths, each corresponding to an element in the scene.
	 */
	template<typename ShellSpace, typename GoalRegion, typename Path>
	using BatchApproachPlannerFn =
			std::function<std::vector<std::optional<Path>>(
					const std::vector<GoalRegion> &goal_regions,
					const ShellSpace &
			)>;

	/**
	 * Type definition for an individual approach planner function.
	 *
	 * This function type represents an individual approach planner that takes a goal region and a shell space,
	 * and produces an optional robot path. If an element is reachable, the function returns a robot path from the
	 * current robot state to the element; otherwise, it returns nullopt.
	 *
	 * @tparam ShellSpace The type representing the shell space around the elements.
	 * @tparam GoalRegion The type representing a goal region for an element.
	 * @param goal_region The goal region representing an element in the scene.
	 * @param shell_space A shell space around the elements.
	 * @return An optional robot path if the element is reachable; otherwise, nullopt.
	 */
	template<typename ShellSpace, typename GoalRegion, typename Path>
	using ApproachPlannerFn =
			std::function<std::optional<Path>(const GoalRegion &, const ShellSpace &)>;

	/**
	 * A function that adapts an individual approach planner to a batch approach planner.
	 *
	 * This function takes an individual approach planner function and returns a batch approach planner function.
	 *
	 * @tparam ShellSpace The type representing the shell space around the elements.
	 * @tparam GoalRegion The type representing a goal region for an element.
	 * @param individual_planner The individual approach planner function to adapt.
	 * @return A batch approach planner function that operates on goal regions and a shell space, producing paths for each element.
	 */
	template<typename ShellSpace, typename GoalRegion, typename Path>
	BatchApproachPlannerFn<ShellSpace, GoalRegion, Path> adapt_individual_planner_to_batch(
			ApproachPlannerFn<ShellSpace, GoalRegion, Path> individual_planner) {
		/**
		 * @param goal_regions A vector of goal regions, each representing an element in the scene.
		 * @param shell_space A shell space around the elements.
		 * @return A vector of optional robot paths, each corresponding to an element in the scene.
		 */
		return [individual_planner](const std::vector<GoalRegion> &goal_regions, const ShellSpace &shell_space) -> std::vector<std::optional<Path>> {
			std::vector<std::optional<Path>> paths(goal_regions.size());

			for (size_t i = 0; i < goal_regions.size(); ++i) {
				paths[i] = individual_planner(goal_regions[i], shell_space);
			}

			return paths;

		};
	}
}

#endif //MGODPL_APPROACH_PLANNING_H
