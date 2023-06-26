// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file approach_planning.h
 * @brief Provides functionalities for approach planning in the context of the MGODPL system.
 *
 * This file contains the definition of the approach planning functionalities, including
 * the definition of functions for finding closest configurations between a shell and a goal region.
 *
 * @copyright (c) 2022 University College Roosevelt
 * @date 26-6-23
 * @author Werner
 */


#ifndef MGODPL_APPROACH_PLANNING_H
#define MGODPL_APPROACH_PLANNING_H

#include <utility>
#include "../ShellConfigurationSpace.h"
#include "../shell_space_impl/CGALMeshShell.h"
#include "EndEffectorOnPointGoalRegion.h"
#include "ShellConfigurationFromShellSurface.h"

namespace mgodpl::approach_planning {

	using namespace mgodpl::moveit_impl;

	/**
	 * @brief Find a pair of closest configurations between a shell and a goal region.
	 *
	 * This function takes in a shell configuration space and a goal region. It projects the target point of the
	 * goal region onto the shell and finds the shell state at the internal point. The shell state is then
	 * projected onto the goal state. The pair of the shell point and the goal state is returned.
	 *
	 * @tparam Shell Template parameter representing the shell configuration.
	 * @param shell_configuration_space The shell configuration space from which the shell state is to be extracted.
	 * @param goal_region The goal region containing the target point to be projected onto the shell.
	 * @return A pair containing the projected shell point and the corresponding goal state.
	 */
	template<typename Shell>
	std::pair<typename shell_point_t<Shell>::type, EndEffectorOnPointGoalRegion> find_closest_configurations(
			const ShellConfigurationFromShellSurface<Shell>& shell_configuration_space,
			const EndEffectorOnPointGoalRegion& goal_region) {

		auto sp = project_euclidean_to_shell(shell_configuration_space.shell, goal_region.target_point);

		auto shell_state = configuration_at_internal_point(shell_configuration_space, sp);

		auto goal_state = mgodpl::goal_region::project(shell_state);

		return std::make_pair(sp, goal_state);
	}

}

#endif //MGODPL_APPROACH_PLANNING_H
