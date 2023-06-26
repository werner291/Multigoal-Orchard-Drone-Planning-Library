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

}

#endif //MGODPL_APPROACH_PLANNING_H
