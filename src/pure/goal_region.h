// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file goal_region.h
 *
 * @brief This file contains some traits and functions for goal regions.
 *
 * @details A goal region is a subspace of the configuration space that is considered a goal.
 */

#ifndef MGODPL_GOAL_REGION_H
#define MGODPL_GOAL_REGION_H

#include "configuration_space.h"

namespace mgodpl::goal_region {

	/**
	 * @brief Test whether a configuration is a member of a goal region.
	 */
	template<typename GoalRegion>
	bool is_goal(const GoalRegion& goal_region, const typename configuration_space::configuration_t<GoalRegion>::type& configuration);

	/**
	 * @brief Project a configuration onto a goal region.
	 */
	template<typename GoalRegion>
	typename configuration_space::configuration_t<GoalRegion>::type project(const GoalRegion& goal_region, const typename configuration_space::configuration_t<GoalRegion>::type& configuration);

}

#endif //MGODPL_GOAL_REGION_H
