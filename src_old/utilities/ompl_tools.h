// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_OMPL_TOOLS_H
#define NEW_PLANNERS_OMPL_TOOLS_H

#include <ompl/geometric/PathGeometric.h>
#include "../PathInterrupt.h"

namespace utilities {

	/**
	 * Truncate a path to the given interrupt.
	 *
	 * @param path 				The path to truncate (in place).
	 * @param at_interrupt 		The interrupt to truncate to.
	 */
	void truncatePathToInterrupt(ompl::geometric::PathGeometric &path, const PathInterrupt &at_interrupt);

	/**
	 * Return whether the given path starts at the given state.
	 *
	 * @param start 		The state to check.
	 * @param path 			The path to check.
	 * @param margin 		The margin of error.
	 * @return 				True if the path starts at the given state, false otherwise.
	 */
	bool pathStartMatches(const ompl::base::State *start,
						  const ompl::geometric::PathGeometric &path,
						  double margin = 1.0e-6);

}

#endif //NEW_PLANNERS_OMPL_TOOLS_H
