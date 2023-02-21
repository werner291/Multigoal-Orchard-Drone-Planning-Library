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

}

#endif //NEW_PLANNERS_OMPL_TOOLS_H
