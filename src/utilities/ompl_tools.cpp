// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <ompl/base/ScopedState.h>
#include "ompl_tools.h"


void utilities::truncatePathToInterrupt(ompl::geometric::PathGeometric &path, const PathInterrupt &at_interrupt) {
	;
	assert(path.getStateCount() >= at_interrupt.segment_index + 1);

	path.getStates().erase(path.getStates().begin(), path.getStates().begin() + (int) at_interrupt.segment_index);

	ompl::base::ScopedState new_start_interpolated(path.getSpaceInformation());

	path.getSpaceInformation()
			->getStateSpace()
			->interpolate(path.getState(0),
						  path.getState(1),
						  at_interrupt.to_next_waypoint_interpolation,
						  new_start_interpolated.get());

	// Copy into the front of the path.
	path.getSpaceInformation()->getStateSpace()->copyState(path.getState(0), new_start_interpolated.get());

}
