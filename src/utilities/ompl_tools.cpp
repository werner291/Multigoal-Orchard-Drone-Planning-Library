// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <ompl/base/ScopedState.h>
#include "ompl_tools.h"


void utilities::truncatePathToInterrupt(ompl::geometric::PathGeometric &path, const PathInterrupt &at_interrupt) {

	assert(path.getStateCount() > at_interrupt.segment_index + 1);

	ompl::geometric::PathGeometric new_path(path.getSpaceInformation());

	for (size_t i = at_interrupt.segment_index; i < path.getStateCount(); i++) {
		new_path.append(path.getState(i));
	}

	path.getStates().erase(path.getStates().begin(), path.getStates().begin() + (int) at_interrupt.segment_index);

	ompl::base::ScopedState new_start_interpolated(path.getSpaceInformation());

	new_path.getSpaceInformation()
			->getStateSpace()
			->interpolate(new_path.getState(0),
						  new_path.getState(1),
						  at_interrupt.to_next_waypoint_interpolation,
						  new_start_interpolated.get());

	// Copy into the front of the path.
	new_path.getSpaceInformation()->getStateSpace()->copyState(new_path.getState(0), new_start_interpolated.get());

	path = new_path;

}

bool
utilities::pathStartMatches(const ompl::base::State *start, const ompl::geometric::PathGeometric &path, double margin) {
	return path.getSpaceInformation()->distance(start, path.getState(0)) < margin;
}
