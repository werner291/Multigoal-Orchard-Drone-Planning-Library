// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3-5-23.
//

#include "midpoint_pull.h"

void midpointPull(const ompl::base::State *state1,
				  const ompl::base::State *state2,
				  const ompl::base::State *state3,
				  ompl::base::State *output,
				  const ompl::base::SpaceInformationPtr &si,
				  const double t) {
	ompl::base::ScopedState<> midpoint(si);

	// Compute the midpoint of the two states
	si->getStateSpace()->interpolate(state1, state3, 0.5, midpoint.get());

	// Interpolate between the midpoint and the second state
	si->getStateSpace()->interpolate(state2, midpoint.get(), t, output);
}

bool applyMidpointPullToPath(ompl::geometric::PathGeometric &path, int index, double t) {
	if (index < 1 || index >= path.getStateCount() - 1)
	{
		throw std::invalid_argument("Invalid index for midpoint pull operation");
	}

	ompl::base::SpaceInformationPtr spaceInfo = path.getSpaceInformation();
	ompl::base::State *state1 = path.getState(index - 1);
	ompl::base::State *state2 = path.getState(index);
	ompl::base::State *state3 = path.getState(index + 1);

	ompl::base::ScopedState<> newState(spaceInfo);
	midpointPull(state1, state2, state3, newState.get(), spaceInfo, t);

	// Check for motion validity between the new state and its neighbors
	if (!spaceInfo->checkMotion(state1, newState.get()) || !spaceInfo->checkMotion(newState.get(), state3)) {
		return false;
	}

	spaceInfo->copyState(state2, newState.get());

	return true;
}
