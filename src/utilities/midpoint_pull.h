// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3-5-23.
//

#ifndef NEW_PLANNERS_MIDPOINT_PULL_H

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>

/**
 * @brief Perform the midpoint pull operation on a triplet of states, smoothing out that section of the path.
 *
 * @param[in] state1 First state in the triplet.
 * @param[in] state2 Second state in the triplet (the state to be "pulled").
 * @param[in] state3 Third state in the triplet.
 * @param[out] output The resulting state after applying the midpoint pull operation.
 * @param[in] si Shared pointer to the SpaceInformation, containing the state space and other relevant information.
 * @param[in] t Interpolation parameter t, within range [0, 1].
 *
 * @pre t is within range [0, 1].
 * @post output is the result of the midpoint pull operation, an interpolation between state2 and the midpoint of state1 and state3.
 *
 * @warning This function does not check for motion validity between the states.
 */
void midpointPull(const ompl::base::State *state1,
				 const ompl::base::State *state2,
				 const ompl::base::State *state3,
				 ompl::base::State *output,
				 const ompl::base::SpaceInformationPtr& si,
				 const double t);

/**
 * @brief Apply the midpoint pull operation to a path at a given index i, checking for motion validity.
 *
 * @param[in,out] path The path to be modified.
 * @param[in] index The index of the state in the path to be "pulled" (i.e., the state to be modified).
 * @param[in] t Interpolation parameter t, within range [0, 1]. 0 corresponds to the original state,
 * 			    1 to the midpoint of the states before and after the state to be modified.
 *
 * @return True if the operation was successful and the path was modified, false otherwise.
 *
 * @pre index is within range [1, path.getStateCount() - 2].
 * @pre t is within range [0, 1].
 * @post The path is modified only if the new path segments are valid, i.e., no collisions are introduced.
 */
bool applyMidpointPullToPath(ompl::geometric::PathGeometric &path, int index, double t);

#define NEW_PLANNERS_MIDPOINT_PULL_H

#endif //NEW_PLANNERS_MIDPOINT_PULL_H
