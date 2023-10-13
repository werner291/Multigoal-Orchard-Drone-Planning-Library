// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#ifndef NEW_PLANNERS_APPLE_STATUS_COLOR_CODING_H
#define NEW_PLANNERS_APPLE_STATUS_COLOR_CODING_H

#include <array>
#include <vtkActor.h>
#include "utilities/goal_events.h"

const std::array<double, 3> APPLE_VISITED_COLOR = {0.0, 0.0, 1.0};
const std::array<double, 3> APPLE_UNKNOWN_COLOR = {1.0, 0.0, 1.0};
const std::array<double, 3> APPLE_KNOWN_COLOR = {1.0, 0.0, 0.0};
const std::array<double, 3> APPLE_FALSE_COLOR = {1.0, 1.0, 0.0};
const std::array<double, 3> APPLE_REMOVED_COLOR = {0.0, 0.0, 0.0};

/**
 * Get the color constant that corresponds to the given discovery status.
 *
 * @param status The discovery status of an apple.
 * @return The color constant that corresponds to the given discovery status.
 */
std::array<double, 3> getColorForDiscoveryStatus(const utilities::DiscoveryStatus &status);

/**
 * Update the colors of the given apple actors based on the discovery status of each apple.
 *
 * @param discovery_statuses A vector of discovery statuses of the apples.
 * @param apple_actors A vector of apple actors whose colors will be updated.
 */
void updateAppleColors(const std::vector<utilities::DiscoveryStatus> &discovery_statuses,
					   const std::vector<vtkActor *> &apple_actors);

#endif //NEW_PLANNERS_APPLE_STATUS_COLOR_CODING_H
