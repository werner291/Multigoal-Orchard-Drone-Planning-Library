// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file configuration.h
 *
 * @brief This file contains traits and functions for configurations. In robotics, a emph{configuration is an assignment
 * of values to all joint variables, fully determining the spatial arrangement and orientation of a robot's parts without
 * ambiguity.
 */

#ifndef MGODPL_CONFIGURATION_H
#define MGODPL_CONFIGURATION_H

namespace mgodpl::configuration {

	/**
	 * @brief Compute a distance between configurations.
	 */
	template<typename Configuration>
	double distance(const Configuration& configuration1, const Configuration& configuration2);

}

#endif //MGODPL_CONFIGURATION_H
