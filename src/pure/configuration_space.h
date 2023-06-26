// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file configuration_space.h
 *
 * @brief This file contains traits and functions for configuration spaces.
 *
 * A configuration space is a space of configurations, i.e. a space of robot states.
 *
 * It is a continuous space, and may be finite or infinite.
 */

#ifndef MGODPL_CONFIGURATION_SPACE_H
#define MGODPL_CONFIGURATION_SPACE_H

namespace mgodpl::configuration_space {

	/**
	 * @brief Extract the configuration type.
	 */
	template<typename ConfigurationSpace>
	struct configuration_t {
		using type = typename ConfigurationSpace::Configuration;
	};

	/**
	 * @brief Sample a configuration from a goal region uniformly at random.
	 *
	 * Note: This generally only works on finite configuration spaces.
	 *
	 * TODO: Try quasi-random sampling at some point?
	 */
	template<typename ConfigurationSpace>
	typename configuration_space::configuration_t<ConfigurationSpace>::type sample_uniform(const ConfigurationSpace& configuration_space);
}

#endif //MGODPL_CONFIGURATION_SPACE_H
