// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file ShellConfigurationSpace.h
 *
 * A shell configuration space is a subspace of the configuration space,
 * structured in such a way that it is easy to quickly find a path between
 * pairs of configurations in the subspace, and to find a configuration
 * close to a given configuration outside the space.
 *
 * Typically, a shell configuration space will know a concept of an "internal point",
 * which is a reduced, more efficient representation of the configurations in the space.
 *
 * Note: a shell configuration space is a configuration space; it inherits from that concept.
 */

#ifndef MGODPL_SHELLCONFIGURATIONSPACE_H
#define MGODPL_SHELLCONFIGURATIONSPACE_H

#include "Shell.h"

#include "configuration_space.h"

namespace mgodpl::shell_configuration_space {

	/**
	 * @brief Get the internal point type.
	 */
	template<typename ShellConfigurationSpace>
	struct internal_point_t {
		using type = typename ShellConfigurationSpace::InternalPoint;
	};

	/**
	 * @brief Get the path type, defined in terms of the internal point type.
	 */
	template<typename ShellConfigurationSpace>
	struct shell_path_t {
		using type = typename ShellConfigurationSpace::Path;
	};

	/**
	 * @brief Given an internal point, return the corresponding configuration.
	 */
	template<typename ShellConfigurationSpace>
	typename configuration_space::configuration_t<ShellConfigurationSpace>::type configuration_at_internal_point(
			const ShellConfigurationSpace& space,
			const typename internal_point_t<ShellConfigurationSpace>::type& internal_point);

	/**
	 * @brief Given a configuration, return a shell point on the shell whose configuration is close.
	 *
	 * @tparam ShellConfigurationSpace The type representing the shell configuration space.
	 *
	 * @param space A reference to the shell configuration space.
	 * @param configuration The configuration for which to find a close shell point.
	 *
	 * @return A shell point on the shell whose configuration is close to the given configuration.
	 */
	template<typename ShellConfigurationSpace>
	typename internal_point_t<ShellConfigurationSpace>::type shell_point_near_configuration(
			const ShellConfigurationSpace& space,
			const typename configuration_space::configuration_t<ShellConfigurationSpace>::type& configuration);

}

#endif //MGODPL_SHELLCONFIGURATIONSPACE_H
