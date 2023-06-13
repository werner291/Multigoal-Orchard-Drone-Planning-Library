// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#ifndef MGODPL_SHELLCONFIGURATIONSPACE_H
#define MGODPL_SHELLCONFIGURATIONSPACE_H

#include "Shell.h"

namespace mgodpl {

	template<typename ShellConfigurationSpace>
	struct configuration_t {
		using type = typename ShellConfigurationSpace::ConfigurationSpace::Configuration;
	};

	/**
	 * @brief Given a shell point on a shell, return the configuration at that point.
	 *
	 * @tparam Shell The type representing the shell.
	 * @tparam ShellConfigurationSpace The type representing the shell configuration space.
	 *
	 * @param shell A reference to the shell.
	 * @param space A reference to the shell configuration space.
	 * @param point A shell point within the shell.
	 *
	 * @return The configuration at the given shell point.
	 */
	template<typename Shell, typename ShellConfigurationSpace>
	typename configuration_t<ShellConfigurationSpace>::type configuration_at_shell_point(const Shell& shell,
																						 const ShellConfigurationSpace& space,
																						 const typename shell_point_t<Shell>::type& point);

	/**
	 * @brief Given a configuration, return a shell point on the shell whose configuration is close.
	 *
	 * @tparam Shell The type representing the shell.
	 * @tparam ShellConfigurationSpace The type representing the shell configuration space.
	 *
	 * @param shell A reference to the shell.
	 * @param space A reference to the shell configuration space.
	 * @param configuration The configuration for which to find a close shell point.
	 *
	 * @return A shell point on the shell whose configuration is close to the given configuration.
	 */
	template<typename Shell, typename ShellConfigurationSpace>
	typename shell_point_t<Shell>::type shell_point_at_configuration(const Shell& shell,
																	 const ShellConfigurationSpace& space,
																	 const typename configuration_t<ShellConfigurationSpace>::type& configuration);

}

#endif //MGODPL_SHELLCONFIGURATIONSPACE_H
