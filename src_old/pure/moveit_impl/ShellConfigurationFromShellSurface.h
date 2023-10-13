// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 24-6-23.
//

#ifndef MGODPL_SHELLCONFIGURATIONFROMSHELLSURFACE_H
#define MGODPL_SHELLCONFIGURATIONFROMSHELLSURFACE_H

#include <moveit/robot_model/robot_model.h>

namespace mgodpl {

	namespace moveit_impl {

		template<typename Shell>

		struct ShellConfigurationFromShellSurface {

			using InternalPoint = typename shell_point_t<Shell>::type;
			using Configuration = moveit::core::RobotState;

			const moveit::core::RobotModelConstPtr robot_model;

			const moveit::core::LinkModel *end_effector;

			const Shell shell;

		};

	}

	namespace shell_configuration_space {
		/**
		 * @brief Get the internal point type.
		 */
		template<typename Shell>
		struct internal_point_t<moveit_impl::ShellConfigurationFromShellSurface<Shell>> {
			using type = typename shell_point_t<Shell>::type;
		};

		/**
		 * @brief Get the path type, defined in terms of the internal point type.
		 */
		template<typename ShellConfigurationSpace>
		struct path_t {
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

}


#endif //MGODPL_SHELLCONFIGURATIONFROMSHELLSURFACE_H
