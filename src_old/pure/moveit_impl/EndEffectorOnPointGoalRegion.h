// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 26-6-23.
//

#ifndef MGODPL_ENDEFFECTORONPOINTGOALREGION_H
#define MGODPL_ENDEFFECTORONPOINTGOALREGION_H

#include <Eigen/Core>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_state/robot_state.h>

#include "../configuration_space.h"
#include "../goal_region.h"


namespace mgodpl {

	namespace moveit_impl {

		/**
		 * A struct that represents a goal region for the end effector to be on a point.
		 *
		 * Note: assumes that variables 0,1,2 are the x,y,z coordinates of the base link, assuming a floating joint.
		 */
		struct EndEffectorOnPointGoalRegion {

			Eigen::Vector3d target_point;
			moveit::core::RobotModelConstPtr robot_model;
			moveit::core::LinkModel *end_effector;
			double max_distance;

		};

	}

	namespace configuration_space {

		template<>
		struct configuration_t<moveit_impl::EndEffectorOnPointGoalRegion> {
			using type = moveit::core::RobotState;
		};

	}

	namespace goal_region {

		template<>
		bool is_goal<moveit_impl::EndEffectorOnPointGoalRegion>(const moveit_impl::EndEffectorOnPointGoalRegion& goal_region, const moveit::core::RobotState& configuration) {

			// Compute the end_effector position.
			Eigen::Vector3d end_effector_position = configuration.getGlobalLinkTransform(goal_region.end_effector).translation();

			// Compute the distance between the end effector and the target point.
			double distance = (end_effector_position - goal_region.target_point).squaredNorm();

			// Return whether the distance is smaller than the maximum distance.
			return distance <= goal_region.max_distance * goal_region.max_distance;

		}

		template<>
		moveit::core::RobotState project<moveit_impl::EndEffectorOnPointGoalRegion>(const moveit_impl::EndEffectorOnPointGoalRegion& goal_region, const moveit::core::RobotState& configuration) {

			// Compute the end_effector position.
			Eigen::Vector3d end_effector_position = configuration.getGlobalLinkTransform(goal_region.end_effector).translation();

			// Compute the vector from the end effector to the target point.
			Eigen::Vector3d vector = goal_region.target_point - end_effector_position;

			// Apply to the base link.
			moveit::core::RobotState projected_configuration = configuration;
			projected_configuration.setVariablePosition(0, configuration.getVariablePosition(0) + vector.x());
			projected_configuration.setVariablePosition(1, configuration.getVariablePosition(1) + vector.y());
			projected_configuration.setVariablePosition(2, configuration.getVariablePosition(2) + vector.z());

			// Return the projected configuration.
			return projected_configuration;

		}

	}

}

#endif //MGODPL_ENDEFFECTORONPOINTGOALREGION_H
