// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/5/24.
//

#ifndef MGODPL_SHELL_PATH_PLANNING_H
#define MGODPL_SHELL_PATH_PLANNING_H

// Forward declarations:
#include <vector>
#include <functional>
#include <optional>
#include "../math/Vec3.h"
#include "fcl_forward_declarations.h"
#include "RandomNumberGenerator.h"
#include "visitation_order.h"

namespace mgodpl {

	struct RobotState;

	struct RobotPath;

	namespace robot_model {
		class RobotModel;
	}

	namespace cgal {
		struct CgalMeshData;
	}

	namespace shell_path_planning {

		struct PlanMultigoalPathHooks {

		};

		using VisitationOrderFn = std::function<std::vector<size_t>(std::vector<std::vector<double>>, std::vector<double>)>;

		/**
		 * A struct containing the various sub-strategy functions for plan_multigoal_path.
		 */
		struct PlanMultigoalPathMethods {
			VisitationOrderFn compute_visitation_order = visitation_order_greedy;
		};

		/**
		* A start-to-finish implementation of shell path planning, which takes an initial state,
		* an obstacle, a set of target points, and a robot model, and returns a path that
		* the robot can follow to reach all target points.
		*
		* In this case, it is assumed that all target points are available ahead of time, without dynamic goal sets.
		*
		* @param initial_state			The initial state of the robot.
		* @param obstacle				The obstacle that the robot must avoid.
		* @param chull_shell			The convex hull shell of the obstacle.
		* @param target_points			The target points that the robot must reach.
		* @param distance_from_target 	The distance from the target points that the robot must reach.
		* @param robot					The robot model.
		* @param rng					A random number generator; state will be modified as numbers are drawn.
		* @param at_goal_hook			A hook that is called when the robot reaches a target point.
		* @param hooks					Hooks for the path planning algorithm.
		*
		* @return A path that the robot can follow to reach all target points.
		*/
		RobotPath plan_multigoal_path(const RobotState &initial_state,
									  const fcl::CollisionObjectd &obstacle,
									  const cgal::CgalMeshData &chull_shell,
									  const std::vector<math::Vec3d> &target_points,
									  double distance_from_target,
									  const robot_model::RobotModel &robot,
									  random_numbers::RandomNumberGenerator &rng,
									  const PlanMultigoalPathMethods &methods = {},
									  const std::optional<std::function<RobotPath(const math::Vec3d target_point, const RobotState &state)>> &at_goal_hook = std::nullopt,
									  const std::optional<PlanMultigoalPathHooks> &hooks = std::nullopt);

	}
}

#endif //MGODPL_SHELL_PATH_PLANNING_H
