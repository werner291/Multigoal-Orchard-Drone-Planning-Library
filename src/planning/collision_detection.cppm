module;

#include <functional>
#include "collision_detection.h"

// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

export module collision_detection;

import functional_utils;

export namespace mgodpl {

	/**
	 * @brief A structure encapsulating the robot model and the collision object.
	 */
	struct CollisionEnvironment {
		const robot_model::RobotModel &robot; ///< The robot model
		const fcl::CollisionObjectd &tree_collision; ///< The collision object
	};

	using CollisionDetectionFn = std::function<bool (const RobotState &)>;
	using MotionCollisionDetectionFn = std::function<bool (const RobotState &, const RobotState &)>;

	/**
	 * @brief Creates a collision checking function for a given robot state.
	 *
	 * @param env The collision environment, which includes the robot model and the collision object.
	 * @return A function that takes a RobotState and checks for collisions in the given environment.
	 */
	CollisionDetectionFn collision_check_fn_in_environment(const CollisionEnvironment &env) {
		return [&](const RobotState &from) {
			return check_robot_collision(env.robot, env.tree_collision, from);
		};
	}

	/**
	 * @brief Creates a motion collision checking function for a given robot state.
	 *
	 * @param env The collision environment, which includes the robot model and the collision object.
	 * @return A function that takes two RobotState objects and checks for collisions during the motion in the given environment.
	 */
	MotionCollisionDetectionFn motion_collision_check_fn_in_environment(const CollisionEnvironment &env) {
		return [&](const RobotState &from, const RobotState &to) {
			return check_motion_collides(env.robot, env.tree_collision, from, to);
		};
	}

	/**
	 * @brief A structure that encapsulates two corresponding collision checking functions.
	 *
	 * Since these functions are so often used together, they are encapsulated in a single structure,
	 * whereby the motion_collision_check function is a continuous version of the collision_check function.
	 */
	struct CollisionFunctions {
		CollisionDetectionFn state_collides;
		MotionCollisionDetectionFn motion_collides;
	};

	/**
	 * @brief Creates a pair of collision checking functions for a given robot state.
	 *
	 * Note: these keep a reference to the environment, so the environment must outlive the returned functions.
	 *
	 * @param env The collision environment, which includes the robot model and the collision object.
	 * @return A CollisionFunctions object that contains both the collision checking function and the motion collision checking function.
	 */
	CollisionFunctions collision_functions_in_environment(const CollisionEnvironment &env) {
		return CollisionFunctions{
			collision_check_fn_in_environment(env),
			motion_collision_check_fn_in_environment(env)
		};
	}

	struct CollisionFunctionsCounts {
		calls_t &collision_check_invocations;
		calls_t &motion_collision_check_invocations;
	};

	/**
	 * @brief An invocation-counting version of `collision_functions_in_environment`.
	 *
	 * This is a convenience function that wraps the collision checking functions in a counting function.
	 */
	CollisionFunctions collision_functions_in_environment_counting(const CollisionFunctions &functions,
																   CollisionFunctionsCounts counts) {
		return CollisionFunctions{
			wrap_invocation_counting(functions.state_collides, counts.collision_check_invocations),
			wrap_invocation_counting(functions.motion_collides, counts.motion_collision_check_invocations)
		};
	}



}