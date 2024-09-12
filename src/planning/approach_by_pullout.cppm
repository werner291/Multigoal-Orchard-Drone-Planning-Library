// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/5/24.
//

module;

#include <optional>
#include "ApproachPath.h"
#include "fcl_forward_declarations.h"
#include "cgal_chull_shortest_paths.h"
#include "../math/Vec3.h"
#include "RobotModel.h"
#include "RandomNumberGenerator.h"
#include "goal_sampling.h"
#include "collision_detection.h"
#include "probing_motions.h"

export module approach_by_pullout;

export namespace mgodpl::approach_planning {

	struct ApproachByPulloutHooks {
		/// A hook that is called when a sample is taken,
		/// called with the state that was just sampled
		/// and whether it is collision free.
		std::function<void(const RobotState& state, bool collision_free)> sampled_state;

		/// A hook that is called when a pullout motion was considered
		std::function<void(const ApproachPath& path, bool collision_free)> pullout_motion_considered;
	};

	/**
	 * @brief Plans an approach path for a robot to reach a target point while avoiding a given obstacle.
	 * @param obstacle The obstacle to avoid.
	 * @param chull_shell The convex hull shell of the obstacle.
	 * @param target_point The target point to reach.
	 * @param distance_from_target The desired distance from the target point.
	 * @param robot The robot model.
	 * @param rng A random number generator.
	 * @param max_goal_samples The maximum number of goal samples to take.
	 * @return An optional approach path. If a path is found, it is returned. If no path is found, std::nullopt is returned.
	 */
	std::optional<ApproachPath> plan_approach_by_pullout(
			 const fcl::CollisionObjectd &obstacle,
			 const cgal::CgalMeshData &chull_shell,
			 const math::Vec3d &target_point,
			 double distance_from_target,
			 const robot_model::RobotModel &robot,
			 random_numbers::RandomNumberGenerator &rng,
			 const size_t max_goal_samples,
			 std::optional<ApproachByPulloutHooks> hooks = std::nullopt) {

		// Get the robot's base and end effector links:
		const auto &base_link = robot.findLinkByName("flying_base");
		const auto &end_effector_link = robot.findLinkByName("end_effector");

		// Take 1000 goal samples:
		for (size_t i = 0; i < max_goal_samples; ++i) {

			auto sample = mgodpl::genGoalStateUniform(
					rng,
					target_point,
					distance_from_target,
					robot,
					base_link,
					end_effector_link
			);

			bool collision_free = !check_robot_collision(robot, obstacle, sample);

			if (hooks) hooks->sampled_state(sample, collision_free);

			// Check collisions:
			if (collision_free) {
				const auto &path = straightout(
						robot,
						sample,
						chull_shell.tree,
						chull_shell.mesh_path
				);

				bool path_collision_free = !check_path_collides(robot, obstacle, path.path);

				if (hooks) hooks->pullout_motion_considered(path, path_collision_free);

				if (path_collision_free) {
					return path;
				}
			}
		}

		return std::nullopt;
	}

}

