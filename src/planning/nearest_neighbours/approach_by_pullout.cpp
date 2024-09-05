// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/5/24.
//

#include "approach_by_pullout.h"

namespace mgodpl::approach_planning {
	std::optional<ApproachPath>
	plan_approach_by_pullout(const fcl::CollisionObjectd &obstacle,
						   const mgodpl::cgal::CgalMeshData &chull_shell,
						   const mgodpl::math::Vec3d &target_point,
						   double distance_from_target,
						   const mgodpl::robot_model::RobotModel &robot,
						   random_numbers::RandomNumberGenerator &rng,
						   const size_t max_goal_samples) {

		// Get the robot's base and end effector links:
		const auto &base_link = robot.findLinkByName("flying_base");
		const auto &end_effector_link = robot.findLinkByName("end_effector");

		// Take 1000 goal samples:
		for (size_t i = 0; i < max_goal_samples; ++i) {

			auto sample = mgodpl::genGoalStateUniform(
					rng,
					target_point,
					robot,
					base_link,
					end_effector_link
			);

			// Check collisions:
			if (!check_robot_collision(robot, obstacle, sample)) {
				const auto &path = straightout(
						robot,
						sample,
						chull_shell.tree,
						chull_shell.mesh_path
				);

				// Check collisions:
				if (!check_path_collides(robot, obstacle, path.path)) {
					return path;
				}
			}
		}

		return std::nullopt;
	}
}