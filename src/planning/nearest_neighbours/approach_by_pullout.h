// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/5/24.
//

#ifndef MGODPL_APPROACH_BY_PULLOUT_H
#define MGODPL_APPROACH_BY_PULLOUT_H

#include <optional>
#include "../ApproachPath.h"
#include "../fcl_forward_declarations.h"
#include "../cgal_chull_shortest_paths.h"
#include "../../math/Vec3.h"
#include "../RobotModel.h"
#include "../RandomNumberGenerator.h"
#include "../goal_sampling.h"
#include "../collision_detection.h"
#include "../probing_motions.h"

namespace mgodpl::approach_planning {

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
	std::optional<ApproachPath> plan_approach_by_pullout(const fcl::CollisionObjectd &obstacle,
														 const cgal::CgalMeshData &chull_shell,
														 const math::Vec3d &target_point,
														 double distance_from_target,
														 const robot_model::RobotModel &robot,
														 random_numbers::RandomNumberGenerator &rng,
														 const size_t max_goal_samples);

}

#endif //MGODPL_APPROACH_BY_PULLOUT_H
