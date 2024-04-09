// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#ifndef MGODPL_PROBING_MOTIONS_H
#define MGODPL_PROBING_MOTIONS_H

#include <CGAL/AABB_tree.h>
#include "../math/Vec3.h"
#include "RobotModel.h"
#include "ApproachPath.h"
#include "RobotState.h"
#include "cgal_chull_shortest_paths.h"
#include "fcl_forward_declarations.h"
#include "../experiment_utils/TreeMeshes.h"
#include "RandomNumberGenerator.h"

namespace mgodpl {
	inline math::Vec3d arm_axis(const robot_model::RobotModel &robot, const robot_model::ForwardKinematicsResult &fk) {
		return fk.forLink(robot.findLinkByName("stick")).orientation.rotate(math::Vec3d::UnitY());
	}

/**
 * A method to find a path from a given target point out of the tree by moving along the axis of the arm.
 * @return
 */
	ApproachPath straightout(const robot_model::RobotModel &robot,
							 const RobotState &target_state,
							 const CGAL::AABB_tree<cgal::AABBTraits> &tree,
							 const cgal::Surface_mesh_shortest_path &algo);

	/**
	 * A method that combined straightout with uniform random sampling.
	 */
	std::optional<ApproachPath> uniform_straightout_approach(const mgodpl::math::Vec3d &target,
															 const mgodpl::robot_model::RobotModel &robot,
															 const fcl::CollisionObjectd &tree_trunk_object,
															 const cgal::CgalMeshData &mesh_data,
															 random_numbers::RandomNumberGenerator &rng,
															 size_t max_attempts,
															 double ee_distance = 0.0
	);

	RobotPath plan_multigoal_path(const robot_model::RobotModel &robot,
								  const mgodpl::Mesh& trunk_mesh,
								  const mgodpl::Mesh& leaves_mesh,
								  const std::vector<math::Vec3d>& fruit_positions,
								  const RobotState &initial_state);

}
#endif //MGODPL_PROBING_MOTIONS_H
