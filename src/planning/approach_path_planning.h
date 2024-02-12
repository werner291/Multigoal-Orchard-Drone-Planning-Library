// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#ifndef MGODPL_APPROACH_PATH_PLANNING_H
#define MGODPL_APPROACH_PATH_PLANNING_H

#include "ApproachPath.h"
#include "RobotModel.h"
#include "RobotState.h"
#include "cgal_chull_shortest_paths.h"

namespace mgodpl {
	namespace approach_planning {

		/**
		 * Given a robot, an initial state, a flying base, and mesh data about the shell around the canopym this function
		 * plans an initial approach path from a state outside the tree canopy to a state within the shell space.
		 *
		 * Note: this function does NOT check for collisions.
		 *
		 * @param robot 				The robot model.
		 * @param initial_state 		The initial state of the robot.
		 * @param flying_base 			The flying base of the robot.
		 * @param mesh_data 			The mesh data related to the convex hull around the tree canopy.
		 * @return 						The initial approach path. (Not checked for collisions!)
		 */
		ApproachPath plan_initial_approach_path(const robot_model::RobotModel &robot,
												const RobotState &initial_state,
												const robot_model::RobotModel::LinkId flying_base,
												const cgal::CgalMeshData &mesh_data);

		/**
		 * Given a robot, mesh data about the shell around the canopy, and a target point, this function generates
		 * a straight-in motion approach path from the projection of the target point onto the shell surface
		 * to the target point.
		 *
		 * Note: this function does NOT check for collisions.
		 *
		 * @param robot 				The robot model.
		 * @param mesh_data 			The mesh data related to the convex hull around the tree canopy.
		 * @param tgt 					The target point.
		 * @return 						The straight-in motion approach path. (Not checked for collisions!)
		 */
		ApproachPath straight_in_motion(const robot_model::RobotModel &robot,
										const cgal::CgalMeshData &mesh_data,
										const math::Vec3d &tgt);
	}
}

#endif //MGODPL_APPROACH_PATH_PLANNING_H
