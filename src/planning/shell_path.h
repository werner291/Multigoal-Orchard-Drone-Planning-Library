// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#ifndef MGODPL_SHELL_PATH_H
#define MGODPL_SHELL_PATH_H

#include "cgal_chull_shortest_paths.h"
#include "RobotPath.h"
#include "RobotModel.h"
#include "ApproachPath.h"

namespace mgodpl {

	/**
	 * Compute a RobotPath from one state on the convex hull shell to another.
	 * @param from 				The origin shellpoint.
	 * @param to 				The destination shellpoint.
	 * @param mesh 				The mesh.
	 * @param robot 			The robot model.
	 * @return 					The shell path.
	 */
	RobotPath shell_path(const cgal::Surface_mesh_shortest_path::Face_location &from,
						 const cgal::Surface_mesh_shortest_path::Face_location &to,
						 const cgal::Surface_mesh &mesh,
						 const robot_model::RobotModel &robot);


	/**
	 * Compute a one-to-many set of distances from one source point to the shell point of a vector of approach paths.
	 *
	 * Note: for algorithmic reasons, it is far more efficient to do this in a one-to-many fashion than to compute distances/paths
	 * one-to-one n^2 times.
	 *
	 * @param from		The source point.
	 * @param paths		The approach paths.
	 * @param mesh		The mesh.
	 * @return			A vector of distances.
	 */
	std::vector<double> shell_distances(const cgal::Surface_mesh_shortest_path::Face_location &from,
										const std::vector<ApproachPath> &paths, // TODO: this should be a vector of shell points, not approach paths.
										const cgal::Surface_mesh &mesh);
}

#endif //MGODPL_SHELL_PATH_H
