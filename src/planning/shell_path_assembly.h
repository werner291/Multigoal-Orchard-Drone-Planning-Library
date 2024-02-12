// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#ifndef MGODPL_SHELL_PATH_ASSEMBLY_H
#define MGODPL_SHELL_PATH_ASSEMBLY_H

#include "RobotPath.h"
#include "RobotModel.h"
#include "cgal_chull_shortest_paths.h"
#include "ApproachPath.h"

namespace mgodpl::shell_path_planning {

	RobotPath retreat_move_probe(const robot_model::RobotModel &robot,
								 const cgal::Surface_mesh &convex_hull,
								 const ApproachPath &retreat_path,
								 const ApproachPath &probe_path);

	RobotPath assemble_final_path(const robot_model::RobotModel &robot,
								  const cgal::Surface_mesh &convex_hull,
								  const std::vector <ApproachPath> &approach_paths,
								  ApproachPath &initial_approach_path,
								  const std::vector <size_t> &order);

}

#endif //MGODPL_SHELL_PATH_ASSEMBLY_H
