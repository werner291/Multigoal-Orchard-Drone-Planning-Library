// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#ifndef MGODPL_APPROACHPATH_H
#define MGODPL_APPROACHPATH_H

#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Surface_mesh_shortest_path/Surface_mesh_shortest_path.h>
#include "RobotPath.h"

namespace mgodpl {
	struct ApproachPath {
		RobotPath path;
		cgal::Surface_mesh_shortest_path::Face_location shell_point;
	};
}

#endif //MGODPL_APPROACHPATH_H
