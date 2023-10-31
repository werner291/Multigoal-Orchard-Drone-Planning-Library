// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/31/23.
//

#ifndef MGODPL_CONVEX_HULL_H
#define MGODPL_CONVEX_HULL_H

#include "Mesh.h"

namespace mgodpl::visibility {

	Mesh convexHull(const std::vector<math::Vec3d> &points);

}

#endif //MGODPL_CONVEX_HULL_H
