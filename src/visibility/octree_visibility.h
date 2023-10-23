// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/23/23.
//

#ifndef MGODPL_OCTREE_VISIBILITY_H
#define MGODPL_OCTREE_VISIBILITY_H

#include <variant>
#include "../math/AABBGrid.h"
#include "GridVec.h"
#include "../math/Triangle.h"
#include "Octree.h"

namespace mgodpl::visibility {

	using VisibilityOctree = Octree<std::monostate, bool>;

	math::AABBd childAABB(const math::AABBd &parent, size_t child_index);

	void cast_occlusion(const math::AABBd &base_volume,
						VisibilityOctree &occluded,
						const math::Triangle &triangle,
						const math::Vec3d &eye,
						int depth);

	VisibilityOctree cast_occlusion(const math::AABBd& base_volume,
						const std::vector<math::Triangle>& triangles,
						const math::Vec3d& eye);

}

#endif //MGODPL_OCTREE_VISIBILITY_H
