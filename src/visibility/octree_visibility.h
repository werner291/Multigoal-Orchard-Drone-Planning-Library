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

	/**
	 * In a given octree, set all cells occluded by a set of triangles to true.
	 *
	 * In this algorithm, triangles are sorted by distance from the eye point,
	 * then processed individually.
	 *
	 * @param base_volume 		The base volume.
	 * @param triangles 		The triangles.
	 * @param eye 				The eye point.
	 * @return 					The occluded octree.
	 */
	VisibilityOctree cast_occlusion_batch_sorting(const math::AABBd &base_volume,
												  const std::vector<math::Triangle>& triangles,
												  const math::Vec3d &eye);

	/**
	 * In a given octree, set all cells occluded by a set of triangles to true.
	 *
	 * In this algorithm, we use a divide-and-conquer approach, where we recursively
	 * subdivide the octree, splitting the set of affected triangles each time as well.
	 *
	 * @param base_volume 		The base volume.
	 * @param triangles 		The triangles. (Passes by value since we internally shuffle it around)
	 * @param eye 				The eye point.
	 * @return 					The occluded octree.
	 */
	VisibilityOctree cast_occlusion_batch_dnc(const math::AABBd &base_volume,
											  std::vector<math::Triangle> triangles,
											  const math::Vec3d &eye);

}

#endif //MGODPL_OCTREE_VISIBILITY_H
