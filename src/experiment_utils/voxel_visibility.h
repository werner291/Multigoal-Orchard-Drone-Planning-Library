// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#ifndef MGODPL_VOXEL_VISIBILITY_H
#define MGODPL_VOXEL_VISIBILITY_H

// Forward declarations:

#include "../math/Vec3.h"

namespace mgodpl {
	template<typename T>
	class Grid3D;

	namespace math {
		class AABBGrid;
		struct Triangle;
	}

	// Declarations:

	namespace voxel_visibility {

		/**
		 * Given a Grid3D<bool> representing the occluded space,
		 * and a view center, return a Grid3D<bool> representing
		 * the visible space from that view center.
		 *
		 * @param gridcoords	An AABBGrid to aid in converting between world coordinates and grid coordinates.
		 * @param occluding 	A Grid3D<bool> representing the occluded space.
		 * @param view_center 	The view center.
		 *
		 * @return 				A Grid3D<bool> representing the visible space from that view center.
		 */
		Grid3D<bool> opaque_to_visible(const Grid3D<bool> &occluding,
									   const math::Vec3i &view_center,
									   bool boundary_cells_are_visible);

		/**
		 * In a given visibility grid, set all cells occluded by a triangle to false.
		 *
		 * This works by essentially considering the open infinite pyramid formed
		 * with eye eye at the apex and the triangle as the "base", though extended
		 * infinitely in all directions.
		 *
		 * @param grid		The visibility grid.
		 * @param triangle	The triangle.
		 * @param eye		The eye point.
		 */
		void cast_occlusion(const math::AABBGrid& grid,
							Grid3D<bool>& occluded,
							const math::Triangle& triangle,
							const math::Vec3d& eye);

		}
}

#endif //MGODPL_VOXEL_VISIBILITY_H
