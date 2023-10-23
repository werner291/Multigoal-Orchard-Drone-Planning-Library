// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#ifndef MGODPL_VOXEL_VISIBILITY_H
#define MGODPL_VOXEL_VISIBILITY_H

#include "../math/Vec3.h"

namespace mgodpl {

	// Forward declarations:
	template<typename T>
	class Grid3D;

	namespace math {
		class AABBGrid;
		struct Triangle;
	}

	// Declarations:

	namespace voxel_visibility {

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

		/**
		 * In a given visibility grid, set all cells occluded by a set of triangles to false,
		 * processing the triangles as a batch.
		 *
		 * @param grid 				The visibility grid.
		 * @param occluded 			The occluded grid.
		 * @param triangles 		The triangles.
		 * @param eye 				The eye point.
		 */
		Grid3D<bool> cast_occlusion(const math::AABBGrid& grid,
							const std::vector<math::Triangle>& triangles,
							const math::Vec3d& eye);
	}
}

#endif //MGODPL_VOXEL_VISIBILITY_H
