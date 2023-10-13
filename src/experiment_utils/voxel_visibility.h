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
		Grid3D<bool> opaque_to_visible(const math::AABBGrid &gridcoords,
									   const Grid3D<bool> &occluding,
									   const math::Vec3d &view_center,
									   bool boundary_cells_are_visible);
	}
}

#endif //MGODPL_VOXEL_VISIBILITY_H
