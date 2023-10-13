// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#include "voxel_visibility.h"
#include "../math/grid_utils.h"
#include "../math/vecmath_utils.h"
#include "../math/AABBGrid.h"
#include "../math/Segment3d.h"
#include "../math/intersections.h"
#include "GridVec.h"

namespace mgodpl {

	using namespace math;

	Grid3D<bool> voxel_visibility::opaque_to_visible(const AABBGrid &gridcoords,
													 const Grid3D<bool> &occluding,
													 const Vec3d &view_center,
													 bool boundary_cells_are_visible) {

		// Initialize the visible grid as a copy of the occluding grid.
		Grid3D<bool> visible(occluding.size(), false);
		Grid3D<bool> visited(occluding.size(), false);

		// Get the grid coordinates of the view center.
		auto view_center_grid = gridcoords.getGridCoordinates(view_center);

		// If the view center is out of bounds, return the visible grid as-is.
		if (!view_center_grid) {
			return visible;
		}

		std::vector<Vec3i> stack{*view_center_grid};

		// While the stack is not empty...
		while (!stack.empty()) {
			// Pop the top element off the stack.
			auto coord = stack.back();
			stack.pop_back();

			visited[coord] = true;

			// Mark the corresponding cell in the visible grid as visible.
			visible[coord] = true;

			// If the cell is transparent...
			if (!occluding[coord]) {

				auto cell_aabb = gridcoords.getAABB(coord).value();

				// For every cell that shares a face with the current cell...
				for (const auto &neighbor: grid_utils::neighbors(coord)) {

					// If the neighbor is out of bounds, skip it.
					if (!occluding.in_bounds(neighbor)) {
						continue;
					}

					if (!boundary_cells_are_visible && occluding[neighbor]) {
						// If the neighbor is opaque, skip it.
						continue;
					}

					if (visited[neighbor]) {
						// Avoid going back to cells we've already visited. (TODO: could replace this by checking if the line is going the wrong way too)
						continue;
					}

					auto neighborAABB = *gridcoords.getAABB(neighbor);

					Segment3d segment(view_center, neighborAABB.center());

					// If the line between the view center and the center of the neighbor crosses through the current cell...
					if (intersects(cell_aabb, segment)) {
						// Push the neighbor onto the stack.
						stack.push_back(neighbor);
					}
				}
			}
		}

		// Return the visible grid.
		return visible;
	}

}