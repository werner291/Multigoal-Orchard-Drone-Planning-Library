// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#include <queue>

#include "../math/grid_utils.h"
#include "../math/vecmath_utils.h"
#include "../math/AABBGrid.h"
#include "../math/Segment3d.h"
#include "../math/intersections.h"
#include "../math/Triangle.h"
#include "../math/aabb_of.h"

#include "GridVec.h"

#include "voxel_visibility.h"
#include "visibility_geometry.h"

namespace mgodpl {

	using namespace math;
	using namespace visibility;

	struct RaySlice {
		const Ray& ray;
		RangeInclusiveD t_range;
	};

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
	void voxel_visibility::cast_occlusion(const AABBGrid &grid,
										  Grid3D<bool> &occluded,
										  const Triangle &triangle,
										  const Vec3d &eye) {

		// Get a rough measure of how big a cell is.
		double cell_size = grid.cellSize().norm();

		double margin = cell_size * 1e-10;

		// The occluded volume is the convex hull of the three rays, limited to the parent AABB.
		// Rays are offset a bit to account for the idea of "boundary cells".
		const std::array<Ray, 3> rays{occluded_ray(eye, triangle.a, cell_size),
									  occluded_ray(eye, triangle.b, cell_size),
									  occluded_ray(eye, triangle.c, cell_size),};

		// Compute the AABB of the intersection of the occluded volume and the grid AABB.
		const auto &occluded_volume_aabb = aabbInAABB(grid.baseAABB(), rays, 0);

		// Find the grid x-coordinates affected by that.
		const int grid_xmin = *grid.getCoordinateInDimension(occluded_volume_aabb->min().x() + margin, 0);
		const int grid_xmax = *grid.getCoordinateInDimension(occluded_volume_aabb->max().x() - margin, 0);

		// Iterate over all grid x-coordinates, treating the volume one slice of grid cells at a time.
		for (int x = grid_xmin; x <= grid_xmax; ++x) {

			// Get the AABB of the slice.
			const AABBd slice_aabb{grid.baseAABB().min() + Vec3d(x * grid.cellSize().x(), 0.0, 0.0),
								   grid.baseAABB().min() + Vec3d((x + 1) * grid.cellSize().x(),
																 grid.baseAABB().size().y(),
																 grid.baseAABB().size().z())};

			// Compute the AABB of the intersection of the occluded volume and the slice AABB.
			const auto &occluded_aabb_in_xslice = aabbInAABB(slice_aabb, rays, 1);

			// If there is no intersection, skip this slice since it's not occluded at all.
			if (!occluded_aabb_in_xslice.has_value()) {
				continue;
			}

			// Sanity check: the intersection AABB should be contained in the slice AABB.
			assert(slice_aabb.inflated(1.0e-6).contains(*occluded_aabb_in_xslice));

			// Find the grid y-coordinates affected by that.
			const int grid_ymin = *grid.getCoordinateInDimension(occluded_aabb_in_xslice->min().y() + margin, 1);
			const int grid_ymax = *grid.getCoordinateInDimension(occluded_aabb_in_xslice->max().y() - margin, 1);

			// Now, iterate over all grid y-coordinates, each time treating a single column of grid cells.
			for (int y = grid_ymin; y <= grid_ymax; ++y) {

				// Get the AABB of the column.
				const AABBd xy_slice_aabb = {slice_aabb.min() + Vec3d(0.0, y * grid.cellSize().y(), 0.0),
											 slice_aabb.min() + Vec3d(slice_aabb.size().x(),
																	  (y + 1) * grid.cellSize().y(),
																	  slice_aabb.size().z())};

				// Compute the AABB of the intersection of the occluded volume and the slice AABB.
				const auto &occluded_aabb_in_xyslice = aabbInAABB(xy_slice_aabb, rays, 2);

				// If there is no intersection, skip this column since it's not occluded at all.
				if (!occluded_aabb_in_xyslice.has_value()) {
					continue;
				}

				// Sanity check: the intersection AABB should be contained in the slice AABB.
				assert(xy_slice_aabb.inflated(1.0e-6).contains(*occluded_aabb_in_xyslice));

				// Find the grid z-coordinates affected by that.
				const int grid_zmin = *grid.getCoordinateInDimension(occluded_aabb_in_xyslice->min().z() + margin, 2);
				const int grid_zmax = *grid.getCoordinateInDimension(occluded_aabb_in_xyslice->max().z() - margin, 2);

				// Mark all affected grid cells as occluded.
				for (int z = grid_zmin; z <= grid_zmax; ++z) {
					occluded[{x, y, z}] = true;
				}
			}
		}
	}

	/**
	 * Given an AABB and an AABB grid ray, check whether all voxels in the AABB were previously marked as occluded.
	 * @param occluded	The grid holding the occlusion information.
	 * @param aabb		The AABB designating a range of voxels (inclusive).
	 * @return			True if all voxels in the AABB were previously marked as occluded, false otherwise.
	 */
	bool all_occluded(
			const Grid3D<bool> &occluded,
			const AABBi& aabb
	) {
		for (int x = aabb.min().x(); x <= aabb.max().x(); ++x) {
			for (int y = aabb.min().y(); y <= aabb.max().y(); ++y) {
				for (int z = aabb.min().z(); z <= aabb.max().z(); ++z) {
					// If any of the voxels is not occluded, return false.
					if (!occluded[{x, y, z}]) {
						return false;
					}
				}
			}
		}

		return true;
	}

	Grid3D<bool> voxel_visibility::cast_occlusion(const AABBGrid &grid,
												  const std::vector<math::Triangle> &triangles,
												  const Vec3d &eye) {

		Grid3D<bool> occluded(grid.size().x(), grid.size().y(), grid.size().z(), false);

		// Then, perform per-triangle occlusion casting:
		for (const auto& [distance, triangle] : sorted_by_distance(triangles, eye)) {

			// Quick check: if the triangle is fully occluded, it will not contribute to further occlusion of the scene.
			const auto& triangle_aabb = grid.touchedCoordinates(math::aabb_of(*triangle));
			if (triangle_aabb.has_value() && all_occluded(occluded, *triangle_aabb)) {
				continue;
			}

			// Perform the expensive occlusion casting.
			cast_occlusion(grid, occluded, *triangle, eye);
		}

		return occluded;
	}

}