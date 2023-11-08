// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#include <queue>

#include "../math/grid_utils.h"
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

		double xmin = grid.baseAABB().min().x();
		double xmax = grid.baseAABB().max().x();

		for (const int dim : {0,1,2}) {
			double dim_xmin = INFINITY;
			double dim_xmax = -INFINITY;
			for (const Ray& ray : rays) {
				if (ray.direction()[dim] == 0.0) continue;
				double t1 = std::max(0.0,(grid.baseAABB().min()[dim] - ray.origin()[dim]) / ray.direction()[dim]);
				double t2 = std::max(0.0,(grid.baseAABB().max()[dim] - ray.origin()[dim]) / ray.direction()[dim]);
				double x1 = ray.origin()[0] + t1 * ray.direction()[0];
				double x2 = ray.origin()[0] + t2 * ray.direction()[0];
				dim_xmin = std::min(dim_xmin, std::min(x1, x2));
				dim_xmax = std::max(dim_xmax, std::max(x1, x2));
			}
			xmin = std::max(xmin, dim_xmin);
			xmax = std::min(xmax, dim_xmax);
		}

		// Find the grid x-coordinates affected by that.
		const int grid_xmin = grid.getCoordinateInDimension(xmin + margin, 0);
		const int grid_xmax = grid.getCoordinateInDimension(xmax - margin, 0);

		// Iterate over all grid x-coordinates, treating the volume one slice of grid cells at a time.
		for (int x = grid_xmin; x <= grid_xmax; ++x) {

			double x_slice_min = grid.baseAABB().min().x() + x * grid.cellSize().x();
			double x_slice_max = grid.baseAABB().min().x() + (x + 1) * grid.cellSize().x();

			AABBd slice_aabb {
				{x_slice_min, grid.baseAABB().min().y(), grid.baseAABB().min().z()},
				{x_slice_max, grid.baseAABB().max().y(), grid.baseAABB().max().z()}
			};

			double ymin = slice_aabb.min().y();
			double ymax = slice_aabb.max().y();

			for (const int dim : {0,1,2}) {
				double dim_ymin = INFINITY;
				double dim_ymax = -INFINITY;
				for (const Ray& ray : rays) {
					if (ray.direction()[dim] == 0.0) continue;
					double t1 = std::max(0.0,(slice_aabb.min()[dim] - ray.origin()[dim]) / ray.direction()[dim]);
					double t2 = std::max(0.0,(slice_aabb.max()[dim] - ray.origin()[dim]) / ray.direction()[dim]);
					double y1 = ray.origin()[1] + t1 * ray.direction()[1];
					double y2 = ray.origin()[1] + t2 * ray.direction()[1];
					dim_ymin = std::min(dim_ymin, std::min(y1, y2));
					dim_ymax = std::max(dim_ymax, std::max(y1, y2));
				}
				ymin = std::max(ymin, dim_ymin);
				ymax = std::min(ymax, dim_ymax);
			}

			// Find the grid y-coordinates affected by that.
			const int grid_ymin = grid.getCoordinateInDimension(ymin + margin, 1);
			const int grid_ymax = grid.getCoordinateInDimension(ymax - margin, 1);
//			const int grid_ymin = *grid.getCoordinateInDimension(triangle.a.y() + margin, 1);
//			const int grid_ymax = *grid.getCoordinateInDimension(triangle.a.y() - margin, 1);

			// Now, iterate over all grid y-coordinates, each time treating a single column of grid cells.
			for (int y = grid_ymin; y <= grid_ymax; ++y) {

				double column_ymin = grid.baseAABB().min().y() + y * grid.cellSize().y();
				double column_ymax = grid.baseAABB().min().y() + (y + 1) * grid.cellSize().y();

				AABBd column_aabb {
					{slice_aabb.min().x(), column_ymin, slice_aabb.min().z()},
					{slice_aabb.max().x(), column_ymax, slice_aabb.max().z()}
				};

				double zmin = column_aabb.min().z();
				double zmax = column_aabb.max().z();

				for (const int dim : {0,1,2}) {
					double dim_zmin = INFINITY;
					double dim_zmax = -INFINITY;
					for (const Ray& ray : rays) {
						if (ray.direction()[dim] == 0.0) continue;
						double t1 = std::max(0.0,(column_aabb.min()[dim] - ray.origin()[dim]) / ray.direction()[dim]);
						double t2 = std::max(0.0,(column_aabb.max()[dim] - ray.origin()[dim]) / ray.direction()[dim]);
						double z1 = ray.origin()[2] + t1 * ray.direction()[2];
						double z2 = ray.origin()[2] + t2 * ray.direction()[2];
						dim_zmin = std::min(dim_zmin, std::min(z1, z2));
						dim_zmax = std::max(dim_zmax, std::max(z1, z2));
					}
					zmin = std::max(zmin, dim_zmin);
					zmax = std::min(zmax, dim_zmax);
				}

				const int grid_zmin = grid.getCoordinateInDimension(zmin + margin, 2);
				const int grid_zmax = grid.getCoordinateInDimension(zmax - margin, 2);

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