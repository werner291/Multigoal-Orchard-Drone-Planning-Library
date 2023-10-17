// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#include <queue>
#include <boost/range/irange.hpp>
#include "voxel_visibility.h"
#include "../math/grid_utils.h"
#include "../math/vecmath_utils.h"
#include "../math/AABBGrid.h"
#include "../math/Segment3d.h"
#include "../math/intersections.h"
#include "../math/Triangle.h"
#include "GridVec.h"
#include "../math/Plane.h"
#include "../math/lp.h"
#include "../math/OpenPyramid.h"

namespace mgodpl {

	using namespace math;

	Grid3D<bool> voxel_visibility::opaque_to_visible(const Grid3D<bool> &occluding,
													 const Vec3i &view_center,
													 bool boundary_cells_are_visible) {

		std::vector<Vec3i> cells_in_order;
		for (int x = 0; x < occluding.size().x(); ++x) {
			for (int y = 0; y < occluding.size().y(); ++y) {
				for (int z = 0; z < occluding.size().z(); ++z) {
					// Skip the view center.
					if (x == view_center.x() && y == view_center.y() && z == view_center.z()) {
						continue;
					}
					cells_in_order.emplace_back(x, y, z);
				}
			}
		}

		// sort by distance of the center to the eye.
		std::sort(cells_in_order.begin(), cells_in_order.end(), [&](const Vec3i &a, const Vec3i &b) {
			return (a - view_center).squaredNorm() < (b - view_center).squaredNorm();
		});

		// Create a grid of visibility.
		Grid3D<bool> visible(occluding.size(), false);

		visible[view_center] = true;

		// Idea: vector to "previous" step in the ray; this allows for branching.


		// Iterate in order.
		for (const Vec3i& cell : cells_in_order) {

			// Ignore transparent cells.
			if (!occluding[cell]) {
				continue;
			}

			Vec3d ray_step = (cell - view_center).cast<double>();
			ray_step /= std::max({std::abs(ray_step.x()), std::abs(ray_step.y()), std::abs(ray_step.z())});

			// Get a line from the cell center away from the eye point.
			ParametricLine line(
					cell.cast<double>(),
					ray_step
					);

			double t = 0.0;

			for (Vec3i current_cell = cell; visible.in_bounds(current_cell); current_cell = line.pointAt(t).round().cast<int>(), t += 1.0) {

				// If the cell is not transparent, mark it as visible.
				visible[current_cell] = false;

			}

		}

		return visible;

	}

	double signum(double x) {
		if (x < 0.0) {
			return -1.0;
		} else if (x > 0.0) {
			return 1.0;
		} else {
			return 0.0;
		}
	}

	/**
	 * Given an AABB and a vector, get the maximum dot product of any point in the AABB and the vector.
	 *
	 * @param vector 		The vector.
	 * @param aabb 			The aabb.
	 * @return 				The maximum dot product of any point in the AABB and the vector.
	 */
	double maxDot(const Vec3d &vector, const AABBd& aabb) {
		return std::max(vector.x() * aabb.min().x(), vector.x() * aabb.max().x()) +
			   std::max(vector.y() * aabb.min().y(), vector.y() * aabb.max().y()) +
			   std::max(vector.z() * aabb.min().z(), vector.z() * aabb.max().z());
	}

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
	void voxel_visibility::cast_occlusion(const AABBGrid& grid, Grid3D<bool>& occluded, const Triangle& triangle, const Vec3d& eye) {

		// Find the points at which the rays from the eye to the triangle vertices intersect the grid base AABB.
		ParametricLine line_a = ParametricLine::through_points(eye, triangle.a);
		ParametricLine line_b = ParametricLine::through_points(eye, triangle.b);
		ParametricLine line_c = ParametricLine::through_points(eye, triangle.c);

		// Find the start and end parameters for the parametric lines.
		// Specifically, we wish to find the X-parameter range that is covered by the occluded area.
		double t_a_min = std::max(0.0, param_at_plane(line_a,0,grid.baseAABB().min()[0]));
		double t_a_max = std::max(0.0, param_at_plane(line_a,0,grid.baseAABB().max()[0]));
		double t_b_min = std::max(0.0, param_at_plane(line_b,0,grid.baseAABB().min()[0]));
		double t_b_max = std::max(0.0, param_at_plane(line_b,0,grid.baseAABB().max()[0]));
		double t_c_min = std::max(0.0, param_at_plane(line_c,0,grid.baseAABB().min()[0]));
		double t_c_max = std::max(0.0, param_at_plane(line_c,0,grid.baseAABB().max()[0]));

		double x_min = std::min({line_a.pointAt(t_a_min)[0], line_a.pointAt(t_a_max)[0],
								 line_b.pointAt(t_b_min)[0], line_b.pointAt(t_b_max)[0],
								 line_c.pointAt(t_c_min)[0], line_c.pointAt(t_c_max)[0]});

		double x_max = std::max({line_a.pointAt(t_a_min)[0], line_a.pointAt(t_a_max)[0],
								 line_b.pointAt(t_b_min)[0], line_b.pointAt(t_b_max)[0],
								 line_c.pointAt(t_c_min)[0], line_c.pointAt(t_c_max)[0]});

		// ...and the corresponding grid X-coordinate range.
		int gx_min = std::clamp(
				(int) std::floor((x_min - grid.baseAABB().min()[0]) / grid.cellSize()[0]),
				0, (int) occluded.size()[0]
				);

		int gx_max = std::clamp(
				(int) std::ceil((x_max - grid.baseAABB().min()[0]) / grid.cellSize()[0]),
				0, (int) occluded.size()[0]
				);

		// Iterate over the grid coordinates.
		for (int x = gx_min; x < gx_max; ++x) {

			// Get the X-range for the slice.
			double slice_xmin = grid.baseAABB().min()[0] + x * grid.cellSize()[0];
			double slice_xmax = grid.baseAABB().min()[0] + (x + 1) * grid.cellSize()[0];

			// Get the line parameters at which the lines enter/exit the slice.
			double t_slice_a_min = std::clamp(param_at_plane(line_a,0,slice_xmin),t_a_min, t_a_max);
			double t_slice_a_max = std::clamp(param_at_plane(line_a,0,slice_xmax),t_a_min, t_a_max);
			double t_slice_b_min = std::clamp(param_at_plane(line_b,0,slice_xmin),t_b_min, t_b_max);
			double t_slice_b_max = std::clamp(param_at_plane(line_b,0,slice_xmax),t_b_min, t_b_max);
			double t_slice_c_max = std::clamp(param_at_plane(line_c,0,slice_xmin),t_c_min, t_c_max);
			double t_slice_c_min = std::clamp(param_at_plane(line_c,0,slice_xmax),t_c_min, t_c_max);

			// Grab the y-parameter range covered by those line segments.
			double y_min = std::min({line_a.pointAt(t_slice_a_min).y(), line_a.pointAt(t_slice_a_max).y(),
									 line_b.pointAt(t_slice_b_min).y(), line_b.pointAt(t_slice_b_max).y(),
									 line_c.pointAt(t_slice_c_min).y(), line_c.pointAt(t_slice_c_max).y()});

			double y_max = std::max({line_a.pointAt(t_slice_a_min).y(), line_a.pointAt(t_slice_a_max).y(),
									 line_b.pointAt(t_slice_b_min).y(), line_b.pointAt(t_slice_b_max).y(),
									 line_c.pointAt(t_slice_c_min).y(), line_c.pointAt(t_slice_c_max).y()});

			// ...and the corresponding grid Y-coordinate range.
			int gy_min = std::clamp(
					(int) std::floor((y_min - grid.baseAABB().min().y()) / grid.cellSize().y()),
					0, (int) occluded.size()[1]
					);

			int gy_max = std::clamp(
					(int) std::ceil((y_max - grid.baseAABB().min().y()) / grid.cellSize().y()),
					0, (int) occluded.size()[1]
					);

			// Iterate over the grid coordinates along the y-dimension
			for (int y = gy_min; y < gy_max; ++y) {

				// Get the Y-range for the row.
				double row_ymin = grid.baseAABB().min().y() + y * grid.cellSize().y();
				double row_ymax = grid.baseAABB().min().y() + (y + 1) * grid.cellSize().y();

				// Get the line parameters at which the lines enter/exit the row.
				double t_row_a_min = std::max(0.0, param_at_plane(line_a,1,row_ymin));
				double t_row_a_max = std::max(0.0, param_at_plane(line_a,1,row_ymax));
				double t_row_b_min = std::max(0.0, param_at_plane(line_b,1,row_ymin));
				double t_row_b_max = std::max(0.0, param_at_plane(line_b,1,row_ymax));
				double t_row_c_min = std::max(0.0, param_at_plane(line_c,1,row_ymin));
				double t_row_c_max = std::max(0.0, param_at_plane(line_c,1,row_ymax));

				// Grab the z-parameter range covered by those line segments.
				double z_min = std::min({line_a.pointAt(t_row_a_min).z(), line_a.pointAt(t_row_a_max).z(),
										 line_b.pointAt(t_row_b_min).z(), line_b.pointAt(t_row_b_max).z(),
										 line_c.pointAt(t_row_c_min).z(), line_c.pointAt(t_row_c_max).z()});

				double z_max = std::max({line_a.pointAt(t_row_a_min).z(), line_a.pointAt(t_row_a_max).z(),
										 line_b.pointAt(t_row_b_min).z(), line_b.pointAt(t_row_b_max).z(),
										 line_c.pointAt(t_row_c_min).z(), line_c.pointAt(t_row_c_max).z()});

				// ...and the corresponding grid Z-coordinate range.
				int gz_min = std::clamp(
						(int) std::floor((z_min - grid.baseAABB().min().z()) / grid.cellSize().z()),
						0, (int) occluded.size()[2]
						);

				int gz_max = std::clamp(
						(int) std::ceil((z_max - grid.baseAABB().min().z()) / grid.cellSize().z()),
						0, (int) occluded.size()[2]
						);

				// Iterate over the grid coordinates along the z-dimension
				for (int z = gz_min; z < gz_max; ++z) {
					// Mark the cell as occluded.
					occluded[Vec3i(x, y, z)] = true;
				}
			}
		}

	}

}