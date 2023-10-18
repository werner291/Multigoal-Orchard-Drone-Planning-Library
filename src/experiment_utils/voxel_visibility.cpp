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
#include "../math/Ray.h"
#include "../math/DomainSlice.h"
#include "GridVec.h"

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

	// 		ParametricLine line_a {triangle.a + (triangle.a - eye).normalized() * cell_size, triangle.a - eye};



	/**
	 * Given an eye poisiton and a point, compute the ray occluded by that point, plus an optional offset.
	 *
	 * @param eye 		The eye position.
	 * @param occluding_point 	The point.
	 * @param offset	How far behind the point to put the 0-point of the line.
	 *
	 * @return 			The ray.
	 */
	Ray occluded_ray(const Vec3d &eye, const Vec3d &occluding_point, double offset = 0.0) {

		Vec3d direction = (occluding_point - eye).normalized();
		Vec3d origin = occluding_point + direction * offset;

		return {origin, direction};

	}

	/**
	 * An axis-aligned slab.
	 */
	template<typename Scalar>
	struct AASlab {
		int dimension;
		RangeInclusive<Scalar> range;
	};

	/**
	 * That is, an axis-aligned bounding box that is infinite in one dimension,or an intersection of two slabs.
	 */
	template<typename Scalar>
	struct AAColumn {
		int infinite_dimension; // The dimension in which the column is infinite (0 = x, 1 = y, 2 = z).
		RangeInclusive<Scalar> range1, range2; // The ranges in the finite dimensions.
	};

	using RaySlice = DomainSlice<const Ray,RangeInclusiveD>;

	/**
	 * Restrict a ray to an AABB.
	 */
	RaySlice restrict_to_aabb(const Ray& ray, const AABBd& aabb) {
		// Find the intersections of the extended line.
		std::optional<std::array<double, 2>> intersections = line_aabb_intersection_params(aabb, ray.parametric_line());
		assert(intersections.has_value());
		return {
				ray, {intersections->at(0), intersections->at(1)}
		};
	}

	/**
	 * Restrict a ray to an AABB.
	 *
	 * Returns nullopt if the ray lies outside the AABB.
	 */
	std::optional<RaySlice> restrict_to_aabb(const RaySlice& ray, const AABBd& aabb) {
		// Find the intersections of the extended line.
		std::optional<std::array<double, 2>> intersections = line_aabb_intersection_params(aabb, ray.slice_of.parametric_line());

		if (!intersections.has_value() || intersections->at(0) > ray.range.max || intersections->at(1) < ray.range.min) {
			return std::nullopt;
		}

		return {{
				ray.slice_of, {std::max(intersections->at(0), ray.range.min), std::min(intersections->at(1), ray.range.max)}
		}};

	}

	// Function that computes occseg_a.slice_of.pointAt(occseg_a.range.min).x(), occseg_a.slice_of.pointAt(occseg_a.range.max).x() :
	Segment3d realize(const RaySlice &ray) {
		return {
				ray.slice_of.pointAt(ray.range.min),
				ray.slice_of.pointAt(ray.range.max)
		};
	}

	AABBd aabb_of(const Segment3d& segment) {
		AABBd aabb = AABBd::inverted_infinity();
		aabb.expand(segment.a);
		aabb.expand(segment.b);
		return aabb;
	}

	// Function that computes occseg_a.slice_of.pointAt(occseg_a.range.min).x(), occseg_a.slice_of.pointAt(occseg_a.range.max).x() :
	RangeInclusiveD realization_range_dim(const RaySlice& ray, int dimension) {
		const auto& realization = realize(ray);

		return {
				std::min(realization.a[dimension], realization.b[dimension]),
				std::max(realization.a[dimension], realization.b[dimension])
		};
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

		// Skip triangles with 0 area.
		if (triangle.area() < 1e-6) {
			return;
		}

		double cell_size = grid.cellSize().norm();
		Ray ray_a = occluded_ray(eye, triangle.a, cell_size);
		Ray ray_b = occluded_ray(eye, triangle.b, cell_size);
		Ray ray_c = occluded_ray(eye, triangle.c, cell_size);

		// Find the occluded rays from the eye to the triangle vertices, restricted to the base AABB of the grid.
		// The occluded area will be, effectively, the intersection of the base AABB and the convex hull of the three rays.
		const auto& occseg_a = restrict_to_aabb({ray_a,{0.0,INFINITY}}, grid.baseAABB());
		const auto& occseg_b = restrict_to_aabb({ray_b,{0.0,INFINITY}}, grid.baseAABB());
		const auto& occseg_c = restrict_to_aabb({ray_c,{0.0,INFINITY}}, grid.baseAABB());

		// Get the AABB of the intersection.
		AABBd occ_aabb = AABBd::inverted_infinity();

		if (occseg_a.has_value()) {
			occ_aabb.expand(occseg_a->slice_of.pointAt(occseg_a->range.min));
			occ_aabb.expand(occseg_a->slice_of.pointAt(occseg_a->range.max));
		}

		if (occseg_b.has_value()) {
			occ_aabb.expand(occseg_b->slice_of.pointAt(occseg_b->range.min));
			occ_aabb.expand(occseg_b->slice_of.pointAt(occseg_b->range.max));
		}

		if (occseg_c.has_value()) {
			occ_aabb.expand(occseg_c->slice_of.pointAt(occseg_c->range.min));
			occ_aabb.expand(occseg_c->slice_of.pointAt(occseg_c->range.max));
		}

		AABBi occ_aabb_grid = grid.touchedCoordinates(occ_aabb).value();

		// Iterate over the grid coordinates.
		for (int x = occ_aabb_grid.min().x(); x <= occ_aabb_grid.max().x(); ++x) {

			// Get the X-range for the slice.
			double slice_xmin = grid.baseAABB().min()[0] + x * grid.cellSize()[0];
			double slice_xmax = grid.baseAABB().min()[0] + (x + 1) * grid.cellSize()[0];

			// Compute the AABB of the slice.
			AABBd slice_aabb_x {
				Vec3d(slice_xmin, grid.baseAABB().min()[1], grid.baseAABB().min()[2]),
				Vec3d(slice_xmax, grid.baseAABB().max()[1], grid.baseAABB().max()[2])
			};

			// Emit geogebra code for the AABB as a concex hull of its corner points.

			const auto& line_a_slice_x = restrict_to_aabb(*occseg_a, slice_aabb_x);
			const auto& line_b_slice_x = restrict_to_aabb(*occseg_b, slice_aabb_x);
			const auto& line_c_slice_x = restrict_to_aabb(*occseg_c, slice_aabb_x);

			if (!line_a_slice_x.has_value() && !line_b_slice_x.has_value() && !line_c_slice_x.has_value()) {
				continue;
			}

			AABBd occ_aabb_x = AABBd::inverted_infinity();

			if (line_a_slice_x.has_value()) {
				occ_aabb_x.expand(line_a_slice_x->slice_of.pointAt(line_a_slice_x->range.min));
				occ_aabb_x.expand(line_a_slice_x->slice_of.pointAt(line_a_slice_x->range.max));
			}

			if (line_b_slice_x.has_value()) {
				occ_aabb_x.expand(line_b_slice_x->slice_of.pointAt(line_b_slice_x->range.min));
				occ_aabb_x.expand(line_b_slice_x->slice_of.pointAt(line_b_slice_x->range.max));
			}

			if (line_c_slice_x.has_value()) {
				occ_aabb_x.expand(line_c_slice_x->slice_of.pointAt(line_c_slice_x->range.min));
				occ_aabb_x.expand(line_c_slice_x->slice_of.pointAt(line_c_slice_x->range.max));
			}

			AABBi occ_aabb_grid_x = grid.touchedCoordinates(occ_aabb_x).value();

			// Iterate over the grid coordinates along the y-dimension
			for (int y = occ_aabb_grid_x.min().y(); y <= occ_aabb_grid_x.max().y(); ++y) {

				// Get the Y-range for the row.
				double row_ymin = grid.baseAABB().min().y() + y * grid.cellSize().y();
				double row_ymax = grid.baseAABB().min().y() + (y + 1) * grid.cellSize().y();

				AABBd slice_aabb_y {
						Vec3d(slice_aabb_x.min().x(), row_ymin, slice_aabb_x.min().z()),
						Vec3d(slice_aabb_x.max().x(), row_ymax, slice_aabb_x.max().z())
				};

				const auto& line_a_slice_y = restrict_to_aabb(*occseg_a, slice_aabb_y);
				const auto& line_b_slice_y = restrict_to_aabb(*occseg_b, slice_aabb_y);
				const auto& line_c_slice_y = restrict_to_aabb(*occseg_c, slice_aabb_y);

				if (!line_a_slice_y.has_value() && !line_b_slice_y.has_value() && !line_c_slice_y.has_value()) {
					continue;
				}

				AABBd occ_aabb_y = AABBd::inverted_infinity();

				if (line_a_slice_y.has_value()) {
					occ_aabb_y.expand(line_a_slice_y->slice_of.pointAt(line_a_slice_y->range.min));
					occ_aabb_y.expand(line_a_slice_y->slice_of.pointAt(line_a_slice_y->range.max));
				}

				if (line_b_slice_y.has_value()) {
					occ_aabb_y.expand(line_b_slice_y->slice_of.pointAt(line_b_slice_y->range.min));
					occ_aabb_y.expand(line_b_slice_y->slice_of.pointAt(line_b_slice_y->range.max));
				}

				if (line_c_slice_y.has_value()) {
					occ_aabb_y.expand(line_c_slice_y->slice_of.pointAt(line_c_slice_y->range.min));
					occ_aabb_y.expand(line_c_slice_y->slice_of.pointAt(line_c_slice_y->range.max));
				}

				AABBi occ_aabb_grid_y = grid.touchedCoordinates(occ_aabb_y).value();

				// Iterate over the grid coordinates along the z-dimension
				for (int z = occ_aabb_grid_y.min().z(); z <= occ_aabb_grid_y.max().z(); ++z) {
					// Mark the cell as occluded.
					occluded[Vec3i(x, y, z)] = true;
				}
			}
		}

	}

}