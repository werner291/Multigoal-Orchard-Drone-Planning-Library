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
		for (const Vec3i &cell: cells_in_order) {

			// Ignore transparent cells.
			if (!occluding[cell]) {
				continue;
			}

			Vec3d ray_step = (cell - view_center).cast<double>();
			ray_step /= std::max({std::abs(ray_step.x()), std::abs(ray_step.y()), std::abs(ray_step.z())});

			// Get a line from the cell center away from the eye point.
			ParametricLine line(cell.cast<double>(), ray_step);

			double t = 0.0;

			for (Vec3i current_cell = cell; visible.in_bounds(current_cell); current_cell = line.pointAt(t)
					.round()
					.cast<int>(), t += 1.0) {

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
	double maxDot(const Vec3d &vector, const AABBd &aabb) {
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

	using RaySlice = DomainSlice<const Ray, RangeInclusiveD>;

	/**
	 * Restrict a ray to an AABB.
	 */
	RaySlice restrict_to_aabb(const Ray &ray, const AABBd &aabb) {
		// Find the intersections of the extended line.
		std::optional<std::array<double, 2>> intersections = line_aabb_intersection_params(aabb, ray.parametric_line());
		assert(intersections.has_value());
		return {ray, {intersections->at(0), intersections->at(1)}};
	}

	/**
	 * Restrict a ray to an AABB.
	 *
	 * Returns nullopt if the ray lies outside the AABB.
	 */
	std::optional<RaySlice> restrict_to_aabb(const RaySlice &ray, const AABBd &aabb) {
		// Find the intersections of the extended line.
		std::optional<std::array<double, 2>> intersections = line_aabb_intersection_params(aabb,
																						   ray.slice_of
																								   .parametric_line());

		if (!intersections.has_value() || intersections->at(0) > ray.range.max ||
			intersections->at(1) < ray.range.min) {
			return std::nullopt;
		}

		return {{ray.slice_of,
				 {std::max(intersections->at(0), ray.range.min), std::min(intersections->at(1), ray.range.max)}}};

	}

	// Function that computes occseg_a.slice_of.pointAt(occseg_a.range.min).x(), occseg_a.slice_of.pointAt(occseg_a.range.max).x() :
	Segment3d realize(const RaySlice &ray) {
		return {ray.slice_of.pointAt(ray.range.min), ray.slice_of.pointAt(ray.range.max)};
	}

	AABBd aabb_of(const Segment3d &segment) {
		AABBd aabb = AABBd::inverted_infinity();
		aabb.expand(segment.a);
		aabb.expand(segment.b);
		return aabb;
	}

	// Function that computes occseg_a.slice_of.pointAt(occseg_a.range.min).x(), occseg_a.slice_of.pointAt(occseg_a.range.max).x() :
	RangeInclusiveD realization_range_dim(const RaySlice &ray, int dimension) {
		const auto &realization = realize(ray);

		return {std::min(realization.a[dimension], realization.b[dimension]),
				std::max(realization.a[dimension], realization.b[dimension])};
	}

	/**
	 * Given an AABB grid, a dimension, and an index, compute the range of the slice in that dimension.
	 *
	 * @param grid 				The grid.
	 * @param dimension 		The dimension.
	 * @param index 			The index.
	 * @return 					The range of the slice in that dimension.
	 */
	RangeInclusiveD slice_range(const AABBGrid &grid, int dimension, int index) {
		return {grid.baseAABB().min()[dimension] + index * grid.cellSize()[dimension],
				grid.baseAABB().min()[dimension] + (index + 1) * grid.cellSize()[dimension]};
	}

	/**
	 * Given a slab and a set of rays, compute the AABB of the intersections of the rays with the slab.
	 *
	 * @param slab 		The slab.
	 * @param rays 		The rays.
	 * @return 			The AABB of the intersections of the rays with the slab.
	 */
	AABBd aabbInSlab(const AASlab<double> &slab, const std::array<Ray, 3> &rays) {

		AABBd aabb = AABBd::inverted_infinity();

		for (const auto& ray : rays) {
			for (const auto& x : {slab.range.min, slab.range.max}) {
				Vec3d point = ray.pointAt(param_at_plane(ray, slab.dimension, x).value_or(0.0));
				aabb.expand(point);
			}
		}

		return aabb;
	}

	/**
	 * Given an AABB and a set of rays, compute the AABB of the intersection of
	 * the convex hull of the rays and the volume delimited by the AABB.
	 *
	 * @param aabb 		The AABB.
	 * @param rays 		The rays.
	 *
	 * @return 			The AABB of the intersection of the convex hull of the rays and the volume delimited by the AABB, or nullopt if the convex hull does not intersect the AABB.
	 */
	std::optional<AABBd> aabbInAABB(const AABBd &aabb, const std::array<Ray, 3> &rays) {

		const auto& aabb_x = aabbInSlab({0, {aabb.min().x(), aabb.max().x()}}, rays);
		const auto& aabb_y = aabbInSlab({1, {aabb.min().y(), aabb.max().y()}}, rays);
		const auto& aabb_z = aabbInSlab({2, {aabb.min().z(), aabb.max().z()}}, rays);

		// Grab the intersection.
		const auto& intersection = aabb_x.intersection(aabb_y);

		if (!intersection.has_value()) {
			return std::nullopt;
		}

		return intersection->intersection(aabb_z);

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
	void voxel_visibility::cast_occlusion(const AABBGrid &grid,
										  Grid3D<bool> &occluded,
										  const Triangle &triangle,
										  const Vec3d &eye) {

		double cell_size = grid.cellSize().norm();

		// The occluded volume is the convex hull of the three rays, limited to the parent AABB.
		const std::array<Ray, 3> rays {
				occluded_ray(eye, triangle.a, cell_size),
				occluded_ray(eye, triangle.b, cell_size),
				occluded_ray(eye, triangle.c, cell_size),
		};

		// Compute the AABB of the intersection of the occluded volume and the grid AABB.
		const auto& occluded_volume_aabb = aabbInAABB(grid.baseAABB(), rays);

		const int grid_xmin = *grid.getCoordinateInDimension(occluded_volume_aabb->min().x(), 0);
		const int grid_xmax = *grid.getCoordinateInDimension(occluded_volume_aabb->max().x(), 0);

		std::cout << "Grid from " << grid_xmin << " to " << grid_xmax << "\n";

		for (int x = grid_xmin; x <= grid_xmax; ++x) {

			// Get the AABB of the slice.
			const AABBd slice_aabb {
					grid.baseAABB().min() + Vec3d(x * grid.cellSize().x(), 0.0, 0.0),
					grid.baseAABB().min() + Vec3d((x + 1) * grid.cellSize().x(),
												  grid.baseAABB().size().y(),
												  grid.baseAABB().size().z())
			};

			// Compute the AABB of the intersection of the occluded volume and the slice AABB.
			const auto& occluded_aabb_in_xslice = aabbInAABB(slice_aabb, rays);

			if (!occluded_aabb_in_xslice.has_value()) {
				continue;
			}

			assert(occluded_volume_aabb->inflated(1.0e-6).contains(*occluded_aabb_in_xslice));

			const int grid_ymin = *grid.getCoordinateInDimension(occluded_aabb_in_xslice->min().y(), 1);
			const int grid_ymax = *grid.getCoordinateInDimension(occluded_aabb_in_xslice->max().y(), 1);

			for (int y = grid_ymin; y <= grid_ymax; ++y) {

				const AABBd xy_slice_aabb = {
						slice_aabb.min() + Vec3d(0.0, y * grid.cellSize().y(), 0.0),
						slice_aabb.min() + Vec3d(slice_aabb.size().x(), (y + 1) * grid.cellSize().y(),
												  slice_aabb.size().z())
				};

				// Compute the AABB of the intersection of the occluded volume and the slice AABB.
				const auto& occluded_aabb_in_xyslice = aabbInAABB(xy_slice_aabb, rays);

				if (!occluded_aabb_in_xyslice.has_value()) {
					continue;
				}

				assert(occluded_aabb_in_xslice->inflated(1.0e-6).contains(*occluded_aabb_in_xyslice));

				const int grid_zmin = *grid.getCoordinateInDimension(occluded_aabb_in_xyslice->min().z(), 2);
				const int grid_zmax = *grid.getCoordinateInDimension(occluded_aabb_in_xyslice->max().z(), 2);

				for (int z = grid_zmin; z <= grid_zmax; ++z) {

					occluded[{x, y, z}] = true;

				}

			}

		}



	}




}