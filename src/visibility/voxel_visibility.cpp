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

namespace mgodpl {

	using namespace math;

	/**
	 * Given an eye position and a point, compute the ray occluded by that point, plus an optional offset.
	 *
	 * @param eye 		The eye position.
	 * @param occluding_point 	The point.
	 * @param offset	How far behind the point to put the 0-point of the line.
	 *
	 * @return 			The ray.
	 */
	Ray occluded_ray(const Vec3d &eye, const Vec3d &occluding_point, double offset = 0.0) {

		// A direction away from the eye.
		Vec3d direction = (occluding_point - eye).normalized();

		// Offset the ray origin by a small amount.
		Vec3d origin = occluding_point + direction * offset;

		return {origin, direction};

	}

	/**
	 * An axis-aligned slab, representing the space between two parallel axis-aligned planes.
	 */
	template<typename Scalar>
	struct AASlab {
		int dimension; /// Whether it's the X, Y or Z axis that this is perpendicular to.
		RangeInclusive<Scalar> range; /// The coordinate range on that axis covered by the slab.
	};

	/**
	 * Given a slab and a triangle, compute the AABB of the portion of the triangle
	 * that lies on the inside of the slab.
	 *
	 * @param slab 			The slab.
	 * @param triangle 		The triangle.
	 * @return 				The AABB of the portion of the triangle that lies on the inside of the slab.
	 */
	std::optional<AABBd> aabbInSlab(const AASlab<double>& slab, const Triangle& triangle) {

		// We try to find the AABB of the intersection of the triangle with the slab (interior + boundary).
		// We treat this as equivalent as the AABB of the *boundary* of the triangle with the slab volume;
		// note that the boundary itself consists of the three edges of the triangle.

		// We start with an empty AABB, to which we will add all the points that are in the slab.
		AABBd aabb = AABBd::inverted_infinity();

		// For each triangle vertex, if it is in the slab, add it to the AABB.
		for (const auto& point : {triangle.a, triangle.b, triangle.c}) {
			if (slab.range.contains(point[slab.dimension])) {
				aabb.expand(point);
			}
		}

		// Then, we consider the three edges of the triangle.
		std::array<Segment3d, 3> segments {
				{Segment3d{triangle.a, triangle.b},
				 Segment3d{triangle.b, triangle.c},
				 Segment3d{triangle.c, triangle.a}}
		};

		// We want to find what part of the edge falls inside of the slab.
		// For building the AABB, it is enough to use the endpoints of
		// the portion of the edge that falls inside the slab.

		// Fer every segment...
		for (auto& segment : segments) {

			// For each of the two planes...
			for (const auto& x : {slab.range.min, slab.range.max}) {

				// Find the intersection parameter t of the segment with the plane.
				auto line = segment.extend_to_line();
				double t = param_at_plane(line, slab.dimension, x);

				if (t >= 0.0 && t <= 1.0) {
					// The segment intersects the plane; add the point to the AABB.
					Vec3d pt = line.pointAt(t);
					aabb.expand(pt);
				}

			}

		}

		// if all finite, then return, otherwise nullopt.
		if (finite(aabb.min().x()) && finite(aabb.min().y()) && finite(aabb.min().z()) &&
			finite(aabb.max().x()) && finite(aabb.max().y()) && finite(aabb.max().z())) {
			return aabb;
		} else {
			return std::nullopt;
		}

	}

	/**
 * Compute the AABB of the portion of the ray that lies on the inside of the slab.
 *
 * @param slab      The slab.
 * @param ray       The ray.
 * @return          The AABB of the portion of the ray that lies on the inside of the slab.
 */
	std::optional<AABBd> rayInSlab(const AASlab<double> &slab, const Ray &ray) {

		// Check if the ray is parallel to the slab, indicating no intersection.
		if (ray.direction()[slab.dimension] == 0.0) {
			return std::nullopt;
		}

		// Calculate the parameter 't1' where the ray intersects the slab's minimum boundary.
		const auto& t1 = param_at_plane(ray, slab.dimension, slab.range.min);

		// Calculate the parameter 't2' where the ray intersects the slab's maximum boundary.
		const auto& t2 = param_at_plane(ray, slab.dimension, slab.range.max);

		// If both 't1' and 't2' are missing (no intersection points), return empty result.
		if (!t1.has_value() && !t2.has_value()) {
			return std::nullopt;
		}

		// Initialize an axis-aligned bounding box (AABB) with extreme values.
		AABBd aabb = AABBd::inverted_infinity();

		// If 't1' is available, expand the AABB to include the corresponding point on the ray.
		if (t1) aabb.expand(ray.pointAt(*t1));

		// If 't2' is available, expand the AABB to include the corresponding point on the ray.
		if (t2) aabb.expand(ray.pointAt(*t2));

		// Check if the resulting AABB is within the slab's boundaries with a small tolerance.
		assert(slab.range.min - 1e-6 <= aabb.min()[slab.dimension] && slab.range.max + 1e-6 >= aabb.max()[slab.dimension]);

		// Return the computed AABB for the inside portion of the slab.
		return aabb;
	}

	/**
	 * Given a slab and a set of rays, compute the AABB of the intersections of the rays with the slab.
	 *
	 * @param slab 		The slab.
	 * @param rays 		The rays.
	 * @return 			The AABB of the intersections of the rays with the slab.
	 */
	std::optional<AABBd> aabbInSlab(const AASlab<double> &slab, const std::array<Ray, 3> &rays) {

		// Calculate the AABB of the triangle formed by the ray origins.
		const auto& triangle_aabb = aabbInSlab(slab, Triangle{rays[0].pointAt(0.0), rays[1].pointAt(0.0), rays[2].pointAt(0.0)});

		// Calculate the AABB of the intersection of the individual rays with the slab.
		const auto& ray0_aabb = rayInSlab(slab, rays[0]);
		const auto& ray1_aabb = rayInSlab(slab, rays[1]);
		const auto& ray2_aabb = rayInSlab(slab, rays[2]);

		// If not one point of the rays is in the slab, there is no intersection.
		if (!triangle_aabb.has_value() && !ray0_aabb.has_value() && !ray1_aabb.has_value() && !ray2_aabb.has_value()) {
			return std::nullopt;
		}

		// Initialize a total AABB with extreme values.
		AABBd total_aabb = AABBd::inverted_infinity();

		// Take the union of the AABBs; since all are inside the parent slab,
		// the union is guaranteed to be inside the slab as well.
		for (const auto& aabb : {triangle_aabb, ray0_aabb, ray1_aabb, ray2_aabb}) {
			if (aabb.has_value()) {
				total_aabb = total_aabb.combined(*aabb);
			}
		}

		// Check if the resulting total AABB is within the slab's boundaries with a small tolerance.
		assert(
				slab.range.min - 1e-6 <= total_aabb.min()[slab.dimension] &&
				slab.range.max + 1e-6 >= total_aabb.max()[slab.dimension]
		);

		// Return the computed total AABB for the intersections with the slab.
		return total_aabb;
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

		// We decompose the AABB into three slabs, one for each axis...
		const auto& aabb_x = aabbInSlab({0, {aabb.min().x(), aabb.max().x()}}, rays);
		const auto& aabb_y = aabbInSlab({1, {aabb.min().y(), aabb.max().y()}}, rays);
		const auto& aabb_z = aabbInSlab({2, {aabb.min().z(), aabb.max().z()}}, rays);

		// ...then take the intersection of the resulting AABB.

		// It's an intersection, so if there's no intersection in any of the slabs,
		// there's no intersection in total either.
		if (!aabb_x.has_value() || !aabb_y.has_value() || !aabb_z.has_value()) {
			return std::nullopt;
		}

		// Get the intersections, with the understanding that an empty optional
		// means that the intersection is emtpy and we can return nullopt.
		const auto& xy = aabb_x->intersection(*aabb_y);

		if (!xy.has_value()) {
			return std::nullopt;
		}

		const auto& result = xy->intersection(*aabb_z);

		if (result) {
			assert(aabb.inflated(1.0e-6).contains(*result));
		}

		return result;

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

		// Get a rough measure of how big a cell is.
		double cell_size = grid.cellSize().norm();

		double margin = cell_size * 1e-10;

		// The occluded volume is the convex hull of the three rays, limited to the parent AABB.
		// Rays are offset a bit to account for the idea of "boundary cells".
		const std::array<Ray, 3> rays{occluded_ray(eye, triangle.a, cell_size),
									  occluded_ray(eye, triangle.b, cell_size),
									  occluded_ray(eye, triangle.c, cell_size),};

		// Compute the AABB of the intersection of the occluded volume and the grid AABB.
		const auto &occluded_volume_aabb = aabbInAABB(grid.baseAABB(), rays);

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
			const auto &occluded_aabb_in_xslice = aabbInAABB(slice_aabb, rays);

			// If there is no intersection, skip this slice since it's not occluded at all.
			if (!occluded_aabb_in_xslice.has_value()) {
				continue;
			}

			// Sanity check: the intersection AABB should be contained in the slice AABB.
			assert(occluded_volume_aabb->inflated(1.0e-6).contains(*occluded_aabb_in_xslice));

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
				const auto &occluded_aabb_in_xyslice = aabbInAABB(xy_slice_aabb, rays);

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

	void voxel_visibility::cast_occlusion(const AABBGrid &grid,
									  Grid3D<bool> &occluded,
									  const std::vector<math::Triangle> &triangles,
									  const Vec3d &eye) {

		// First, sort the triangles by the distance of their centroid to the eye.
		std::vector<std::pair<double, const Triangle*>> triangles_with_distance;

		for (const auto& triangle : triangles) {
			triangles_with_distance.emplace_back(((triangle.a + triangle.b + triangle.c) / 3.0 - eye).squaredNorm(), &triangle);
		}

		// Then, perform per-triangle occlusion casting:
		for (const auto& [distance, triangle] : triangles_with_distance) {

			// Quick check: if the triangle is fully occluded, it will not contribute to further occlusion of the scene.
			const auto& triangle_aabb = grid.touchedCoordinates(math::aabb_of(*triangle));
			if (triangle_aabb.has_value() && all_occluded(occluded, *triangle_aabb)) {
				continue;
			}

			// Perform the expensive occlusion casting.
			cast_occlusion(grid, occluded, *triangle, eye);
		}
	}
}