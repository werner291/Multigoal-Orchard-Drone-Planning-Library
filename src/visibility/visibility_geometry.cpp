// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/23/23.
//

#include "visibility_geometry.h"
#include "../math/Segment3d.h"
#include "../math/intersections.h"

namespace mgodpl::visibility {

	using namespace math;

	Ray occluded_ray(const Vec3d &eye, const Vec3d &occluding_point, double offset) {

		// A direction away from the eye.
		Vec3d direction = (occluding_point - eye).normalized();

		// Offset the ray origin by a small amount.
		Vec3d origin = occluding_point + direction * offset;

		return {origin, direction};

	}

	std::optional<AABBd> aabbInSlab(const AASlab<double> &slab, const Triangle &triangle) {

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

	std::vector<std::pair<double, const math::Triangle *>>
	sorted_by_distance(const std::vector<mgodpl::math::Triangle> &triangles, const Vec3d &eye) {
		// First, sort the triangles by the distance of their centroid to the eye.
		std::vector<std::pair<double, const Triangle*>> triangles_with_distance;

		for (const auto& triangle : triangles) {
			triangles_with_distance.emplace_back(((triangle.a + triangle.b + triangle.c) / 3.0 - eye).squaredNorm(), &triangle);
		}
		return triangles_with_distance;
	}
}