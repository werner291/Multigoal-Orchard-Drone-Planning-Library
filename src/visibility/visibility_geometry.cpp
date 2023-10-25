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

	inline bool triangleBroadlyIntersects(const AABBd &aabb, const Triangle &triangle) {
		return std::min({triangle.a.x(), triangle.b.x(), triangle.c.x()}) <= aabb.max().x() &&
			   std::max({triangle.a.x(), triangle.b.x(), triangle.c.x()}) >= aabb.min().x() &&
			   std::min({triangle.a.y(), triangle.b.y(), triangle.c.y()}) <= aabb.max().y() &&
			   std::max({triangle.a.y(), triangle.b.y(), triangle.c.y()}) >= aabb.min().y() &&
			   std::min({triangle.a.z(), triangle.b.z(), triangle.c.z()}) <= aabb.max().z() &&
			   std::max({triangle.a.z(), triangle.b.z(), triangle.c.z()}) >= aabb.min().z();

//		if (triangle_aabb_broadly_intersects) {
//
//			// Then, we consider the three edges of the triangle.
//			std::array<Segment3d, 3> segments{{Segment3d{triangle.a, triangle.b},
//											   Segment3d{triangle.b, triangle.c},
//											   Segment3d{triangle.c, triangle.a}}};
//
//			// We want to find what part of the edge falls inside of the slab.
//			// For building the AABB, it is enough to use the endpoints of
//			// the portion of the edge that falls inside the slab.
//
//			// For every segment...
//			for (auto &segment: segments) {
//
//				// For each of the two planes...
//				for (const auto &x: {slab.range.min, slab.range.max}) {
//
//					// Find the intersection parameter t of the segment with the plane.
//					auto line = segment.extend_to_line();
//					double t = param_at_plane(line, slab.dimension, x);
//
//					if (t >= 0.0 && t <= 1.0) {
//						// The segment intersects the plane; add the point to the AABB.
//						Vec3d pt = line.pointAt(t);
//						aabb_in_dim.expand(pt);
//					}
//				}
//			}
//		}
	}

	std::optional<math::AABBd>
	aabbInAABB(const math::AABBd &aabb,
			   const std::array<mgodpl::math::Ray, 3> &rays,
			   int dim) {

		// TODO account for the triangle of origins.

		AABBd total_aabb = aabb;
//
//		const Triangle &triangle = Triangle{rays[0].origin(), rays[1].origin(), rays[2].origin()};
//		bool triangle_aabb_broadly_intersects = triangleBroadlyIntersects(aabb, triangle);

		for (const int dim: {0, 1, 2}) {

			AABBd aabb_in_dim = AABBd::inverted_infinity();

			const AASlab<double> &slab = {dim, {aabb.min()[dim], aabb.max()[dim]}};

			// For each triangle vertex, if it is in the slab, add it to the AABB.
			for (const auto &point: {rays[0].origin(), rays[1].origin(), rays[2].origin()}) {
				if (slab.range.contains(point[slab.dimension])) {
					aabb_in_dim.expand(point);
				}
			}

			for (const Ray &ray: rays) {
				double t1 = (slab.range.min - ray.origin()[slab.dimension]) / ray.direction()[slab.dimension];
				double t2 = (slab.range.max - ray.origin()[slab.dimension]) / ray.direction()[slab.dimension];

				// If 't1' is available, expand the AABB to include the corresponding point on the ray.
				if (t1 > 0.0)
					aabb_in_dim.expand(ray.pointAt(t1));

				// If 't2' is available, expand the AABB to include the corresponding point on the ray.
				if (t2 > 0.0)
					aabb_in_dim.expand(ray.pointAt(t2));
			}

			const auto &isect = total_aabb.intersection(aabb_in_dim);

			if (!isect) {
				return std::nullopt;
			}

			total_aabb = *isect;
		}

		return total_aabb;
	}

	std::vector<std::pair<double, const math::Triangle *>>
	sorted_by_distance(const std::vector<mgodpl::math::Triangle> &triangles, const Vec3d &eye) {
		// First, sort the triangles by the distance of their centroid to the eye.
		std::vector<std::pair<double, const Triangle *>> triangles_with_distance;

		for (const auto &triangle: triangles) {
			triangles_with_distance.emplace_back(((triangle.a + triangle.b + triangle.c) / 3.0 - eye).squaredNorm(),
												 &triangle);
		}
		return triangles_with_distance;
	}
}