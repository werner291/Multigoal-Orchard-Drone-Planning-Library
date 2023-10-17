
// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <gtest/gtest.h>
#include <boost/range/irange.hpp>
#include <random>

#include "../../src/math/Vec3.h"
#include "../../src/math/ParametricLine.h"
#include "../../src/math/AABB.h"
#include "../../src/math/intersections.h"

TEST(IntersectionsTest, line_aabb_intersection_params) {

	using namespace mgodpl::math;

	// Get an RNG.
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-2.0, 2.0);

	for (int rep_i : boost::irange(0,100)) {
		// Create an AABB.
		Vec3d min_corner{dis(gen), dis(gen), dis(gen)};
		Vec3d max_corner{dis(gen), dis(gen), dis(gen)};
		// Swap the corners if necessary.
		for (int i = 0; i < 3; ++i) {
			if (min_corner[i] > max_corner[i]) {
				std::swap(min_corner[i], max_corner[i]);
			}
		}

		AABBd aabb(min_corner, max_corner);

		// Create a line from the center; it should always intersect.

		Vec3d center = aabb.center();
		Vec3d direction { dis(gen), dis(gen), dis(gen) };

		ParametricLine line(center, direction);

		auto isect = line_aabb_intersection_params(aabb, line);

		ASSERT_TRUE(isect.has_value());
		ASSERT_LE(isect->at(0), isect->at(1));

		// Compute the points.
		Vec3d pointA = line.pointAt(isect->at(0));
		Vec3d pointB = line.pointAt(isect->at(1));

		// Check that the points are on the AABB surface.
		ASSERT_TRUE(std::abs(pointA.x() - aabb.center().x()) <= aabb.size().x() / 2.0 ||
					std::abs(pointA.y() - aabb.center().y()) <= aabb.size().y() / 2.0 ||
					std::abs(pointA.z() - aabb.center().z()) <= aabb.size().z() / 2.0);

		ASSERT_TRUE(std::abs(pointB.x() - aabb.center().x()) <= aabb.size().x() / 2.0 ||
					std::abs(pointB.y() - aabb.center().y()) <= aabb.size().y() / 2.0 ||
					std::abs(pointB.z() - aabb.center().z()) <= aabb.size().z() / 2.0);

		// Pick a random direction.
		Vec3d random_direction { dis(gen), dis(gen), dis(gen) };

		// Pick a point a random distance away from pointA in the random direction.
		double distance = dis(gen);
		Vec3d random_point = pointA + random_direction * distance;

		// Now, construct a line from that point and the opposite direction.
		ParametricLine line2(random_point, -random_direction);

		// Check that the line intersects the AABB at pointA. There may be 2 intersections, and it may be either one.
		auto isect2 = line_aabb_intersection_params(aabb, line2);

		ASSERT_TRUE(isect2.has_value());

		ASSERT_TRUE((line2.pointAt(isect2->at(0)) - pointA).squaredNorm() < 1.0e-6 ||
					(line2.pointAt(isect2->at(1)) - pointA).squaredNorm() < 1.0e-6);
	}
}

TEST(IntersectionTest, no_intersection) {

	using namespace mgodpl::math;

	// Get an RNG.
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-2.0, 2.0);

	for (int i : boost::irange(0,100)) {

		// Generate an AABB
		Vec3d min_corner{dis(gen), dis(gen), dis(gen)};
		Vec3d max_corner{dis(gen), dis(gen), dis(gen)};
		// Swap the corners if necessary.
		for (int i = 0; i < 3; ++i) {
			if (min_corner[i] > max_corner[i]) {
				std::swap(min_corner[i], max_corner[i]);
			}
		}

		AABBd aabb(min_corner, max_corner);

		// Generate a parametric line that does not intersect the AABB by taking a tangent to the surface of the circumsphere.

		// Find the center of the AABB.
		Vec3d center = aabb.center();

		// Calculate the radius of the circumsphere.
		double radius = aabb.size().norm() / 2.0;

		// Generate a random unit vector that represents a point on the sphere.
		Vec3d radius_vector = Vec3d(dis(gen), dis(gen), dis(gen)).normalized();

		// Move a point slightly away from the sphere's surface.
		Vec3d point_on_sphere = center + radius_vector * (radius + 0.1);

		// Calculate a perpendicular vector for the parametric line.
		Vec3d perpendicular = Vec3d::UnitX().cross(radius_vector);

		// Create the parametric line.
		ParametricLine line(point_on_sphere, perpendicular);

		// Check if the line intersects with the AABB.
		auto isect = line_aabb_intersection_params(aabb, line);

		// Ensure that there is no intersection.
		ASSERT_FALSE(isect.has_value());
	}

}