
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/src/Geometry/ParametrizedLine.h>
#include <boost/range/irange.hpp>
#include "../src/utilities/math_utils.h"

TEST(MathTest, closest_point_test) {


	Eigen::ParametrizedLine<double, 3> l1(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.5, 0, 0));

	Eigen::ParametrizedLine<double, 3> l2(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 10.0, 0));
	l2.origin() = l2.pointAt(5.0);

	auto [t,s] = closest_point_on_line(l1, l2);

	ASSERT_NEAR(t, 0, 1.0e-6);
	ASSERT_NEAR(s, -5.0, 1.0e-6);


}

TEST(MathTest, line_aabb_intersection_params) {

	// Get an RNG.
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-2.0, 2.0);

	for (int rep_i : boost::irange(0,100)) {
		// Create an AABB.
		Eigen::Vector3d min_corner{dis(gen), dis(gen), dis(gen)};
		Eigen::Vector3d max_corner{dis(gen), dis(gen), dis(gen)};
		// Swap the corners if necessary.
		for (int i = 0; i < 3; ++i) {
			if (min_corner[i] > max_corner[i]) {
				std::swap(min_corner[i], max_corner[i]);
			}
		}
		Eigen::AlignedBox3d aabb(min_corner, max_corner);

		// Create a line from the center; it should always intersect.

		Eigen::Vector3d center = aabb.center();
		Eigen::Vector3d direction = Eigen::Vector3d::Random() * 2.0;

		Eigen::ParametrizedLine<double, 3> line(center, direction);

		using namespace math_utils;

		auto isect = line_aabb_intersection_params(aabb, line);

		ASSERT_TRUE(isect.has_value());
		ASSERT_LE(isect->at(0), isect->at(1));

		// Compute the points.
		Eigen::Vector3d pointA = line.pointAt(isect->at(0));
		Eigen::Vector3d pointB = line.pointAt(isect->at(1));

		// Check that the points are on the AABB surface.
		ASSERT_TRUE(std::abs(pointA.x() - aabb.center().x()) <= aabb.sizes().x() / 2.0 ||
					std::abs(pointA.y() - aabb.center().y()) <= aabb.sizes().y() / 2.0 ||
					std::abs(pointA.z() - aabb.center().z()) <= aabb.sizes().z() / 2.0);

		ASSERT_TRUE(std::abs(pointB.x() - aabb.center().x()) <= aabb.sizes().x() / 2.0 ||
					std::abs(pointB.y() - aabb.center().y()) <= aabb.sizes().y() / 2.0 ||
					std::abs(pointB.z() - aabb.center().z()) <= aabb.sizes().z() / 2.0);

		// Pick a random direction.
		Eigen::Vector3d random_direction = Eigen::Vector3d::Random();

		// Pick a point a random distance away from pointA in the random direction.
		double distance = dis(gen);
		Eigen::Vector3d random_point = pointA + distance * random_direction;

		// Now, construct a line from that point and the opposite direction.
		Eigen::ParametrizedLine<double, 3> line2(random_point, -random_direction);

		// Check that the line intersects the AABB at pointA. There may be 2 intersections, and it may be either one.
		auto isect2 = line_aabb_intersection_params(aabb, line2);

		ASSERT_TRUE(isect2.has_value());

		ASSERT_TRUE((line2.pointAt(isect2->at(0)) - pointA).squaredNorm() < 1.0e-6 ||
					(line2.pointAt(isect2->at(1)) - pointA).squaredNorm() < 1.0e-6);
	}
}