
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/src/Geometry/ParametrizedLine.h>
#include "../src/math.h"

TEST(MathTest, closest_point_test) {


	Eigen::ParametrizedLine<double, 3> l1(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.5, 0, 0));

	Eigen::ParametrizedLine<double, 3> l2(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 10.0, 0));
	l2.origin() = l2.pointAt(5.0);

	auto [t,s] = closest_point_on_line(l1, l2);

	ASSERT_NEAR(t, 0, 1.0e-6);
	ASSERT_NEAR(s, -5.0, 1.0e-6);


}