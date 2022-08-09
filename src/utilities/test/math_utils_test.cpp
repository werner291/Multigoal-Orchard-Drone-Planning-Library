#include <gtest/gtest.h>
#include <ompl/util/RandomNumbers.h>
#include <Eigen/Core>
#include "../math_utils.h"

TEST(MathUtilsTest, projectionParameter) {

	ompl::RNG rng(42);

	for (size_t i = 0; i < 1000; ++i) {

		Eigen::ParametrizedLine<double, 3> l1(
				Eigen::Vector3d(
				rng.uniformReal(-10.0, 10.0),
				rng.uniformReal(-10.0, 10.0),
				rng.uniformReal(-10.0, 10.0)
				),
				Eigen::Vector3d(
						rng.uniformReal(-10.0, 10.0),
						rng.uniformReal(-10.0, 10.0),
						rng.uniformReal(-10.0, 10.0)));


		Eigen::Vector3d p = Eigen::Vector3d(
				rng.uniformReal(-10.0, 10.0),
				rng.uniformReal(-10.0, 10.0),
				rng.uniformReal(-10.0, 10.0)
				);

		double t = projectionParameter(l1, p);

		Eigen::Vector3d q = l1.pointAt(t);

		double c = (p-q).dot(l1.direction());

		ASSERT_NEAR(c, 0, 1.0e-6);

	}

}

TEST(MathUtilsTest, test_closest_point) {

	ompl::RNG rng(42);

	for (size_t i = 0; i < 1000; ++i) {

		Eigen::Vector3d a(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));
		Eigen::Vector3d b(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));
		Eigen::Vector3d c(rng.uniformReal(-1, 1), rng.uniformReal(-1, 1), rng.uniformReal(-1, 1));

		Eigen::Vector3d normal = (b - a).cross(c - a).normalized();

		Eigen::Vector3d ab_normal = (b - a).cross(normal).normalized();
		Eigen::Vector3d bc_normal = (c - b).cross(normal).normalized();
		Eigen::Vector3d ca_normal = (a - c).cross(normal).normalized();

		Eigen::Vector3d pt_ab = a + rng.uniformReal(0, 1) * (b - a);
		Eigen::Vector3d pt_bc = b + rng.uniformReal(0, 1) * (c - b);
		Eigen::Vector3d pt_ca = c + rng.uniformReal(0, 1) * (a - c);

		Eigen::Vector3d pt_ab_query = pt_ab + ab_normal * rng.uniformReal(0.0, 1) + normal * rng.uniformReal(0.0, 1);
		Eigen::Vector3d pt_bc_query = pt_bc + bc_normal * rng.uniformReal(0.0, 1) + normal * rng.uniformReal(0.0, 1);
		Eigen::Vector3d pt_ca_query = pt_ca + ca_normal * rng.uniformReal(0.0, 1) + normal * rng.uniformReal(0.0, 1);

		Eigen::Vector3d closest_pt_ab = closest_point_on_triangle(pt_ab_query, a, b, c);
		Eigen::Vector3d closest_pt_bc = closest_point_on_triangle(pt_bc_query, a, b, c);
		Eigen::Vector3d closest_pt_ca = closest_point_on_triangle(pt_ca_query, a, b, c);

		ASSERT_NEAR((closest_pt_ab - pt_ab).squaredNorm(), 0.0, 1.0e-6);
		ASSERT_NEAR((closest_pt_bc - pt_bc).squaredNorm(), 0.0, 1.0e-6);
		ASSERT_NEAR((closest_pt_ca - pt_ca).squaredNorm(), 0.0, 1.0e-6);

	}



}