
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/src/Geometry/ParametrizedLine.h>
#include <ompl/util/RandomNumbers.h>
#include "../src/utilities/math_utils.h"

TEST(MathTest, closest_point_test) {


	Eigen::ParametrizedLine<double, 3> l1(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.5, 0, 0));

	Eigen::ParametrizedLine<double, 3> l2(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 10.0, 0));
	l2.origin() = l2.pointAt(5.0);

	auto [t,s] = closest_point_on_line(l1, l2);

	ASSERT_NEAR(t, 0, 1.0e-6);
	ASSERT_NEAR(s, -5.0, 1.0e-6);


}

TEST(MathTest, open_triangle_test) {

	ompl::RNG rng(42);

	for (size_t i = 0; i < 100; ++i) {

		Eigen::Vector3d apex = Eigen::Vector3d::Random();
		Eigen::Vector3d ray1 = Eigen::Vector3d::Random().normalized();
		Eigen::Vector3d ray2 = Eigen::Vector3d::Random().normalized();

		OpenTriangle tri {apex, ray1, ray2};

		double t1 = rng.uniformReal(0, 1);
		double t2 = rng.uniformReal(0, 1);
		double scale = rng.uniformReal(3.0, 5.0);

		t1 *= scale;
		t2 *= scale;

		{
				Eigen::Vector3d p = tri.apex + tri.dir1 * t1 + tri.dir2 * t2;
				Eigen::Vector3d response = closest_point_on_open_triangle(p, tri);
				ASSERT_NEAR((response - p).norm(), 0, 1.0e-6);
		}

		{
			Eigen::Vector3d p = tri.apex + tri.dir1 * t1 + tri.dir2 * t2;
			Eigen::Vector3d response = closest_point_on_open_triangle(p + rng.uniformReal(-1.0, 1.0) * ray1.cross(ray2), tri);
			ASSERT_NEAR((response - p).norm(), 0, 1.0e-6);
		}

		{
			Eigen::Vector3d p = tri.apex + tri.dir1 * (t1+1.0) + tri.dir2 * (t2-1.0);
			Eigen::Vector3d response = closest_point_on_open_triangle(p + rng.uniformReal(-1.0, 1.0) * ray1.cross(ray2), tri);

			Eigen::Vector3d expected = Ray3d(tri.apex, tri.dir1).closest_point(p);

			ASSERT_NEAR((response - expected).norm(), 0, 1.0e-6);
		}

	}

}