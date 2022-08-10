
#include <gtest/gtest.h>
#include <boost/math/special_functions/sign.hpp>

#include "../ConvexHullShell.h"
#include "../../utilities/convex_hull.h"
#include "../../utilities/math_utils.h"

class ConvexHullShell_test : public ::testing::Test {
protected:
	void SetUp() override {

		// Generate a random set of 100 points
		ompl::RNG rng(43);

		points.reserve(100);

		for (int i = 0; i < 100; i++) {
			geometry_msgs::msg::Point p;
			p.x = rng.uniformReal(-1, 1);
			p.y = rng.uniformReal(-1, 1);
			p.z = rng.uniformReal(-1, 1);
			points.push_back(p);
		}

		shell = std::make_unique<ConvexHullShell>(convexHull(points));

		std::vector<geometry_msgs::msg::Point> cube_points;
		cube_points.reserve(8);
		for (int i = 0; i < 8; i++) {
			geometry_msgs::msg::Point p;
			p.x = i & 1 ? 1 : -1;
			p.y = i & 2 ? 1 : -1;
			p.z = i & 4 ? 1 : -1;
			cube_points.push_back(p);
		}

		cube_shell = std::make_unique<ConvexHullShell>(convexHull(cube_points));


	}

	std::vector<geometry_msgs::msg::Point> points;
	std::unique_ptr<ConvexHullShell> shell;

	std::unique_ptr<ConvexHullShell> cube_shell;
};


TEST_F(ConvexHullShell_test, all_points_on_or_inside) {

	for (const auto &p : points) {
		EXPECT_TRUE(shell->signed_distance(Eigen::Vector3d(p.x, p.y, p.z)) <= 1.0e-10);
	}

}



TEST_F(ConvexHullShell_test, topology_invariants) {


	for (size_t face_i = 0; face_i < shell->num_facets(); ++face_i) {

		const auto &facet = shell->facet(face_i);

		// Check if every neighbour has a back-pointer to this face.
		for (size_t neighbour : facet.neighbours()) {
			EXPECT_TRUE(contains(shell->facet(neighbour).neighbours(), face_i));
		}

	}


}

TEST_F(ConvexHullShell_test, cube_projections) {

	ompl::RNG rng(42);

	for (size_t i = 0; i < 1000; ++i) {
		Eigen::Vector3d p;

		p.x() = rng.uniformReal(-2.0, 2.0);
		p.y() = rng.uniformReal(-2.0, 2.0);
		p.z() = rng.uniformReal(-2.0, 2.0);

		Eigen::Vector3d expected_proj;

		if (abs(p.x()) > 1.0 || abs(p.y()) > 1.0 || abs(p.z()) > 1.0) {
			expected_proj.x() = std::clamp(p.x(), -1.0, 1.0);
			expected_proj.y() = std::clamp(p.y(), -1.0, 1.0);
			expected_proj.z() = std::clamp(p.z(), -1.0, 1.0);
		} else {
			double max_dim = std::max(std::max(abs(p.x()), abs(p.y())), abs(p.z()));

			expected_proj.x() = (abs(p.x()) == max_dim) ? p.x() / max_dim : p.x();
			expected_proj.y() = (abs(p.y()) == max_dim) ? p.y() / max_dim : p.y();
			expected_proj.z() = (abs(p.z()) == max_dim) ? p.z() / max_dim : p.z();
		}

		auto proj = cube_shell->project(Apple {p, {0.0, 0.0, 0.0}});

		std::cout << "p: " << p.transpose() << std::endl;
		std::cout << "proj: " << proj.position.transpose() << std::endl;
		std::cout << "expected_proj: " << expected_proj.transpose() << std::endl;

		ASSERT_NEAR(expected_proj.x(), proj.position.x(), 1.0e-10);
		ASSERT_NEAR(expected_proj.y(), proj.position.y(), 1.0e-10);
		ASSERT_NEAR(expected_proj.z(), proj.position.z(), 1.0e-10);
	}

}

TEST_F(ConvexHullShell_test, DISABLED_brute_force_comparison) {

	ompl::RNG rng(42);

	for (size_t i = 0; i < 1000; ++i) {

		std::cout << "i: " << i << std::endl;

		Eigen::Vector3d p;

		p.x() = rng.uniformReal(-2.0, 2.0);
		p.y() = rng.uniformReal(-2.0, 2.0);
		p.z() = rng.uniformReal(-2.0, 2.0);

		double expected_signed_dist = INFINITY;
		Eigen::Vector3d expected_proj;

		for (size_t facet_index = 0; facet_index < shell->num_facets(); ++facet_index) {
			const auto& [va,vb,vc] = shell->facet_vertices(facet_index);

			Eigen::Vector3d closest_point = closest_point_on_triangle(p, va, vb, vc);

			double distance = (p - closest_point).squaredNorm();

			if (distance < expected_signed_dist) {
				expected_signed_dist = distance;
				expected_proj = closest_point;
			}
		}

		std::cout << "Expected = (" << expected_proj.x() << "," << expected_proj.y() << "," << expected_proj.z() << ")" << std::endl;

		auto pt = shell->project(Apple {p, {0.0, 0.0, 0.0}});

		assert((pt.position - expected_proj).squaredNorm() < 1.0e-10);
		ASSERT_NEAR((pt.position - expected_proj).squaredNorm(), 0.0, 1.0e-10);
	}

}

TEST_F(ConvexHullShell_test, path_on_surface_test) {

	// Pick to random points and project them onto the surface.

	ompl::RNG rng(43);

	auto start = shell->project(Apple {Eigen::Vector3d {rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0)}, {0.0, 0.0, 0.0}});
	auto goal = shell->project(Apple {Eigen::Vector3d {rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0)}, {0.0, 0.0, 0.0}});

	auto path = shell->convex_hull_walk(start, goal);

	for (auto & i : path) {
		const auto& [va1,vb1,vc1] = shell->facet_vertices(i.face_id);
		Eigen::Vector3d closest_point = closest_point_on_triangle(i.position, va1, vb1, vc1);
		ASSERT_NEAR((i.position - closest_point).squaredNorm(), 0.0, 1.0e-10);
	}

	for (size_t i = 0; i+1 < path.size(); ++i) {

		auto a = path[i];
		auto b = path[i+1];

		if (a.face_id != b.face_id) {
			ASSERT_EQ(a.position, b.position);
		}

	}

}