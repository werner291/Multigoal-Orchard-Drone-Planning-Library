
#include <gtest/gtest.h>
#include <ompl/util/RandomNumbers.h>
#include <Eigen/Core>
#include "../src/StreamingConvexHull.h"
#include "../src/utilities/math_utils.h"
#include "../src/utilities/geogebra.h"

TEST(StreamingConvexHull_test, deterministic_simple) {

	StreamingConvexHull sch = StreamingConvexHull::fromSpherifiedCube(1);

	sch.addPoint(Eigen::Vector3d(0.5, 0.5, 0.5));
	sch.addPoint(Eigen::Vector3d(0.5, 0.5, -0.5));
	sch.addPoint(Eigen::Vector3d(0.5, -0.5, 0.5));
	sch.addPoint(Eigen::Vector3d(0.5, -0.5, -0.5));
	sch.addPoint(Eigen::Vector3d(-0.5, 0.5, 0.5));
	sch.addPoint(Eigen::Vector3d(-0.5, 0.5, -0.5));
	sch.addPoint(Eigen::Vector3d(-0.5, -0.5, 0.5));
	sch.addPoint(Eigen::Vector3d(-0.5, -0.5, -0.5));

	EXPECT_EQ(sch.getSupportingSet().size(), 8);

	auto mesh = sch.toMesh();

	// The mesh should have 8 vertices and 12 triangles.
	EXPECT_EQ(mesh.vertices.size(), 8);
	EXPECT_EQ(mesh.triangles.size(), 12);

}


TEST(StreamingConvexHull_test, randomized) {

	StreamingConvexHull sch = StreamingConvexHull::fromSpherifiedCube(3);

	// Generate 100 points in a sphere.
	std::vector<Eigen::Vector3d> points;

	ompl::RNG rng;

	for (int i = 0; i < 100; i++) {
		points.emplace_back(rng.gaussian(0, 1), rng.gaussian(0, 1), rng.gaussian(0, 1));
		points.back().normalize();

		sch.addPoint(points.back());
	}


	// Check if all points of the hull are in the sphere.
	shape_msgs::msg::Mesh mesh = sch.toMesh();

	for (const auto &vertex : mesh.vertices) {
		Eigen::Vector3d point(vertex.x, vertex.y, vertex.z);
		EXPECT_LE(point.norm(), 1.0 + 1e-6);
	}

	// Check if all points of the sphere are in the hull by at least a small margin
	for (const auto &point : points) {
		Eigen::Vector3d closest_point;
		double distance = std::numeric_limits<double>::infinity();
		for (const auto &triangle: mesh.triangles) {
			Eigen::Vector3d a(mesh.vertices[triangle.vertex_indices[0]].x, mesh.vertices[triangle.vertex_indices[0]].y, mesh.vertices[triangle.vertex_indices[0]].z);
			Eigen::Vector3d b(mesh.vertices[triangle.vertex_indices[1]].x, mesh.vertices[triangle.vertex_indices[1]].y, mesh.vertices[triangle.vertex_indices[1]].z);
			Eigen::Vector3d c(mesh.vertices[triangle.vertex_indices[2]].x, mesh.vertices[triangle.vertex_indices[2]].y, mesh.vertices[triangle.vertex_indices[2]].z);
			Eigen::Vector3d closest = closest_point_on_triangle(point, a, b, c);
			double distance_to_triangle = (closest - point).norm();
			if (distance_to_triangle < distance) {
				distance = distance_to_triangle;
				closest_point = closest;
			}
		}
		EXPECT_LE((closest_point - point).norm(), 0.2);
	}


}