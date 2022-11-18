// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 18-11-22.
//

#include <gtest/gtest.h>
#include <ompl/util/RandomNumbers.h>
#include <shape_msgs/msg/mesh.hpp>
#include "../src/TriangleAABB.h"
#include "../src/utilities/msgs_utilities.h"

TEST(PointOnMeshLookup, test) {

	ompl::RNG rng;

	// Generate two meshes at random

	shape_msgs::msg::Mesh mesh1;

	for (size_t i = 0; i < 10; i++) {

		// Generate a random triangle

		geometry_msgs::msg::Point p1;
		p1.x = rng.uniformReal(-2, -1);
		p1.y = rng.uniformReal(-1, 1);
		p1.z = rng.uniformReal(-1, 1);

		geometry_msgs::msg::Point p2;
		p1.x = rng.uniformReal(-2, -1);
		p2.y = rng.uniformReal(-1, 1);
		p2.z = rng.uniformReal(-1, 1);

		geometry_msgs::msg::Point p3;
		p1.x = rng.uniformReal(-2, -1);
		p3.y = rng.uniformReal(-1, 1);
		p3.z = rng.uniformReal(-1, 1);

		mesh1.vertices.push_back(p1);
		mesh1.vertices.push_back(p2);
		mesh1.vertices.push_back(p3);

		shape_msgs::msg::MeshTriangle triangle;
		triangle.vertex_indices[0] = i * 3;
		triangle.vertex_indices[1] = i * 3 + 1;
		triangle.vertex_indices[2] = i * 3 + 2;

		mesh1.triangles.push_back(triangle);

	}

	shape_msgs::msg::Mesh mesh2;

	for (size_t i = 0; i < 10; i++) {

		// Generate a random triangle

		geometry_msgs::msg::Point p1;
		p1.x = rng.uniformReal(1, 2);
		p1.y = rng.uniformReal(-1, 1);
		p1.z = rng.uniformReal(-1, 1);

		geometry_msgs::msg::Point p2;
		p2.x = rng.uniformReal(1, 2);
		p2.y = rng.uniformReal(-1, 1);
		p2.z = rng.uniformReal(-1, 1);

		geometry_msgs::msg::Point p3;
		p3.x = rng.uniformReal(1, 2);
		p3.y = rng.uniformReal(-1, 1);
		p3.z = rng.uniformReal(-1, 1);

		mesh2.vertices.push_back(p1);
		mesh2.vertices.push_back(p2);
		mesh2.vertices.push_back(p3);

		shape_msgs::msg::MeshTriangle triangle;
		triangle.vertex_indices[0] = i * 3;
		triangle.vertex_indices[1] = i * 3 + 1;
		triangle.vertex_indices[2] = i * 3 + 2;

		mesh2.triangles.push_back(triangle);

	}

	std::vector<shape_msgs::msg::Mesh> meshes {mesh1, mesh2};

	PointOnMeshLookup lookup(meshes);

	// 100 times, pick a triangle and a point on that triangle, and check if the lookup returns the correct triangle

	for (size_t i = 0; i < 100; ++i) {

		// Pick a random mesh
		size_t mesh_id = rng.uniformInt(0, 1);

		// Pick a random triangle
		size_t triangle_id = rng.uniformInt(0, 9);

		// Pick a random point on that triangle
		Eigen::Vector3d p1 = toEigen(meshes[mesh_id].vertices[meshes[mesh_id].triangles[triangle_id].vertex_indices[0]]);
		Eigen::Vector3d p2 = toEigen(meshes[mesh_id].vertices[meshes[mesh_id].triangles[triangle_id].vertex_indices[1]]);
		Eigen::Vector3d p3 = toEigen(meshes[mesh_id].vertices[meshes[mesh_id].triangles[triangle_id].vertex_indices[2]]);

		Eigen::Vector3d p = p1 + rng.uniformReal(0, 1) * (p2 - p1) + rng.uniformReal(0, 1) * (p3 - p1);

		// Check if the lookup returns the correct triangle
		auto res = lookup.on_which_mesh(p, 1.0e-6);

		EXPECT_EQ(mesh_id, *res);
	}

}