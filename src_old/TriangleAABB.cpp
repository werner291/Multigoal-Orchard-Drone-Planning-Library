// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 18-11-22.
//

#include "TriangleAABB.h"

TriangleAABB::TriangleAABB(const shape_msgs::msg::Mesh &mesh) {

	// Convert the mesh's triangles to a vector of CGAL triangles
	for (const auto &triangle : mesh.triangles) {

		Point p1(mesh.vertices[triangle.vertex_indices[0]].x,
				 mesh.vertices[triangle.vertex_indices[0]].y,
				 mesh.vertices[triangle.vertex_indices[0]].z);

		Point p2(mesh.vertices[triangle.vertex_indices[1]].x,
				 mesh.vertices[triangle.vertex_indices[1]].y,
				 mesh.vertices[triangle.vertex_indices[1]].z);

		Point p3(mesh.vertices[triangle.vertex_indices[2]].x,
				 mesh.vertices[triangle.vertex_indices[2]].y,
				 mesh.vertices[triangle.vertex_indices[2]].z);

		triangles.emplace_back(p1, p2, p3);
	}

	// Build the AABB tree
	tree.insert(triangles.begin(), triangles.end());

}

std::pair<size_t, double> TriangleAABB::closest_triangle_to_point(Eigen::Vector3d &query_point) const {

	// Convert to CGAL
	Point q_cgal(query_point.x(), query_point.y(), query_point.z());

	// Find the closest triangle and the point on that triangle
	const auto& [closest_point, triangle_itr] = tree.closest_point_and_primitive(q_cgal);

	return {
		// CGAL returns an iterator into the `triangles` vector, so we subtract the begin iterator to get the index
		triangle_itr - triangles.begin(),
		// The square distance between the query point and the closest point on the triangle
		CGAL::to_double(squared_distance(q_cgal, closest_point))
	};

}