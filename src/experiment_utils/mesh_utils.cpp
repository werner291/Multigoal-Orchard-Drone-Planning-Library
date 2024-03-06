
#include "mesh_utils.h"

//Eigen::Vector3d closestPointOnMesh(const shape_msgs::msg::Mesh &mesh, const Eigen::Vector3d &query_point) {
//	double closest_distance = std::numeric_limits<double>::max();
//	Eigen::Vector3d closest_point;
//
//	for (const auto &tr: mesh.triangles) {
//		Eigen::Vector3d a = toEigen(mesh.vertices[tr.vertex_indices[0]]);
//		Eigen::Vector3d b = toEigen(mesh.vertices[tr.vertex_indices[1]]);
//		Eigen::Vector3d c = toEigen(mesh.vertices[tr.vertex_indices[2]]);
//
//		Eigen::Vector3d candidate = closest_point_on_triangle(query_point, a, b, c);
//
//		double distance = (query_point - candidate).norm();
//		if (distance < closest_distance) {
//			closest_distance = distance;
//			closest_point = candidate;
//		}
//	}
//	return closest_point;
//}

shape_msgs::msg::Mesh combine_meshes(const std::vector<shape_msgs::msg::Mesh> &meshes) {

	shape_msgs::msg::Mesh combined_mesh;

	// For every mesh...
	for (const auto &mesh : meshes) {

		size_t index_offset = combined_mesh.vertices.size();

		// ...and for every vertex in the mesh...
		for (const auto &vertex : mesh.vertices) {
			// ...add the vertex to the combined mesh.
			combined_mesh.vertices.push_back(vertex);
		}

		// For every triangle in the mesh...
		for (const auto &triangle : mesh.triangles) {
			// ...add the triangle to the combined mesh.
			combined_mesh.triangles.push_back(triangle);

			// ...and adjust the vertex indices to account for the offset.
			combined_mesh.triangles.back().vertex_indices[0] += index_offset;
			combined_mesh.triangles.back().vertex_indices[1] += index_offset;
			combined_mesh.triangles.back().vertex_indices[2] += index_offset;
		}

	}

	return combined_mesh;

}

mgodpl::math::AABBd mesh_aabb(const shape_msgs::msg::Mesh &mesh) {

	mgodpl::math::AABBd aabb = mgodpl::math::AABBd::inverted_infinity();

	for (const auto &vertex : mesh.vertices) {
		aabb.expand({vertex.x, vertex.y, vertex.z});
	}

	return aabb;
}

shape_msgs::msg::Mesh createGroundPlane(double width, double height) {
	shape_msgs::msg::Mesh ground_plane;

	// Define the vertices of the ground plane.
	double half_width = 0.5 * width;
	double half_height = 0.5 * height;

	ground_plane.vertices.resize(4);
	ground_plane.vertices[0].x = -half_width;
	ground_plane.vertices[0].y = -half_height;
	ground_plane.vertices[0].z = 0.0;

	ground_plane.vertices[1].x = half_width;
	ground_plane.vertices[1].y = -half_height;
	ground_plane.vertices[1].z = 0.0;

	ground_plane.vertices[2].x = half_width;
	ground_plane.vertices[2].y = half_height;
	ground_plane.vertices[2].z = 0.0;

	ground_plane.vertices[3].x = -half_width;
	ground_plane.vertices[3].y = half_height;
	ground_plane.vertices[3].z = 0.0;

	// Define the triangles forming the ground plane.
	ground_plane.triangles.resize(2);
	ground_plane.triangles[0].vertex_indices = {0, 1, 2};
	ground_plane.triangles[1].vertex_indices = {0, 2, 3};

	return ground_plane;
}

void append_mesh(shape_msgs::msg::Mesh &left_mesh, const shape_msgs::msg::Mesh &right_mesh) {

	// For the right-hand mesh...
	size_t index_offset = left_mesh.vertices.size();

	// ...and for every vertex in the right-hand mesh...
	for (const auto &vertex : right_mesh.vertices) {
		// ...add the vertex to the left-hand mesh.
		left_mesh.vertices.push_back(vertex);
	}

	// For every triangle in the right-hand mesh...
	for (const auto &triangle : right_mesh.triangles) {
		// ...add the triangle to the left-hand mesh.
		left_mesh.triangles.push_back(triangle);

		// ...and adjust the vertex indices to account for the offset.
		left_mesh.triangles.back().vertex_indices[0] += index_offset;
		left_mesh.triangles.back().vertex_indices[1] += index_offset;
		left_mesh.triangles.back().vertex_indices[2] += index_offset;
	}
}

std::vector<std::array<mgodpl::math::Vec3d, 3>> triangles_from_mesh(const shape_msgs::msg::Mesh &leaves_mesh) {

	// Initialize a vector to store the vertices
	std::vector<std::array<mgodpl::math::Vec3d,3>> vertices;

	// Iterate over the triangles in the mesh
	for (const auto& triangle : leaves_mesh.triangles) {

		// Extract the first vertex of the triangle and store it in a mgodpl::math::Vec3d
		mgodpl::math::Vec3d vertex1 = {leaves_mesh.vertices[triangle.vertex_indices[0]].x,
									   leaves_mesh.vertices[triangle.vertex_indices[0]].y,
									   leaves_mesh.vertices[triangle.vertex_indices[0]].z};

		// Extract the second vertex of the triangle and store it in a mgodpl::math::Vec3d
		mgodpl::math::Vec3d vertex2 = {leaves_mesh.vertices[triangle.vertex_indices[1]].x,
									   leaves_mesh.vertices[triangle.vertex_indices[1]].y,
									   leaves_mesh.vertices[triangle.vertex_indices[1]].z};

		// Extract the third vertex of the triangle and store it in a mgodpl::math::Vec3d
		mgodpl::math::Vec3d vertex3 = {leaves_mesh.vertices[triangle.vertex_indices[2]].x,
									   leaves_mesh.vertices[triangle.vertex_indices[2]].y,
									   leaves_mesh.vertices[triangle.vertex_indices[2]].z};

		// Add the vertices to the vector
		vertices.push_back({vertex1, vertex2, vertex3});
	}

	// Return the vector of vertices
	return vertices;
}

//shape_msgs::msg::Mesh translate_mesh(shape_msgs::msg::Mesh mesh, const Eigen::Vector3d &translation) {
//
//	for (auto &vertex : mesh.vertices) {
//		vertex.x += translation.x();
//		vertex.y += translation.y();
//		vertex.z += translation.z();
//	}
//
//	return mesh;
//
//}
