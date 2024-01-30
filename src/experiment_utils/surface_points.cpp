// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-1-24.
//

#include "surface_points.h"
#include "../math/Triangle.h"

namespace mgodpl {
	math::Vec3d random_barycentric(random_numbers::RandomNumberGenerator &rng) {

		double r1 = rng.uniform01();
		double r2 = rng.uniform01();

		// Reflect the point if it is outside the triangle
		if (r1 + r2 > 1.0) {
			r1 = 1.0 - r1;
			r2 = 1.0 - r2;
		}

		return {r1, r2, 1.0 - r1 - r2};
	}

	std::vector<SurfacePoint> sample_points_on_mesh(random_numbers::RandomNumberGenerator &rng,
															const shape_msgs::msg::Mesh &mesh,
															size_t num_points) {

		// Prepare a vector to store cumulative areas of triangles
		std::vector<double> cumulative_areas;
		cumulative_areas.reserve(mesh.triangles.size());

		double total_area = 0.0;

		// Calculate the total area of the mesh and the cumulative areas of the triangles
		for (const auto &triangle: mesh.triangles) {
			const auto &vertex1 = mesh.vertices[triangle.vertex_indices[0]];
			const auto &vertex2 = mesh.vertices[triangle.vertex_indices[1]];
			const auto &vertex3 = mesh.vertices[triangle.vertex_indices[2]];
			total_area += math::Triangle(math::Vec3d(vertex1.x, vertex1.y, vertex1.z),
										 math::Vec3d(vertex2.x, vertex2.y, vertex2.z),
										 math::Vec3d(vertex3.x, vertex3.y, vertex3.z)).area();
			cumulative_areas.push_back(total_area);
		}

		// Prepare a vector to store the sampled points
		std::vector<SurfacePoint> points;
		points.reserve(num_points);

		// Sample points on the mesh
		for (size_t i = 0; i < num_points; ++i) {

			// Generate a random area within the total area
			double random_area = rng.uniform01() * total_area;

			// Find the triangle that contains the random area
			auto triangle_it = std::lower_bound(cumulative_areas.begin(), cumulative_areas.end(), random_area);

			// Get the index of the triangle
			size_t triangle_index = std::distance(cumulative_areas.begin(), triangle_it);

			// Get the vertices of the triangle
			const auto &triangle = mesh.triangles[triangle_index];
			const auto &vertex1 = mesh.vertices[triangle.vertex_indices[0]];
			const auto &vertex2 = mesh.vertices[triangle.vertex_indices[1]];
			const auto &vertex3 = mesh.vertices[triangle.vertex_indices[2]];

			// Generate a random barycentric coordinate within the triangle
			math::Vec3d barycentric = random_barycentric(rng);

			// Calculate the position of the point using the barycentric coordinate
			math::Vec3d a = math::Vec3d(vertex1.x, vertex1.y, vertex1.z);
			math::Vec3d b = math::Vec3d(vertex2.x, vertex2.y, vertex2.z);
			math::Vec3d c = math::Vec3d(vertex3.x, vertex3.y, vertex3.z);
			math::Vec3d position = a * barycentric[0] + b * barycentric[1] + c * barycentric[2];

			// Calculate the normal of the triangle
			math::Vec3d normal = math::Triangle(a, b, c).normal();

			// Store the position and normal in the points vector
			points.push_back({position, normal});
		}

		// Return the sampled points
		return points;
	}

}