// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-1-24.
//

#include "surface_points.h"
#include "../math/Triangle.h"

namespace mgodpl {

	size_t SeenPoints::count_seen() const
	{
		return std::count(ever_seen.begin(), ever_seen.end(), true);
	}

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
		// Mesh must be non-empty:
		assert(!mesh.vertices.empty());
		assert(!mesh.triangles.empty());

		// Prepare a vector to store cumulative areas of triangles
		std::vector<double> cumulative_areas = triangle_cumulative_areas(mesh);
		double total_area = cumulative_areas.back();

		// Prepare a vector to store the sampled points
		std::vector<SurfacePoint> points;
		points.reserve(num_points);

		// Sample points on the mesh
		for (size_t i = 0; i < num_points; ++i) {

			// Store the position and normal in the points vector
			points.push_back(sample_point_on_mesh(rng, mesh, cumulative_areas));
		}

		// Return the sampled points
		return points;
	}

	SurfacePoint sample_point_on_mesh(random_numbers::RandomNumberGenerator &rng,
									  const shape_msgs::msg::Mesh &mesh,
									  const std::vector<double> &cumulative_areas) {

		assert(!cumulative_areas.empty());

		double total_area = cumulative_areas.back();

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

		SurfacePoint point = {position, normal};
		return point;
	}

	ScannablePoints createScannablePoints(random_numbers::RandomNumberGenerator& rng, const shape_msgs::msg::Mesh& mesh,
		size_t num_points, double max_distance, double min_distance, double max_angle, std::optional<std::shared_ptr<MeshOcclusionModel>> occlusion_model)
	{
		return {max_distance, min_distance, max_angle, sample_points_on_mesh(rng, mesh, num_points), occlusion_model};
	}

	bool is_visible(const ScannablePoints& scannable_points, size_t point_index, const math::Vec3d& eye_position)
	{
		// Get the point from the ScannablePoints object
		const SurfacePoint& point = scannable_points.surface_points[point_index];

		// Calculate the vector from the point to the eye position
		auto delta = eye_position - point.position;

		// Calculate the distance from the point to the eye position
		double distance = delta.norm();

		// If the distance is greater than the maximum distance, the point is not visible
		if (distance > scannable_points.max_distance)
		{
			return false;
		}

		// If the distance is smaller than the minimum distance, the point is not visible
		if (distance < scannable_points.min_distance)
		{
			return false;
		}

		// Calculate the angle between the point's normal and the vector from the point to the eye
		double angle = std::acos(point.normal.dot(delta) / distance);

		// If the angle is greater than the maximum angle, the point is not visible
		if (angle > scannable_points.max_angle)
		{
			return false;
		}

		// Saving the most expensive calculation for last: the occlusion check
		if (scannable_points.occlusion_model.has_value()) {
			return !(*scannable_points.occlusion_model)->checkOcclusion(point.position, eye_position);
		}

		// If the point passed both the distance and angle checks, it is visible
		return true;
	}

	size_t update_visibility(const ScannablePoints& scannable_points, const math::Vec3d& eye_position,
	                         SeenPoints& seen_points)
	{
		size_t n_seen = 0;
		for (size_t i = 0; i < scannable_points.surface_points.size(); ++i)
		{
			if (!seen_points.ever_seen[i] && is_visible(scannable_points, i, eye_position))
			{
				seen_points.ever_seen[i] = true;
				++n_seen;
			}
		}
		return n_seen;
	}

	std::vector<double> triangle_cumulative_areas(const shape_msgs::msg::Mesh &mesh) {
		std::vector<double> cumulative_areas;
		cumulative_areas.reserve(mesh.triangles.size());
		double total_area = 0;
		for (const auto &triangle : mesh.triangles) {
			const auto &a = mesh.vertices[triangle.vertex_indices[0]];
			const auto &b = mesh.vertices[triangle.vertex_indices[1]];
			const auto &c = mesh.vertices[triangle.vertex_indices[2]];
			total_area += math::Triangle(math::Vec3d(a.x, a.y, a.z),
										 math::Vec3d(b.x, b.y, b.z),
										 math::Vec3d(c.x, c.y, c.z)).area();
			cumulative_areas.push_back(total_area);
		}
		return cumulative_areas;
	}

	bool is_visible(const SurfacePoint &point,
					const math::Vec3d &eye_pos,
					const math::Vec3d &eye_forward,
					double max_distance,
					double min_distance,
					double max_scan_angle,
					double fov_angle,
					const MeshOcclusionModel &mesh_occlusion_model) {
		// Calculate the vector from the point to the eye position
		auto delta = eye_pos - point.position;

		// Calculate the distance from the point to the eye position
		double distance = delta.norm();

		// Check if the point is visible based on the following conditions:
		// 1. The distance from the point to the eye position is within the specified range (min_distance to max_distance).
		// 2. The angle between the point's normal and the vector from the point to the eye position is less than or equal to the maximum scan angle.
		// 3. The angle between the eye forward direction and the vector from the point to the eye position is less than or equal to the field of view angle.
		// 4. The point is not occluded according to the mesh occlusion model. (most expensive, so last)
		return distance <= max_distance &&
			   distance >= min_distance &&
			   std::acos(point.normal.dot(delta) / distance) <= max_scan_angle &&
			   std::acos(eye_forward.dot(-delta) / distance) <= fov_angle &&
			   !mesh_occlusion_model.checkOcclusion(point.position, eye_pos);
	}
}
