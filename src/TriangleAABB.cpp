// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 18-11-22.
//

#include "TriangleAABB.h"
#include "exploration/VtkToPointCloud.h"

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

PointOnMeshLookup::PointOnMeshLookup(const std::vector<shape_msgs::msg::Mesh> &meshes) : aabb(combine_meshes(meshes)) {

	// For every mesh...
	for (size_t mesh_id = 0; mesh_id < meshes.size(); mesh_id++) {
		// ...and every triangle in that mesh...
		for (size_t triangle_id = 0; triangle_id < meshes[mesh_id].triangles.size(); triangle_id++) {
			// Push a copy of the mesh ID to the triangle ID mapping, such that the triangle ID
			// can be mapped back to the mesh ID in O(1) time.
			triangle_id_to_mesh_id.push_back(mesh_id);
		}
	}
}

std::optional<size_t> PointOnMeshLookup::on_which_mesh(Eigen::Vector3d &point, double margin) const {

	// Look up the ID of the closest triangle to the point
	auto triangle_id = aabb.closest_triangle_to_point(point);

	if (triangle_id.second < margin) {
		// If it's within the margin, return the mesh ID through `triangle_id_to_mesh_id`
		return triangle_id_to_mesh_id[triangle_id.first];
	} else {
		// Otherwise, return nothing
		return std::nullopt;
	}

}

SegmentedPointCloud::ByType PointSegmenter::segmentPointCloudData(vtkPolyData *pPolyData) {

	// Convert the VTK polydata to a point cloud, where every point is
	// assigned a `SegmentedPointCloud::Type` based on its color.
	auto points = ::segmentPointCloudData(pPolyData);

	// Allocate a struct to sort the points into for each type
	SegmentedPointCloud::ByType segmented_points;

	for (auto &point : points.points) {

		switch (point.type) {
			case SegmentedPointCloud::PT_TARGET:

				// If the point is a target, locate the mesh that it's on
				if (auto mesh_id = lookup.on_which_mesh(point.position, 0.01)) {

					// If it's on a mesh, add it to the target points
					segmented_points.target.push_back({
						point.position,
						*mesh_id
					});

				}
				// Else, this must be noise. Ignore it.
				break;

			case SegmentedPointCloud::PT_OBSTACLE:
				// Obstacle points are easy: just push them onto the obstacle vector
				segmented_points.obstacle.push_back(point.position);
				break;
			case SegmentedPointCloud::PT_SOFT_OBSTACLE:
				// Same for soft obstacles
				segmented_points.soft_obstacle.push_back(point.position);
				break;
		}

	}

	// Return the segmented points
	return segmented_points;

}

PointSegmenter::PointSegmenter(const WorkspaceSpec &spec) : lookup(PointOnMeshLookup(spec.orchard.trees[0].second.fruit_meshes)) {
}
