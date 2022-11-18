// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 18-11-22.
//

#include "TriangleAABB.h"
#include "exploration/VtkToPointCloud.h"

TriangleAABB::TriangleAABB(const shape_msgs::msg::Mesh &mesh) {


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

	tree.insert(triangles.begin(), triangles.end());

}

std::pair<size_t, double> TriangleAABB::closest(Eigen::Vector3d &point) const {

	Point p(point.x(), point.y(), point.z());

	auto closest = tree.closest_point_and_primitive(p);

	return {closest.second - triangles.begin(), CGAL::to_double(squared_distance(closest.first,closest.first))};

}

PointOnMeshLookup::PointOnMeshLookup(const std::vector<shape_msgs::msg::Mesh> &meshes) : aabb(combine_meshes(meshes)) {


		triangle_to_fruit.reserve(meshes.size());
		for (size_t fruit_id = 0; fruit_id < meshes.size(); fruit_id++) {
			for (size_t triangle_id = 0; triangle_id <
										 meshes[fruit_id].triangles
												 .size(); triangle_id++) {
				triangle_to_fruit.push_back(fruit_id);
			}
		}



}

std::optional<size_t> PointOnMeshLookup::on_which_mesh(Eigen::Vector3d &point, double margin) const {

	auto triangle_id = aabb.closest(point);

	std::cout << "Triangle id: " << triangle_id.first << " distance: " << triangle_id.second << " mesh id: " << triangle_to_fruit[triangle_id.first] << std::endl;

	if (triangle_id.second < margin) {
		return triangle_to_fruit[triangle_id.first];
	} else {
		return std::nullopt;
	}


}

SegmentedPointCloud::ByType PointSegmenter::segmentPointCloudData(vtkPolyData *pPolyData) {

	auto points = ::segmentPointCloudData(pPolyData);

	SegmentedPointCloud::ByType segmented_points;

	for (auto &point : points.points) {

		switch (point.type) {
			case SegmentedPointCloud::PT_TARGET:

				if (auto tri = lookup.on_which_mesh(point.position, 0.01)) {

					segmented_points.target.push_back({
						point.position,
						*tri
					});

				}
				break;
			case SegmentedPointCloud::PT_OBSTACLE:
				segmented_points.obstacle.push_back(point.position);
				break;
			case SegmentedPointCloud::PT_SOFT_OBSTACLE:
				segmented_points.soft_obstacle.push_back(point.position);
				break;
		}

	}

	return segmented_points;

}

PointSegmenter::PointSegmenter(const WorkspaceSpec &spec) : lookup(PointOnMeshLookup(spec.orchard.trees[0].second.fruit_meshes)) {
}
