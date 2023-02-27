//
// Created by werner on 27-2-23.
//

#include "MeshOcclusionModel.h"

MeshOcclusionModel::MeshOcclusionModel(const shape_msgs::msg::Mesh &mesh) {

	// Use CGAL to create an AABB tree of the mesh's triangles.

	for (const auto &triangle : mesh.triangles) {
		const auto &p1 = mesh.vertices[triangle.vertex_indices[0]];
		const auto &p2 = mesh.vertices[triangle.vertex_indices[1]];
		const auto &p3 = mesh.vertices[triangle.vertex_indices[2]];

		triangles.emplace_back(Point(p1.x, p1.y, p1.z), Point(p2.x, p2.y, p2.z), Point(p3.x, p3.y, p3.z));
	}

	tree.insert(triangles.begin(), triangles.end());

}

bool
MeshOcclusionModel::checkOcclusion(const Eigen::Vector3d &point, const Eigen::Vector3d &viewpoint) const {

	Segment segment(Point(viewpoint.x(), viewpoint.y(), viewpoint.z()),
					Point(point.x(), point.y(), point.z()));

	// Check if the line intersects the mesh.
	return tree.do_intersect(segment);

}
