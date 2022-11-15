
#include "mesh_utils.h"
#include "moveit_conversions.h"
#include "msgs_utilities.h"
#include "math_utils.h"

Eigen::Vector3d closestPointOnMesh(const shape_msgs::msg::Mesh &mesh, const Eigen::Vector3d &query_point) {
	double closest_distance = std::numeric_limits<double>::max();
	Eigen::Vector3d closest_point;

	for (const auto &tr: mesh.triangles) {
		Eigen::Vector3d a = toEigen(mesh.vertices[tr.vertex_indices[0]]);
		Eigen::Vector3d b = toEigen(mesh.vertices[tr.vertex_indices[1]]);
		Eigen::Vector3d c = toEigen(mesh.vertices[tr.vertex_indices[2]]);

		Eigen::Vector3d candidate = closest_point_on_triangle(query_point, a, b, c);

		double distance = (query_point - candidate).norm();
		if (distance < closest_distance) {
			closest_distance = distance;
			closest_point = candidate;
		}
	}
	return closest_point;
}
