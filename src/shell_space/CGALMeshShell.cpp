
#include "CGALMeshShell.h"
#include "../utilities/geogebra.h"
#include "../utilities/enclosing_sphere.h"

using namespace mgodpl::cgal_utils;

std::shared_ptr<ShellPath<CGALMeshPointAndNormal>> CGALMeshShell::path_from_to(const CGALMeshPointAndNormal &a, const CGALMeshPointAndNormal &b) const {

	// Initialize the Surface_mesh_shortest_path object with the surface mesh
	Surface_mesh_shortest_path shortest_paths(tmesh);
	// Add the start point a as a source point for the shortest path computation
	shortest_paths.add_source_point(a.point);

	// Compute the shortest path from point a to point b
	PathVisitor v(tmesh);
	shortest_paths.shortest_path_sequence_to_source_points(b.point.first, b.point.second, v);

	// The computed path is from the end point to the start point, so reverse it to get the path from the start point to the end point
	std::reverse(v.states.begin(), v.states.end());

	std::vector<CGALMeshPointAndNormal> points;

	// Add the start point to the path
	points.push_back(a);

	// Add the intermediate points to the path
	for (auto &state : v.states) {
		points.push_back({state, faceNormal(tmesh, state.first)});
	}

	// Add the end point to the path
	points.push_back(b);

	// Return the computed path as a PiecewiseLinearPath object
	return std::make_shared<PiecewiseLinearPath<CGALMeshPointAndNormal>>(std::move(points));

}

Triangle_mesh from_rosmsg(const shape_msgs::msg::Mesh &mesh) {

	Triangle_mesh tmesh;

	// We first convert every vertex to a CGAL point, building a vector to translate between the original and CGAL point indices.
	std::vector<Triangle_mesh::vertex_index> vertices;

	for (auto &v: mesh.vertices) {
		vertices.push_back(tmesh.add_vertex(Kernel::Point_3(v.x, v.y, v.z)));
	}

	// Now we add every triangle to the mesh.
	for (auto &f: mesh.triangles) {
		Triangle_mesh::face_index fi = tmesh.add_face(vertices[f.vertex_indices[0]],
													  vertices[f.vertex_indices[1]],
													  vertices[f.vertex_indices[2]]);
		assert(fi != Triangle_mesh::null_face());
	}

	return tmesh;
}

CGALMeshShell::CGALMeshShell(const shape_msgs::msg::Mesh &mesh, double rotationWeight, double padding)
		: rotation_weight(rotationWeight), padding(padding),tmesh(from_rosmsg(mesh)) {

	// We initialize the AABB tree such that we don't have to re-compute it every projection query.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(tree);
}

CGALMeshShell::CGALMeshShell(Triangle_mesh mesh, double rotationWeight, double padding)
		: rotation_weight(rotationWeight), padding(padding),tmesh(mesh) {

	// We initialize the AABB tree such that we don't have to re-compute it every projection query.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(tree);

}

CGALMeshPointAndNormal CGALMeshShell::nearest_point_on_shell(const Eigen::Vector3d &pt) const {
	return locate(tmesh, {pt.x(), pt.y(), pt.z()}, tree);
}

double CGALMeshShell::path_length(const std::vector<CGALMeshPointAndNormal> &path) const {

	double length = 0.0;

	for (size_t i = 0; i < path.size() - 1; i++) {
		length += (surface_point(path[i]) - surface_point(path[i + 1])).norm();
		length += rotation_weight * (arm_vector(path[i]) - arm_vector(path[i + 1])).norm();
	}

	return length;
}

double CGALMeshShell::path_length(const std::shared_ptr<ShellPath<CGALMeshPointAndNormal>> &path) const {

	auto p = std::dynamic_pointer_cast<PiecewiseLinearPath<CGALMeshPointAndNormal>>(path);

	assert(p);

	return path_length(p->points);
}

Eigen::Vector3d CGALMeshShell::arm_vector(const CGALMeshPointAndNormal &p) const {
	return -p.normal;
}

std::vector<std::vector<double>> CGALMeshShell::distance_matrix(const std::vector<CGALMeshPointAndNormal> &points) const {

	mgodpl::cgal_utils::WeightedMesh mesh {
		tmesh,
		rotation_weight
	};

	return mgodpl::metric_space::point_distance_all_to_all<mgodpl::cgal_utils::WeightedMesh>(mesh, points);

}

Eigen::Vector3d CGALMeshShell::surface_point(const CGALMeshPointAndNormal &p) const {
	return toEigen(mgodpl::cgal_utils::to_carthesian(tmesh, p.point));
}

std::shared_ptr<WorkspaceShell<CGALMeshPointAndNormal>>
convexHullAroundLeavesCGAL(const AppleTreePlanningScene &scene_info, double rotation_weight, double padding) {

	return std::make_shared<CGALMeshShell>(convexHull(utilities::extract_leaf_vertices(scene_info)),
										   rotation_weight,
										   padding);
}
