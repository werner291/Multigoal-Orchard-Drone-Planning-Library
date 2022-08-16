//
// Created by werner on 12-8-22.
//

#include "CGALMeshShell.h"
#include "../utilities/experiment_utils.h"
#include "../utilities/convex_hull.h"
#include "../utilities/geogebra.h"

Eigen::Vector3d faceNormal(const Triangle_mesh& tmesh, const Surface_mesh_shortest_path::face_descriptor &face) {
	// Look up the vertices (same way as in toCarthesian).
	auto vertices = tmesh.vertices_around_face(tmesh.halfedge(face));

	assert(vertices.size() == 3);

	auto itr = vertices.begin();

	std::array<Kernel::Point_3, 3> points{
		tmesh.point(*(itr++)),
		tmesh.point(*(itr++)),
		tmesh.point(*(itr++))
	};

	Eigen::Vector3d va(points[0].x(), points[0].y(), points[0].z());
	Eigen::Vector3d vb(points[1].x(), points[1].y(), points[1].z());
	Eigen::Vector3d vc(points[2].x(), points[2].y(), points[2].z());

	// Compute the normalized cross product.
	// This one inexplicably points inward, so we flip it again. Does the CGAL code flip our vertices somewhere?
	return -(vb - va).cross(vc - va).normalized();
}

Eigen::Vector3d normalAt(const Triangle_mesh& tmesh, const CGALMeshPoint &near) {
	return faceNormal(tmesh, near.first);
}

moveit::core::RobotState
CGALMeshShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const CGALMeshPoint &a) const {

	// Compute the normal vector and carthesian coordinates of the point.
	const Eigen::Vector3d normal = normalAt(a);
	const Eigen::Vector3d pos = toCarthesian(a);

	// ANd simp[ly return a robot with the end-eddector at that position (+ padding) and facing into the normal.
	return robotStateFromFacing(drone,
			// Translate the point by the padding times the normal.
								pos + normal * padding,
			// Facing inward, so opposite the normal.
								-normal);
}

Eigen::Vector3d toEigen(const Kernel::Point_3 &p) {
	return  { p.x(), p.y(), p.z() };
}

struct PathVisitor {

	const Triangle_mesh& mesh;
	const moveit::core::RobotModelConstPtr& drone;
	std::vector<moveit::core::RobotState> states;
	Surface_mesh_shortest_path path_algo;

	explicit PathVisitor(const Triangle_mesh &mesh, const moveit::core::RobotModelConstPtr &drone)
			: mesh(mesh), drone(drone), path_algo(mesh) {
	}

	void operator()(Surface_mesh_shortest_path::halfedge_descriptor edge, Surface_mesh_shortest_path::FT t)
	{
		Eigen::Vector3d pos = toEigen(path_algo.point(edge, t));

		states.push_back(robotStateFromFacing(
				drone,
				pos,
				-faceNormal(mesh, mesh.face(edge)))
		);

		states.push_back(robotStateFromFacing(
				drone,
				pos,
				-faceNormal(mesh, mesh.face(mesh.opposite(edge))))
		);
	}

	void operator()(Surface_mesh_shortest_path::vertex_descriptor vertex)
	{
		throw std::runtime_error("Not implemented");
	}

	void operator()(Surface_mesh_shortest_path::face_descriptor f, Surface_mesh_shortest_path::Barycentric_coordinates location)
	{
		states.push_back(robotStateFromFacing(
				drone,
				toEigen(path_algo.point(f, location)),
				-faceNormal(mesh, f)
		));
	}

};

std::vector<moveit::core::RobotState> CGALMeshShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																   const CGALMeshPoint &a,
																   const CGALMeshPoint &b) const {

	// Initialize the shortest path algorithm with a reference to the triangle mesh.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	// We add a "source" point a.
	shortest_paths.add_source_point(a);

	PathVisitor visitor(tmesh, drone);
	shortest_paths.shortest_path_sequence_to_source_points(b.first, b.second, visitor);

	return std::move(visitor.states);
}

CGALMeshPoint CGALMeshShell::gaussian_sample_near_point(const CGALMeshPoint &near) const {

	ompl::RNG rng;

	// This method simply generates a random offset from a gaussian sample, then...
	Eigen::Vector3d offset(rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1));

	// ...adds it to the given point (after converting to carthesian coordinates), and projects back to a CGALMeshPoint.
	return project(toCarthesian(near) + offset);

}

Eigen::Vector3d CGALMeshShell::toCarthesian(const CGALMeshPoint &pt) const {
	Surface_mesh_shortest_path shortest_paths(tmesh);
	return toEigen(shortest_paths.point(pt.first, pt.second));
}

Eigen::Vector3d CGALMeshShell::normalAt(const CGALMeshPoint &near) const {
	return ::normalAt(tmesh, near);
}

double CGALMeshShell::predict_path_length(const CGALMeshPoint &a, const CGALMeshPoint &b) const {

	// Initialize the shortest path algorithm with a reference to the triangle mesh.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	// Add pur start point a
	shortest_paths.add_source_point(a);

	// Compute the path to a from b. This technically computes the reversed path, but this should not make a difference for the path length.
	std::vector<Kernel::Point_3> path;
	shortest_paths.shortest_path_points_to_source_points(b.first, b.second, std::back_inserter(path));

	// Add up the distances between the points of the path.
	double total_length = 0.0;

	for (size_t i = 0; i + 1 < path.size(); ++i) {
		double d = sqrt((path[i] - path[i + 1]).squared_length());
		total_length += d;
	}

	// Add up the turning angles at every point of the path.
	double total_rotation = 0.0;

	for (size_t i = 0; i + 2 < path.size(); ++i) {

		double da = angle((path[i + 1] - path[i]), path[i + 2] - path[i + 1]);

		total_rotation += da;
	}

	// Total prediction is the sum of the two, with the rotation weighed parametrically.
	return total_length + total_rotation * rotation_weight;

}

CGALMeshPoint CGALMeshShell::project(const moveit::core::RobotState &st) const {
	return project(st.getGlobalLinkTransform("end_effector").translation());
}

CGALMeshPoint CGALMeshShell::project(const Apple &st) const {
	return project(st.center);
}

CGALMeshShell::CGALMeshShell(const shape_msgs::msg::Mesh &mesh, double rotationWeight, double padding)
		: rotation_weight(rotationWeight), padding(padding) {

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

	// We initialize the AABB tree such that we don't have to re-compute it every projection query.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(tree);

}

CGALMeshPoint CGALMeshShell::project(const Eigen::Vector3d &pt) const {

	// initialize the algorithm struct.
	Surface_mesh_shortest_path shortest_paths(tmesh);

	// Look up the nearest point, using the AABB tree to accelerate the search.
	return shortest_paths.locate(Kernel::Point_3(pt.x(), pt.y(), pt.z()), tree);
}

std::shared_ptr<OMPLSphereShellWrapper<CGALMeshPoint>>
CGALConvexHullShellBuilder::buildShell(const AppleTreePlanningScene &scene_info,
									   const ompl::base::SpaceInformationPtr &si) const {
	return std::make_shared<OMPLSphereShellWrapper<CGALMeshPoint>>(std::make_shared<CGALMeshShell>(convexHull(
			extract_leaf_vertices(scene_info)), rotation_weight, padding), si);
}

Json::Value CGALConvexHullShellBuilder::parameters() const {
	Json::Value params;

	params["type"] = "convex_hull_cgal";
	params["rotation_weight"] = rotation_weight;
	params["padding"] = padding;

	return params;
}

CGALConvexHullShellBuilder::CGALConvexHullShellBuilder(double padding, double rotationWeight)
		: padding(padding), rotation_weight(rotationWeight) {
}
