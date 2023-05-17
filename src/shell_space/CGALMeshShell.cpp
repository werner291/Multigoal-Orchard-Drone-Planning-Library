
#include "CGALMeshShell.h"
#include "../utilities/experiment_utils.h"
#include "../utilities/convex_hull.h"
#include "../utilities/geogebra.h"
#include "../utilities/enclosing_sphere.h"

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

Eigen::Vector3d toEigen(const Kernel::Point_3 &p) {
	return  { p.x(), p.y(), p.z() };
}

/**
 * A visitor for Surface_mesh_shortest_path::shortest_path_sequence_to_source_points that builds a robot
 * path along the generated surface path.
 */
struct PathVisitor {

	/// Reference to the triangle mesh
	const Triangle_mesh& mesh;

	/// The path being built.
	std::vector<CGALMeshPoint> states;

	/// Shortest path algorithm struct (for point lookups and such)
	Surface_mesh_shortest_path path_algo;

	/**
	 * Constructor.
	 * @param mesh 		The triangle mesh.
	 * @param drone 	The robot model.
	 */
	explicit PathVisitor(const Triangle_mesh &mesh)
			: mesh(mesh), path_algo(mesh) {
	}

	/**
	 * Called when the path leaves a face through a half-edge.
	 * @param edge 		The half-edge for leaving the face (NOT the opposite half-edge where we enter the face!)
	 * @param t 		Interpolation value between the two vertices of the half-edge where the intersection occurs.
	 */
	void operator()(Surface_mesh_shortest_path::halfedge_descriptor edge, Surface_mesh_shortest_path::FT t)
	{
		// Compute the intersection position.
		Eigen::Vector3d pos = toEigen(path_algo.point(edge, t));

		states.push_back(path_algo.face_location(edge, t));
		// TODO restore
		//		states.push_back(
		//				path_algo.face_location(mesh.opposite(edge), 1.0-t)
		//		);
	}

	/**
	 * The path *exactly* crosses an edge.
	 *
	 * Note: we use the mysterious "Surface_mesh_shortest_path::face_location" function;
	 * I'm not sure how it chooses which face it's on, but it seems to work?
	 *
	 * @param vertex 	The vertex of the edge where the path crosses.
	 */
	void operator()(Surface_mesh_shortest_path::vertex_descriptor vertex)
	{
		states.push_back(path_algo.face_location(vertex));
	}

	/**
	 * The path includes a point on the interior of a face. Note that, since a geodesic is a straight line on planar surfaces,
	 * this case will only occur at the start of the path. Nevertheless, nothing should break if this *does* occur for some reason.
	 *
	 * @param f 			The face where the path has a point.
	 * @param location 		The location of the point on the face (in barycentric coordinates)
	 */
	void operator()(Surface_mesh_shortest_path::face_descriptor f, Surface_mesh_shortest_path::Barycentric_coordinates location)
	{
		// Add a Robot state...
		states.emplace_back(f,location);
	}

};

Eigen::Vector3d CGALMeshShell::surface_point(const CGALMeshShellPoint &pt) const {
	Surface_mesh_shortest_path shortest_paths(tmesh);
	return toEigen(shortest_paths.point(pt.point.first, pt.point.second)) + normalAt(pt) * padding;
}

Eigen::Vector3d CGALMeshShell::normalAt(const CGALMeshShellPoint &near) const {
	return near.normal;
}

std::shared_ptr<ShellPath<CGALMeshShellPoint>> CGALMeshShell::path_from_to(const CGALMeshShellPoint &a, const CGALMeshShellPoint &b) const {

	// Initialize the Surface_mesh_shortest_path object with the surface mesh
	Surface_mesh_shortest_path shortest_paths(tmesh);
	// Add the start point a as a source point for the shortest path computation
	shortest_paths.add_source_point(a.point);

	// Compute the shortest path from point a to point b
	PathVisitor v(tmesh);
	shortest_paths.shortest_path_sequence_to_source_points(b.point.first, b.point.second, v);

	// The computed path is from the end point to the start point, so reverse it to get the path from the start point to the end point
	std::reverse(v.states.begin(), v.states.end());

	std::vector<CGALMeshShellPoint> points;

	// Add the start point to the path
	points.push_back(a);

	// Add the intermediate points to the path
	for (auto &state : v.states) {
		points.push_back({state, faceNormal(tmesh, state.first)});
	}

	// Add the end point to the path
	points.push_back(b);

	// Return the computed path as a PiecewiseLinearPath object
	return std::make_shared<PiecewiseLinearPath<CGALMeshShellPoint>>(std::move(points));

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

	nearest_point_on_shell(
			Eigen::Vector3d(42.0, 42.0, 57.0)
			);

}

CGALMeshShell::CGALMeshShell(Triangle_mesh mesh, double rotationWeight, double padding)
		: rotation_weight(rotationWeight), padding(padding),tmesh(mesh) {

	// We initialize the AABB tree such that we don't have to re-compute it every projection query.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(tree);

}

CGALMeshShellPoint CGALMeshShell::nearest_point_on_shell(const Eigen::Vector3d &pt) const {

	// initialize the algorithm struct.
	Surface_mesh_shortest_path shortest_paths(tmesh);

	auto on_mesh = shortest_paths.locate(Kernel::Point_3(pt.x(), pt.y(), pt.z()), tree);

	auto pt_on_mesh = toEigen(shortest_paths.point(on_mesh.first, on_mesh.second));

	auto offset = pt - pt_on_mesh;

	auto normal = faceNormal(tmesh, on_mesh.first);

	// CHeck if any of the barycentric coordinates are 0 or 1. If so, we're on an edge or vertex.
	if (on_mesh.second[0] < 1.0e-6 || on_mesh.second[0] > 1.0 - 1.0e-6 ||
		on_mesh.second[1] < 1.0e-6 || on_mesh.second[1] > 1.0 - 1.0e-6 ||
		on_mesh.second[2] < 1.0e-6 || on_mesh.second[2] > 1.0 - 1.0e-6) {

		return {
			on_mesh,
			offset.normalized()
		};

	} else {

		return {
			on_mesh,
			normal
		};

	}
}

double CGALMeshShell::path_length(const std::shared_ptr<ShellPath<CGALMeshShellPoint>> &path) const {

	auto p = std::dynamic_pointer_cast<PiecewiseLinearPath<CGALMeshShellPoint>>(path);

	assert(p);

	double length = 0.0;

	for (size_t i = 0; i < p->points.size() - 1; i++) {
		length += (surface_point(p->points[i]) - surface_point(p->points[i + 1])).norm();
		length += rotation_weight * (arm_vector(p->points[i]) - arm_vector(p->points[i + 1])).norm();
	}

	return length;
}

Eigen::Vector3d CGALMeshShell::arm_vector(const CGALMeshShellPoint &p) const {
	return -normalAt(p);
}

std::vector<std::vector<double>> CGALMeshShell::distance_matrix(const std::vector<CGALMeshShellPoint> &points) const {

	// Initialize the Surface_mesh_shortest_path object with the surface mesh
	Surface_mesh_shortest_path shortest_paths(tmesh);

	// Initialize a 2D vector to store the distances between each pair of points
	std::vector<std::vector<double>> distances(points.size(), std::vector<double>(points.size(), 0.0));

	// Iterate over each point in the input vector
	for (size_t i = 0; i < points.size(); i++) {

		// Add the current point as a source point for the shortest path computation
		shortest_paths.add_source_point(points[i].point.first, points[i].point.second);

		// For each other point in the vector...
		for (size_t j = i + 1; j < points.size(); j++) {

			// Compute the shortest distance from the current source point to this point
			auto result = shortest_paths.shortest_distance_to_source_points(points[j].point.first,
																			points[j].point.second);

			// Store the computed distance in the distance matrix
			// The distance from point i to point j is the same as the distance from point j to point i
			distances[i][j] = result.first;
			distances[j][i] = result.first;
		}
	}

	// Return the computed distance matrix
	return distances;
}

std::shared_ptr<WorkspaceShell<CGALMeshShellPoint>>
convexHullAroundLeavesCGAL(const AppleTreePlanningScene &scene_info, double rotation_weight, double padding) {

	return std::make_shared<CGALMeshShell>(convexHull(utilities::extract_leaf_vertices(scene_info)),
										   rotation_weight,
										   padding);
}
