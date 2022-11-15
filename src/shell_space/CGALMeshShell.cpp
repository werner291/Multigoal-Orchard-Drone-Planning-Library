
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

		states.push_back(
				path_algo.face_location(edge, t)
		);

		states.push_back(
				path_algo.face_location(mesh.opposite(edge), 1.0-t)
		);
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

Eigen::Vector3d CGALMeshShell::surface_point(const CGALMeshPoint &pt) const {
	Surface_mesh_shortest_path shortest_paths(tmesh);
	return toEigen(shortest_paths.point(pt.first, pt.second)) + normalAt(pt) * padding;
}

Eigen::Vector3d CGALMeshShell::normalAt(const CGALMeshPoint &near) const {
	return ::normalAt(tmesh, near);
}

std::shared_ptr<ShellPath<CGALMeshPoint>> CGALMeshShell::path_from_to(const CGALMeshPoint &a, const CGALMeshPoint &b) const {

	// Initialize the shortest path algorithm with a reference to the triangle mesh.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	// Add pur start point a
	shortest_paths.add_source_point(a);

	// Compute the path to a from b.
	PathVisitor v(tmesh);
	shortest_paths.shortest_path_sequence_to_source_points(b.first, b.second, v);

	assert(v.states.size() > 0);

	// CGAL gives us the path from the end to the start, so we reverse it.
	std::reverse(v.states.begin(), v.states.end());

	return std::make_shared<PiecewiseLinearPath<CGALMeshPoint>>(std::move(v.states));

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

CGALMeshPoint CGALMeshShell::nearest_point_on_shell(const Eigen::Vector3d &pt) const {

	// initialize the algorithm struct.
	Surface_mesh_shortest_path shortest_paths(tmesh);

	// Look up the nearest point, using the AABB tree to accelerate the search.
	return shortest_paths.locate(Kernel::Point_3(pt.x(), pt.y(), pt.z()), tree);
}

double CGALMeshShell::path_length(const std::shared_ptr<ShellPath<CGALMeshPoint>> &path) const {

	auto p = std::dynamic_pointer_cast<PiecewiseLinearPath<CGALMeshPoint>>(path);

	assert(p);

	double length = 0.0;

	for (size_t i = 0; i < p->points.size() - 1; i++) {
		length += (surface_point(p->points[i]) - surface_point(p->points[i + 1])).norm();
		length += rotation_weight * (arm_vector(p->points[i]) - arm_vector(p->points[i + 1])).norm();
	}

	return length;
}

Eigen::Vector3d CGALMeshShell::arm_vector(const CGALMeshPoint &p) const {
	return -normalAt(p);
}

std::shared_ptr<WorkspaceShell<CGALMeshPoint>> convexHullAroundLeavesCGAL(const AppleTreePlanningScene &scene_info,
																		  const ompl::base::SpaceInformationPtr &si,
																		  double rotation_weight,
																		  double padding) {

	return std::make_shared<CGALMeshShell>(convexHull(extract_leaf_vertices(scene_info)), rotation_weight, padding);
}
