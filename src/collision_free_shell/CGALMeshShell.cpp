//
// Created by werner on 12-8-22.
//

#include "CGALMeshShell.h"
#include "../utilities/experiment_utils.h"
#include "../utilities/convex_hull.h"
#include "../utilities/geogebra.h"

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

std::vector<moveit::core::RobotState> CGALMeshShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																   const CGALMeshPoint &a,
																   const CGALMeshPoint &b) const {

	// Initialize the shortest path algorithm with a reference to the triangle mesh.
	Surface_mesh_shortest_path shortest_paths(tmesh);
	// We add a "source" point a.
	shortest_paths.add_source_point(a);

	// Compute the path from a to b.
	std::vector<Kernel::Point_3> path;
	shortest_paths.shortest_path_points_to_source_points(b.first, b.second, std::back_inserter(path));
	// Reverse the path, since the path, as computed, is from b to a.
	std::reverse(path.begin(), path.end());

	// Initialize a path containing just an initial state.
	std::vector<moveit::core::RobotState> states{
			state_on_shell(drone, a) // Initial state is the state at the given start point.
	};

	// We'll keep track of the "last" normal and carthesian coordinates,
	// so that we can insert states as-needed as these change.
	// We do it this way because it is much easier to deal with than
	// making a special case for vertex crossings.
	Eigen::Vector3d normal = normalAt(a);
	Eigen::Vector3d pos = toCarthesian(a);

	assert((states[0].getGlobalLinkTransform("end_effector").translation() - pos).norm() < 1e-6);

	for (auto &i: path) {

		// Look up the coordinates of the new point and convert to Eigen.
		Eigen::Vector3d new_pos(i.x(), i.y(), i.z());

		// Translation vector to get from the last point to the current.
		Eigen::Vector3d delta = new_pos - pos;

		if ((pos - new_pos).norm() < 1.0e-10) {
			// The new point is the same as the last point, so we don't need to add a state.
			// Note that we don't care about the normal vectors, as those are computes as a function
			// of straight-line sections of the path.

			continue;
		} else {
			// The new point is different from the last point, so we need to add at least one state.

			// Check if the current (last) normal vector is still perpendicular to the path.
			if (abs(normal.dot(delta)) > 1.0e-10) {
				// If not, we must add a state without movement but with a
				// new normal to prevent the robot from crossing into the shell when interpolating.
				//
				// Effectively, we want the robot to "rotate" around the end-effector, keeping the latter in one place.
				// TODO: Might need more than one state, now that I think about it...

				// Compute a vector perpendicular to the old normal vector and the delta vector.
				// This will put any new normal vector into the same plane as the old one. (TODO Should I want that?)
				Eigen::Vector3d corner_normal = delta.cross(normal);

				// Then, compute a vector perpendicular to the corner normal and the delta vector.
				// This will make the normal perpendicular to the new path section.
				Eigen::Vector3d new_normal = delta.cross(corner_normal).normalized();

				// If the vector faces inward, flip it. (TODO: is this math sufficiently robust?)
				if (new_normal.dot(normal) < 0.0) {
					new_normal = -new_normal;
				}

				// Record the new normal vector.
				normal = new_normal;

				// Add a state with the new normal vector, but at the old position.
				states.push_back({robotStateFromFacing(drone, pos + normal * padding, -normal)});
			}

			// Advance the carthesian coordinates.
			pos = new_pos;

			// Add a new state with assumed identical normal to the previous one.
			states.push_back({robotStateFromFacing(drone, pos + normal * padding, -normal)});

		}

	}

	// Add a final state.
	states.push_back(state_on_shell(drone, b));

	assert((states[states.size()-1].getGlobalLinkTransform("end_effector").translation() - pos).norm() < 1e-6);
	assert((states[states.size()-2].getGlobalLinkTransform("end_effector").translation() - pos).norm() < 1e-6);

	return states;
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

	// Look up the vertices (same way as in toCarthesian).
	auto vertices = tmesh.vertices_around_face(tmesh.halfedge(near.first));

	assert(vertices.size() == 3);

	auto itr = vertices.begin();

	std::array<Kernel::Point_3, 3> points{tmesh.point(*(itr++)), tmesh.point(*(itr++)), tmesh.point(*(itr++)),};

	Eigen::Vector3d va(points[0].x(), points[0].y(), points[0].z());
	Eigen::Vector3d vb(points[1].x(), points[1].y(), points[1].z());
	Eigen::Vector3d vc(points[2].x(), points[2].y(), points[2].z());

	// Compute the normalized cross product.
	// This one inexplicably points inward, so we flip it again. Does the CGAL code flip our vertices somewhere?
	return -(vb - va).cross(vc - va).normalized();
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
