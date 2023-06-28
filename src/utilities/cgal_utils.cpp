// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 28-6-23.
//

#include "cgal_utils.h"

Eigen::Vector3d mgodpl::cgal_utils::toEigen(const Kernel::Point_3 &p) {
	return  { p.x(), p.y(), p.z() };
}

void
mgodpl::cgal_utils::PathVisitor::operator()(CGAL::Surface_mesh_shortest_path<mgodpl::cgal_utils::Traits>::face_descriptor f,
											std::array<double, 3> location) {
	// Add a Robot state...
	states.emplace_back(f,location);
}

void mgodpl::cgal_utils::PathVisitor::operator()(CGAL::Surface_mesh_shortest_path<Traits>::vertex_descriptor vertex) {
	states.push_back(path_algo.face_location(vertex));
}

void mgodpl::cgal_utils::PathVisitor::operator()(CGAL::Surface_mesh_shortest_path<Traits>::halfedge_descriptor edge,
												 double t) {
	// Compute the intersection position.
	Eigen::Vector3d pos = toEigen(path_algo.point(edge, t));

	states.push_back(path_algo.face_location(edge, t));
	states.push_back(
			path_algo.face_location(mesh.opposite(edge), 1.0-t)
	);
}

mgodpl::cgal_utils::PathVisitor::PathVisitor(const mgodpl::cgal_utils::Triangle_mesh &mesh) : mesh(mesh), path_algo(mesh) {
}

Eigen::Vector3d mgodpl::cgal_utils::faceNormal(const mgodpl::cgal_utils::Triangle_mesh &tmesh,
											   const CGAL::Surface_mesh_shortest_path<mgodpl::cgal_utils::Traits>::face_descriptor &face) {
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

/**
 * Locate the nearest point on a triangle mesh to a given point.
 */
mgodpl::cgal_utils::CGALMeshPointAndNormal mgodpl::cgal_utils::locate(const Triangle_mesh& tmesh, const Kernel::Point_3& pt, const LocationTree& tree) {
	// initialize the algorithm struct.
	Surface_mesh_shortest_path shortest_paths(tmesh);

	auto on_mesh = shortest_paths.locate(pt, tree);

	auto pt_on_mesh = toEigen(shortest_paths.point(on_mesh.first, on_mesh.second));

	Eigen::Vector3d offset = toEigen(pt) - pt_on_mesh;

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

		return {on_mesh, normal};

	}
}

mgodpl::cgal_utils::Kernel::Point_3 mgodpl::cgal_utils::to_carthesian(const Triangle_mesh &tmesh, const CGALMeshPoint &p) {

	Surface_mesh_shortest_path path_algo(tmesh);

	return path_algo.point(p.first, p.second);

}

double mgodpl::cgal_utils::path_length(const mgodpl::cgal_utils::WeightedMesh &mesh,
									   const std::vector<CGALMeshPointAndNormal> &path) {

	double length = 0.0;

	for (size_t i = 0; i < path.size() - 1; i++) {

		Kernel::Point_3 p1 = to_carthesian(mesh.mesh, path[i].point);
		Kernel::Point_3 p2 = to_carthesian(mesh.mesh, path[i + 1].point);

		length += std::sqrt(CGAL::squared_distance(p1, p2));
		length += std::acos(std::clamp(path[i].normal.dot(path[i + 1].normal), -1.0, 1.0)) * mesh.rotation_weight;
	}

	return length;
}


template<>
std::vector<std::vector<double>> mgodpl::metric_space::point_distance_all_to_all(const cgal_utils::WeightedMesh &context,
																				 const std::vector<cgal_utils::CGALMeshPointAndNormal> &points) {

	using namespace cgal_utils;

	// Initialize a 2D vector to store the distances between each pair of points
	std::vector<std::vector<double>> distances(points.size(), std::vector<double>(points.size(), 0.0));

	// Iterate over each point in the input vector
	for (size_t i = 0; i < points.size(); i++) {

		// Initialize the Surface_mesh_shortest_path object with the surface mesh
		Surface_mesh_shortest_path shortest_paths(context.mesh);

		// Add the current point as a source point for the shortest path computation
		shortest_paths.add_source_point(points[i].point.first, points[i].point.second);

		// For each other point in the vector...
		for (size_t j = i + 1; j < points.size(); j++) {

			PathVisitor v(context.mesh);
			shortest_paths.shortest_path_sequence_to_source_points(points[j].point.first,
																   points[j].point.second,
																   v);

			// The computed path is from the end point to the start point, so reverse it to get the path from the start point to the end point
			std::reverse(v.states.begin(), v.states.end());

			std::vector<CGALMeshPointAndNormal> geodesic;

			// Add the start point to the path
			geodesic.push_back(points[i]);

			// Add the intermediate points to the path
			for (auto &state: v.states) {
				geodesic.push_back({state, faceNormal(context.mesh, state.first)});
			}

			// Add the end point to the path
			geodesic.push_back(points[j]);

			// Return the computed path as a PiecewiseLinearPath object
			double length = path_length(context, geodesic);

			// Store the computed distance in the distance matrix
			// The distance from point i to point j is the same as the distance from point j to point i
			distances[i][j] = length;
			distances[j][i] = length;
		}
	}

	// Return the computed distance matrix
	return distances;

}

