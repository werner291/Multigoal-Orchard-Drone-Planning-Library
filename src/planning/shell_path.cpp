// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/22/24.
//

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include "shell_path.h"
#include "cgal_chull_shortest_paths.h"
#include "state_tools.h"

using namespace mgodpl::cgal;

mgodpl::RobotPath mgodpl::shell_path(const CGAL::Surface_mesh_shortest_path<mgodpl::cgal::Traits>::Face_location &from,
									 const CGAL::Surface_mesh_shortest_path<mgodpl::cgal::Traits>::Face_location &to,
									 const mgodpl::cgal::Surface_mesh &mesh,
									 const robot_model::RobotModel &robot) {

	Surface_mesh_shortest_path mesh_path(mesh);

	// Swapping the "to" and the "from" points here, because the algorithm backtracks to the source point.
	mesh_path.add_source_point(to.first, to.second);

	std::vector<Surface_mesh_shortest_path::Face_location> path;

	PathVisitor path_visitor{.mesh = mesh, .path_algo = mesh_path, .states = path};

	mesh_path.shortest_path_sequence_to_source_points(from.first, from.second, path_visitor);

	RobotPath shell_path;

	for (const auto &[face, barycentric]: path) {
		auto pt = mesh_path.point(face, barycentric);
		auto face_normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, mesh);
		auto robot_state = fromEndEffectorAndVector(robot,
													{pt.x(), pt.y(), pt.z()},
													{face_normal.x(), face_normal.y(), face_normal.z()});
		shell_path.states.push_back(robot_state);
	}

	return shell_path;

}

std::vector<double> mgodpl::shell_distances(const cgal::Surface_mesh_shortest_path::Face_location &from,
											const std::vector<ApproachPath> &paths,
											const Surface_mesh &mesh) {
	Surface_mesh_shortest_path mesh_path(mesh);

	mesh_path.add_source_point(from.first, from.second);

	std::vector<double> distances;
	distances.reserve(paths.size());

	for (const auto &path: paths) {
		const auto &[face, barycentric] = path.shell_point;
		distances.push_back(mesh_path.shortest_distance_to_source_points(face, barycentric).first);
	}

	return distances;
}

std::vector<std::vector<double>> mgodpl::shell_distances(const std::vector<ApproachPath>& approach_paths,
	const cgal::CgalMeshData& mesh_data)
{
	std::vector<std::vector<double>> target_to_target_distances;
	target_to_target_distances.reserve(approach_paths.size());
	for (const ApproachPath& path : approach_paths) {
		target_to_target_distances.emplace_back(shell_distances(path.shell_point,
		                                                        approach_paths,
		                                                        mesh_data.convex_hull));
	}
	return target_to_target_distances;
}
