//
// Created by werner on 12-8-22.
//

#include "CGALMeshShell.h"

moveit::core::RobotState
CGALMeshShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const CGALMeshPoint &a) const {

	const Eigen::Vector3d normal = normalAt(a);
	const Eigen::Vector3d pos = toCarthesian(a);

	return robotStateFromFacing(drone,
			// Translate the point by the padding times the normal.
								pos + normal * padding,
			// Facing inward, so opposite the normal.
								-normal);
}

std::vector<moveit::core::RobotState> CGALMeshShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone,
																   const CGALMeshPoint &a,
																   const CGALMeshPoint &b) const {

	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.add_source_point(a);
	std::vector<Kernel::Point_3> path;
	shortest_paths.shortest_path_points_to_source_points(b.first, b.second, std::back_inserter(path));

	std::vector<moveit::core::RobotState> states;

	Eigen::Vector3d normal = normalAt(a);
	Eigen::Vector3d pos = toCarthesian(a);

	states.push_back(state_on_shell(drone,a));

	for (auto & i : path) {

		Eigen::Vector3d new_pos(i.x(), i.y(), i.z());
		Eigen::Vector3d delta = new_pos - pos;

		if ((pos - new_pos).norm() < 1.0e-10) {
			continue;
		} else {

			if (abs(normal.dot(delta)) > 1.0e-10) {
				normal = delta.cross(normal);
				normal = delta.cross(normal).normalized();

				states.push_back({ robotStateFromFacing(drone, pos + normal * padding,-normal)});
			}

			pos = new_pos;

			states.push_back({ robotStateFromFacing(drone, pos + normal * padding,-normal)});

		}

	}

	states.push_back(state_on_shell(drone, b));

	return states;
}

CGALMeshPoint CGALMeshShell::gaussian_sample_near_point(const CGALMeshPoint &near) const {

	Eigen::Vector3d euclidean = toCarthesian(near);

	ompl::RNG rng;

	Eigen::Vector3d offset(rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1), rng.gaussian(0.0, 0.1));

	return project(euclidean + offset);

}

Eigen::Vector3d CGALMeshShell::toCarthesian(const CGALMeshPoint &near) const {
	auto vertices = tmesh.vertices_around_face(tmesh.halfedge(near.first));

	assert(vertices.size() == 3);

	auto itr = vertices.begin();

	std::array<Kernel::Point_3 , 3> points {
			tmesh.point(*(itr++)),
			tmesh.point(*(itr++)),
			tmesh.point(*(itr++)),
	};

	Eigen::Vector3d va(points[0].x(), points[0].y(), points[0].z());
	Eigen::Vector3d vb(points[1].x(), points[1].y(), points[1].z());
	Eigen::Vector3d vc(points[2].x(), points[2].y(), points[2].z());

	Eigen::Vector3d euclidean = near.second[0] * va + near.second[1] * vb + near.second[2] * vc;
	return euclidean;
}

Eigen::Vector3d CGALMeshShell::normalAt(const CGALMeshPoint &near) const {
	auto vertices = tmesh.vertices_around_face(tmesh.halfedge(near.first));

	assert(vertices.size() == 3);

	auto itr = vertices.begin();

	std::array<Kernel::Point_3 , 3> points {
			tmesh.point(*(itr++)),
			tmesh.point(*(itr++)),
			tmesh.point(*(itr++)),
	};

	Eigen::Vector3d va(points[0].x(), points[0].y(), points[0].z());
	Eigen::Vector3d vb(points[1].x(), points[1].y(), points[1].z());
	Eigen::Vector3d vc(points[2].x(), points[2].y(), points[2].z());

	// TODO: Verify that this is pointing outward.
	return (vb - va).cross(vc - va);
}

double CGALMeshShell::predict_path_length(const CGALMeshPoint &a, const CGALMeshPoint &b) const {

	Surface_mesh_shortest_path shortest_paths(tmesh);

	shortest_paths.add_source_point(a);

	return shortest_paths.shortest_distance_to_source_points(b.first, b.second).first;

}

CGALMeshPoint CGALMeshShell::project(const moveit::core::RobotState &st) const {
	return project(st.getGlobalLinkTransform("end_effector").translation());
}

CGALMeshPoint CGALMeshShell::project(const Apple &st) const {
	return project(st.center);
}

CGALMeshShell::CGALMeshShell(const shape_msgs::msg::Mesh &mesh) {

	std::vector<Triangle_mesh::vertex_index> vertices;

	for (auto &v : mesh.vertices) {
		vertices.push_back(tmesh.add_vertex(Kernel::Point_3(v.x, v.y, v.z)));
	}

	for (auto &f : mesh.triangles) {
		tmesh.add_face(vertices[f.vertex_indices[0]], vertices[f.vertex_indices[1]], vertices[f.vertex_indices[2]]);
	}

	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(tree);

}

CGALMeshPoint CGALMeshShell::project(const Eigen::Vector3d &pt) const {

	Surface_mesh_shortest_path shortest_paths(tmesh);

	return shortest_paths.locate(Kernel::Point_3(pt.x(), pt.y(), pt.z()), tree);
}
