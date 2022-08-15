//
// Created by werner on 12-8-22.
//

#include "CGALMeshShell.h"
#include "../utilities/experiment_utils.h"
#include "../utilities/convex_hull.h"
#include "../utilities/geogebra.h"

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
	std::reverse(path.begin(), path.end());

	std::vector<moveit::core::RobotState> states;

	Eigen::Vector3d normal = normalAt(a);
	Eigen::Vector3d pos = toCarthesian(a);

//	std::cout << "Segment(( " << pos.x() << "," << pos.y() << "," << pos.z() << "), ( " << (pos.x() + normal.x()) << "," << (pos.y() + normal.y()) << "," << (pos.z() + normal.z()) << "))" << std::endl;

	states.push_back(state_on_shell(drone,a));

	for (auto & i : path) {

		Eigen::Vector3d new_pos(i.x(), i.y(), i.z());
		Eigen::Vector3d delta = new_pos - pos;

		if ((pos - new_pos).norm() < 1.0e-10) {
			continue;
		} else {

			if (abs(normal.dot(delta)) > 1.0e-10) {

				Eigen::Vector3d corner_normal = delta.cross(normal);

				Eigen::Vector3d new_normal = delta.cross(corner_normal).normalized();

				if (new_normal.dot(normal) < 0.0) {
					new_normal = -new_normal;
				}

				normal = new_normal;

				assert(abs(normal.dot(delta)) < 1.0e-10);

				states.push_back({ robotStateFromFacing(drone, pos + normal * padding,-normal)});
			}

			pos = new_pos;

//			std::cout << "Segment(( " << pos.x() << "," << pos.y() << "," << pos.z() << "), ( " << (pos.x() + normal.x()) << "," << (pos.y() + normal.y()) << "," << (pos.z() + normal.z()) << "))" << std::endl;

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
	return -(vb - va).cross(vc - va).normalized();
}

double CGALMeshShell::predict_path_length(const CGALMeshPoint &a, const CGALMeshPoint &b) const {

	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.add_source_point(a);
	std::vector<Kernel::Point_3> path;
	shortest_paths.shortest_path_points_to_source_points(b.first, b.second, std::back_inserter(path));

//	std::cout << "CGAL = Polyline({";
//
//	for (size_t i = 0; i < path.size(); i++) {
//		std::cout << "(" << path[i].x() << "," << path[i].y() << "," << path[i].z() << ")";
//		if (i < path.size() - 1) {
//			std::cout << ",";
//		}
//	}
//
//	std::cout << "})" << std::endl;

	double total_length = 0.0;

	for (size_t i = 0; i + 1 < path.size(); ++i) {
		double d = sqrt((path[i] - path[i + 1]).squared_length());
		total_length += d;
	}

	double total_rotation = 0.0;

	for (size_t i = 0; i + 2 < path.size(); ++i) {

		double da = angle((path[i + 1] - path[i]),path[i + 2] - path[i + 1]);

		total_rotation += da;
	}

//	std::cout << "Total length = " << total_length << std::endl;
//	std::cout << "Total rotation = " << total_rotation << std::endl;

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

	std::vector<Triangle_mesh::vertex_index> vertices;

	for (auto &v : mesh.vertices) {
		vertices.push_back(tmesh.add_vertex(Kernel::Point_3(v.x, v.y, v.z)));
	}

	for (auto &f : mesh.triangles) {
		Triangle_mesh::face_index fi = tmesh.add_face(vertices[f.vertex_indices[0]], vertices[f.vertex_indices[1]], vertices[f.vertex_indices[2]]);
		assert(fi != Triangle_mesh::null_face());
	}

	Surface_mesh_shortest_path shortest_paths(tmesh);
	shortest_paths.build_aabb_tree(tree);

//	geogebra_dump_mesh(mesh);

}

CGALMeshPoint CGALMeshShell::project(const Eigen::Vector3d &pt) const {

	Surface_mesh_shortest_path shortest_paths(tmesh);

	return shortest_paths.locate(Kernel::Point_3(pt.x(), pt.y(), pt.z()), tree);
}

std::shared_ptr<OMPLSphereShellWrapper<CGALMeshPoint>>
CGALConvexHullShellBuilder::buildShell(const AppleTreePlanningScene &scene_info,
									   const ompl::base::SpaceInformationPtr &si) const {
	return std::make_shared<OMPLSphereShellWrapper<CGALMeshPoint>>(std::make_shared<CGALMeshShell>(
			convexHull(extract_leaf_vertices(scene_info)),
			rotation_weight,
			padding
			), si);
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
