//
// Created by werner on 6-3-23.
//

#include "enclosing_sphere.h"
#include "Seb_point.h"
#include "Seb.h"

bodies::BoundingSphere
compute_enclosing_sphere_around_leaves(const moveit_msgs::msg::PlanningScene &planning_scene_message,
									   const double padding) {

	typedef double FT;
	typedef Seb::Point<FT> Point;
	typedef std::vector<Point> PointVector;
	typedef Seb::Smallest_enclosing_ball<FT> Miniball;

	std::vector<Point> points;

	for (const auto &col: planning_scene_message.world.collision_objects) {
		if (col.id == "leaves") {
			for (const auto& mesh : col.meshes) {
				for (auto v : mesh.vertices) {
					std::vector<FT> point_vect { v.x, v.y, v.z };

					points.emplace_back(3, point_vect.begin());
				}
			}
		}
	}

	Miniball mb(3, points);

	std::vector<FT> center(mb.center_begin(), mb.center_end());
	FT radius = sqrt(mb.squared_radius());

	Eigen::Vector3d center_eigen(center[0], center[1], center[2]);

	bodies::BoundingSphere sphere;
	sphere.center = center_eigen;
	sphere.radius = radius + padding;

	return sphere;
}

std::vector<geometry_msgs::msg::Point> extract_leaf_vertices(const AppleTreePlanningScene &scene_info) {
	std::vector<geometry_msgs::msg::Point> mesh_points;
	for (const auto &col: scene_info.scene_msg->world.collision_objects) {
		if (col.id == "leaves") {
			for (const auto &mesh: col.meshes) {
				for (auto v: mesh.vertices) {
					mesh_points.push_back(v);
				}
			}
		}
	}
	return mesh_points;
}
