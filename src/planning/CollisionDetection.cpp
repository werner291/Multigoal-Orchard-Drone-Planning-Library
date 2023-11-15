// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/2/23.
//

#include "CollisionDetection.h"

#include "MyCollisionEnv.h"
#include "moveit_forward_declarations.h"

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <geometric_shapes/shape_operations.h>

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/math/bv/OBB.h>
#include <fcl/math/bv/RSS.h>

bool mgodpl::moveit_facade::CollisionDetection::collides(const mgodpl::moveit_facade::JointSpacePoint &state) const {

}

bool mgodpl::moveit_facade::CollisionDetection::collides_ccd(const mgodpl::moveit_facade::JointSpacePoint &state1,
															 const mgodpl::moveit_facade::JointSpacePoint &state2) const {

	return collision_ccd_toi(state1, state2).has_value();

}

mgodpl::moveit_facade::CollisionDetection::CollisionDetection(const std::vector<shape_msgs::msg::Mesh> &obstacle_meshes,
															  const moveit::core::RobotModelConstPtr robot) :
	robot_model(robot),
    obstacle_bvh_obb(std::make_shared<fcl::BvhModel<fcl::OBBd>>()),
	obstacle_bvh_rss(std::make_shared<fcl::BvhModel<fcl::RSSd>>()) {

	// this->collision_env = std::make_shared<MyCollisionEnv>(robot, std::make_shared<collision_detection::World>(), 0.0, 1.0);
	//
	// // Add the obstacles to the collision environment.
	// for (const auto &mesh : obstacle_meshes) {
	// 	this->collision_env->getWorld()->addToObject(
	// 			"static_obstacles",
	// 			shapes::ShapeConstPtr(shapes::constructShapeFromMsg(mesh)),
	// 			Eigen::Isometry3d::Identity()
	// 			);
	// }

	obstacle_bvh_obb->beginModel();
	obstacle_bvh_rss->beginModel();



	for (const auto &mesh : obstacle_meshes) {

		std::vector<fcl::Vector3d> vertices;
		std::vector<fcl::Triangle> triangles;

		for (const auto& vertex: mesh.vertices) {
			vertices.emplace_back(vertex.x, vertex.y, vertex.z);
		}

		for (const auto &triangle : mesh.triangles) {

			triangles.emplace_back(
					triangle.vertex_indices[0],
					triangle.vertex_indices[1],
					triangle.vertex_indices[2]
					);

		}

		obstacle_bvh_obb->addSubModel(vertices, triangles);

	}

}

bool mgodpl::moveit_facade::CollisionDetection::path_collides(const mgodpl::moveit_facade::JointSpacePath &path) {

	for (size_t i = 0; i + 1 < path.path.size(); ++i) {
		if (collides_ccd(path.path[i], path.path[i + 1])) {
			return true;
		}
	}

	return false;

}

std::optional<double>
mgodpl::moveit_facade::CollisionDetection::collision_ccd_toi(const mgodpl::moveit_facade::JointSpacePoint &state1,
															 const mgodpl::moveit_facade::JointSpacePoint &state2) const {

	MyCollisionEnv::ContinuousCollisionRequest request;
	MyCollisionEnv::ContinuousCollisionResult result;

	request.distance_threshold = 0.01;

	moveit::core::RobotState robot_state1(collision_env->getRobotModel());
	moveit::core::RobotState robot_state2(collision_env->getRobotModel());

	state1.to_moveit(robot_state1);
	state2.to_moveit(robot_state2);

	collision_env->checkRobotCollision(request, result, robot_state1, robot_state2);

	if (result.collision) {
		return result.time_of_contact;
	} else {
		return std::nullopt;
	}

}
