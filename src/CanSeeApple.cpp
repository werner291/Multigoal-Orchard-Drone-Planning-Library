// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 23-5-23.
//

#include "CanSeeApple.h"
#include "utilities/MeshOcclusionModel.h"
#include "utilities/alpha_shape.h"

#include <range/v3/to_container.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/range/conversion.hpp>

// Function checks whether an apple is occluded based on distance
bool distance_occlusion(const moveit::core::RobotState &state, const Apple &apple) {

	// Get the position of the robot's end effector
	Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

	// Define a constant for maximum discovery distance
	const double discovery_max_distance = 1.0;

	// Return whether the squared norm between the apple and the end effector is less than the square of maximum distance
	return (ee_pos - apple.center).squaredNorm() < discovery_max_distance * discovery_max_distance;

}

// Modifying mesh_occludes_vision to include link name
CanSeeAppleFn mesh_occludes_vision(const shape_msgs::msg::Mesh &mesh, const std::string& link_name) {

	auto occlusion_model = std::make_shared<MeshOcclusionModel>(mesh, 0);

	return [occlusion_model=std::move(occlusion_model), link_name](const moveit::core::RobotState &state, const Apple &apple) {

		Eigen::Vector3d ee_pos = state.getGlobalLinkTransform(link_name).translation();

		return !occlusion_model->checkOcclusion(apple.center, ee_pos);
	};
}

// Modifying leaves_alpha_shape_occludes_vision to include link name
CanSeeAppleFn leaves_alpha_shape_occludes_vision(shape_msgs::msg::Mesh leaves_mesh, const std::string& link_name) {

	auto alphashape = alphaShape(leaves_mesh.vertices | ranges::views::transform([](const auto &v) {
		return Eigen::Vector3d{v.x, v.y, v.z};
	}) | ranges::to_vector, LEAVES_ALPHA_SQRTRADIUS);

	return mesh_occludes_vision(alphashape, link_name);
}

// Function always returns true indicating no occlusion
bool omniscient_occlusion(const moveit::core::RobotState &state, const Apple &apple) {
	return true;
}

CanSeeAppleFn only_if_both(const CanSeeAppleFn &fn1, const CanSeeAppleFn &fn2) {
	return [fn1, fn2](const moveit::core::RobotState &state, const Apple &apple) {
		return fn1(state, apple) && fn2(state, apple);
	};
}

CanSeeAppleFn in_angle(double fov, Eigen::Vector3d front, std::string link_name) {

	return [fov, front, link_name](const moveit::core::RobotState &state, const Apple &apple) {

		Eigen::Vector3d ee_pos = state.getGlobalLinkTransform(link_name).translation();

		Eigen::Vector3d apple_to_ee = ee_pos - apple.center;

		return apple_to_ee.dot(front) > cos(fov / 2);

	};

}
