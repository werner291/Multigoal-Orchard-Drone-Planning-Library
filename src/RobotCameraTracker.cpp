// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#include "RobotCameraTracker.h"

RobotCameraTracker::RobotCameraTracker(vtkCamera *camera,
									   const moveit::core::RobotState &state,
									   Eigen::AlignedBox3d sceneBounds) : camera(camera), scene_bounds(sceneBounds) {
	// Initialize robot position
	previous_camera_position = {10.0, 10.0, 10.0};


}

std::pair<Eigen::Vector2d, Eigen::Vector2d>
closestPointAndNormalOnAABB(const Eigen::AlignedBox2d &aabb, const Eigen::Vector2d &q) {
	Eigen::Vector2d result;
	Eigen::Vector2d normal;

	if (aabb.contains(q)) {
		double distances[4] = {std::abs(q[0] - aabb.min()[0]), std::abs(q[0] - aabb.max()[0]),
							   std::abs(q[1] - aabb.min()[1]), std::abs(q[1] - aabb.max()[1])};

		int minIndex = std::distance(distances, std::min_element(distances, distances + 4));

		switch (minIndex) {
			case 0:
				result = {aabb.min()[0], q[1]};
				normal = {-1.0, 0.0};
				break;
			case 1:
				result = {aabb.max()[0], q[1]};
				normal = {1.0, 0.0};
				break;
			case 2:
				result = {q[0], aabb.min()[1]};
				normal = {0.0, -1.0};
				break;
			case 3:
				result = {q[0], aabb.max()[1]};
				normal = {0.0, 1.0};
				break;
		}
	} else {
		result[0] = std::clamp(q[0], aabb.min()[0], aabb.max()[0]);
		result[1] = std::clamp(q[1], aabb.min()[1], aabb.max()[1]);

		if (result[0] == aabb.min()[0]) {
			normal = {-1.0, 0.0};
		} else if (result[0] == aabb.max()[0]) {
			normal = {1.0, 0.0};
		} else if (result[1] == aabb.min()[1]) {
			normal = {0.0, -1.0};
		} else if (result[1] == aabb.max()[1]) {
			normal = {0.0, 1.0};
		}
	}

	return {result, normal};
}


void RobotCameraTracker::update(const moveit::core::RobotState &state) {

	Eigen::Vector3d current_robot_position = state.getGlobalLinkTransform("base_link").translation();

	Eigen::Vector2d flat = current_robot_position.topRows<2>();
	Eigen::AlignedBox2d flat_bounds(scene_bounds.min().topRows<2>(), scene_bounds.max().topRows<2>());

	auto [closest_point, normal] = closestPointAndNormalOnAABB(flat_bounds, flat);

	Eigen::Vector3d camera_position{closest_point.x() + normal.x() * 2.0, closest_point.y() + normal.y() * 2.0,
									current_robot_position.z() + 2.0};

	// Interpolate between the current and new position to make the camera movement smoother

	previous_camera_position = camera_position;// * 0.1 + previous_camera_position * 0.9;

	camera->SetEyePosition(previous_camera_position.data());
	camera->SetFocalPoint(current_robot_position.data());
	camera->SetViewUp(0.0, 0.0, 1.0);


}
