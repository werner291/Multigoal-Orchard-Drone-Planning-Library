// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/15/24.
//

#include <fcl/common/types.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision.h>

#include "collision_detection.h"

bool mgodpl::check_link_collision(const mgodpl::robot_model::RobotModel::Link &link,
								  const fcl::CollisionObjectd &tree_trunk_object,
								  const math::Transformd &link_tf) {
	bool collision = false;

	for (const auto &collision_geometry: link.collision_geometry) {
		if (const auto &box = std::get_if<Box>(&collision_geometry.shape)) {
			math::Transformd total_tf = link_tf.then(collision_geometry.transform);

			fcl::Transform3d fcl_tf;
			fcl_tf.setIdentity();
			fcl_tf.translation() = fcl::Vector3d(total_tf.translation.x(), total_tf.translation.y(),
												 total_tf.translation.z());
			fcl_tf.rotate(fcl::Quaterniond(total_tf.orientation.w, total_tf.orientation.x,
										   total_tf.orientation.y, total_tf.orientation.z));

			fcl::CollisionObjectd box_object(
					std::make_shared<fcl::Boxd>(box->size.x(), box->size.y(), box->size.z()),
					fcl_tf);

			fcl::CollisionRequestd request;
			fcl::CollisionResultd result;
			fcl::collide(
					&tree_trunk_object,
					&box_object,
					request,
					result
			);

			if (result.isCollision()) {
				collision = true;
				break;
			}
		} else {
			throw std::runtime_error("Only boxes are implemented for collision geometry.");
		}
	}

	return collision;
}

bool mgodpl::check_robot_collision(const mgodpl::robot_model::RobotModel &robot,
								   const fcl::CollisionObjectd &tree_trunk_object,
								   const mgodpl::RobotState &state) {
	bool collision = false;

	const auto &fk = robot_model::forwardKinematics(
			robot,
			state.joint_values,
			robot.findLinkByName("flying_base"),
			state.base_tf
	);

	for (size_t i = 0; i < fk.link_transforms.size(); ++i) {
		const auto &link_tf = fk.link_transforms[i];

		collision |= check_link_collision(robot.getLinks()[i], tree_trunk_object, link_tf);

		if (collision) {
			break;
		}
	}

	return collision;
}

bool mgodpl::check_motion_collides(const mgodpl::robot_model::RobotModel &robot,
								   const fcl::CollisionObjectd &tree_trunk_object,
								   const mgodpl::RobotState &state1,
								   const mgodpl::RobotState &state2,
								   double &toi) {

	// Compute the distance between the two.
	double distance = equal_weights_distance(state1, state2);
	const double MAX_STEP = 0.1;

	size_t n_steps = std::ceil(distance / MAX_STEP);

	for (size_t step_i = 0; step_i <= n_steps; ++step_i) {
		double t = (double) step_i / (double) n_steps;
		auto interpolated_state = interpolate(state1, state2, t);

		// Check if the robot is in collision at the interpolated state.
		if (check_robot_collision(robot, tree_trunk_object, interpolated_state)) {
			toi = t;
			return true;
		}
	}
	return false;
}

bool mgodpl::check_path_collides(const mgodpl::robot_model::RobotModel &robot,
								 const fcl::CollisionObjectd &tree_trunk_object,
								 const mgodpl::RobotPath &path,
								 mgodpl::PathPoint &collision_point) {
	for (size_t segment_i = 0; segment_i + 1 < path.states.size(); ++segment_i) {
		const auto &state1 = path.states[segment_i];
		const auto &state2 = path.states[segment_i + 1];

		double toi;
		if (check_motion_collides(robot, tree_trunk_object, state1, state2, toi)) {
			collision_point.segment_i = segment_i;
			return true;
		}
	}
	return false;
}
