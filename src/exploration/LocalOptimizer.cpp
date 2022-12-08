//
// Created by werner on 28-11-22.
//

#include "LocalOptimizer.h"
#include "../shell_space/MoveItShellSpace.h"

void adjustSegmentation(SegmentedRobotPath &lastPath) {// First, we break up longer segments by interpolation.
	for (auto &segment: lastPath.segments) {
		segment.split_long_segments(1.0);
	}

	for (auto &segment: lastPath.segments) {
		segment.collapse_short_segments(0.05);
	}
}

/**
 *
 * Given a point and a velocity, as well as an axis of rotation, this function computes
 * the dot product between the velocity and the hypothetical velocity of the point
 * if it were rotated around the axis of rotation at unit radian speed.
 *
 * @param axis_origin 			The origin of the axis of rotation
 * @param axis_direction 		The direction of the axis of rotation
 * @param point 				The point at which we want to compute the effect
 * @param point_movement 		The movement of the point
 * @return 						The dot product between the point movement and the hypothetical movement of the point
 */
double rotation_alignment(const Eigen::Vector3d &axis_origin,
						  const Eigen::Vector3d &axis_direction,
						  const Eigen::Vector3d &point,
						  const Eigen::Vector3d &point_movement) {
	return (point - axis_origin).cross(axis_direction).dot(point_movement);
}

const moveit::core::JointModel *parentJoint(const moveit::core::JointModel *joint) {
	return joint->getParentLinkModel() == nullptr ? nullptr : joint->getParentLinkModel()->getParentJointModel();
}

moveit::core::RobotState stepIncreaseClearance(const DirectPointCloudCollisionDetection::ClosestPointOnRobot &pt,
											   const moveit::core::RobotState &rs) {

	const double STEP_SIZE = 0.1;

	moveit::core::RobotState rs2 = rs;

	// Imagine this vector as a force that pushes the robot away from the obstacle,
	// applied at the point of contact and/or nearest point on the robot.
	// We want to move the robot in the direction of this force, allowing all joints
	// to contribute to the movement rather than just moving the base of the robot.
	Eigen::Vector3d clearance_vector = pt.on_robot - pt.point;

	for (auto joint = pt.link->getParentJointModel(); joint != nullptr; joint = parentJoint(joint)) {

		Eigen::Isometry3d parent_link_transform =
				joint->getParentLinkModel() == nullptr ? Eigen::Isometry3d::Identity() : rs2.getGlobalLinkTransform(
						joint->getParentLinkModel());

		switch (joint->getType()) {

			case moveit::core::JointModel::UNKNOWN:
				break;
			case moveit::core::JointModel::REVOLUTE: {

				Eigen::Vector3d axis = parent_link_transform.rotation() *
									   dynamic_cast<const moveit::core::RevoluteJointModel *>(joint)->getAxis();
				Eigen::Vector3d origin = parent_link_transform.translation(); // TODO Is this correct?

				double influence = rotation_alignment(origin, axis, pt.on_robot, clearance_vector);

				rs2.setJointPositions(joint, {rs2.getJointPositions(joint)[0] + STEP_SIZE * influence});
			}
				break;

			case moveit::core::JointModel::PRISMATIC:
				throw std::runtime_error("Planar joints are not supported");
				break;

			case moveit::core::JointModel::PLANAR:
				throw std::runtime_error("Planar joints are not supported");
				break;

			case moveit::core::JointModel::FLOATING: {
				// Here, we assume that this joint is always upright; it is effectively a combination of a translation and a revolute joint.

				auto floating_joint = dynamic_cast<const moveit::core::FloatingJointModel *>(joint);

				const double *joint_positions = rs2.getJointPositions(joint);

				Eigen::Vector3d translation(joint_positions[0], joint_positions[1], joint_positions[2]);

				// Note: Eigen's quaternions are WXYZ, Moveit's are XYZW. Gotta be careful.
				Eigen::Quaterniond rotation(joint_positions[6],
											joint_positions[3],
											joint_positions[4],
											joint_positions[5]);

				assert(rotation * Eigen::Vector3d::UnitZ() == Eigen::Vector3d::UnitZ());

				double rotation_influence = rotation_alignment(translation,
															   Eigen::Vector3d::UnitZ(),
															   pt.on_robot,
															   clearance_vector);

				if (rotation_influence > 0) {
					rotation = rotation * Eigen::AngleAxisd(STEP_SIZE * rotation_influence, Eigen::Vector3d::UnitZ());
					rs2.setJointPositions(joint,
										  {translation.x() + STEP_SIZE * clearance_vector.x(),
										   translation.y() + STEP_SIZE * clearance_vector.y(),
										   translation.z() + STEP_SIZE * clearance_vector.z(), rotation.x(),
										   rotation.y(), rotation.z(), rotation.w()});
				}
			}

				break;
			case moveit::core::JointModel::FIXED:
				// Can't do anything.
				break;
		}

	}

	rs2.enforceBounds();

	rs2.update();

	return rs2;

}

moveit::core::RobotState
smoothWaypoint(std::pair<const moveit::core::RobotState &, const moveit::core::RobotState &> prev_next,
			   const moveit::core::RobotState &to_change) {

	const auto &[prev_waypoint, next_waypoint] = prev_next;

	moveit::core::RobotState smoothed = to_change;

	prev_waypoint.interpolate(next_waypoint, 0.5, smoothed);

	std::array<double, 7> joint_position_buffer{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	for (const auto &joint: to_change.getRobotModel()->getActiveJointModels()) {

		// We interpolate, biasing the result towards the original to_change state by the joint distance factor.

		double joint_distance_factor = joint->getDistanceFactor();

		double t = std::min(0.1 / joint_distance_factor, 0.1);

		assert(joint->getVariableCount() <= 7);

		joint->interpolate(prev_waypoint.getJointPositions(joint),
						   next_waypoint.getJointPositions(joint),
						   t,
						   joint_position_buffer.data());

		smoothed.setJointPositions(joint, joint_position_buffer.data());

	}

	return smoothed;

}


void localOptimizeSegmentedPath(const RobotPastTrace &robot_past,
								SegmentedRobotPath &lastPath,
								const DirectPointCloudCollisionDetection &collision_detector) {



	// TODO: When computing an entirely new plan, maybe we can blend it with the old one,
	// to avoid wasting some of the local optimization in the old one.
	adjustSegmentation(lastPath);

	/**
	 * Desirable properties of the path:
	 *
	 * 1. Path should be relatively short
	 * 2. Path should be relatively smooth, taking the last few states into account as well
	 * 3. Path should put some distance between the robot and the obstacles
	 * 4. Path should put some distance between the sensor and the obstacles
	 * 5. Path should have the sensor facing in the direction of motion if the robot moves into unseen areas.
	 * 6. Should avoid changing the path near the head?
	 *
	 */

	for (auto ix = lastPath.first_waypoint_index(); ix < lastPath.last_waypoint_index(); ix = lastPath.next_waypoint_index(ix)) {

		const auto &prev_waypoint = ix == lastPath.first_waypoint_index() ? robot_past.lastRobotState()
																		  : lastPath.waypoint(lastPath.prev_waypoint_index(
						ix));
		auto &waypoint = lastPath.waypoint(ix);
		const auto &next_waypoint = lastPath.waypoint(lastPath.next_waypoint_index(ix));

		double straight_distance = next_waypoint.distance(prev_waypoint);

		std::vector<moveit::core::RobotState> proposed_states;

		if (auto clearance_info = collision_detector.closestPoint(waypoint, PREFERRED_MINIMUM_CLEARANCE)) {

			assert(clearance_info->distance <= PREFERRED_MINIMUM_CLEARANCE);

			Eigen::Vector3d direction = clearance_info->on_robot - clearance_info->point;

			proposed_states.push_back(stepIncreaseClearance(*clearance_info, waypoint));

		}

		{

			moveit::core::RobotState proposed_state = waypoint;
			prev_waypoint.interpolate(next_waypoint, 0.5, proposed_state);
			proposed_states.push_back(proposed_state);
			//			proposed_states.push_back(smoothWaypoint({prev_waypoint, next_waypoint}, waypoint));

		}

		moveit::core::RobotState candidate = waypoint;

		for (const auto &proposed_state: proposed_states) {
			double distance = proposed_state.distance(waypoint);
			// TODO: Ideally I'd like to do an equal-weighted average here instead of this ordered stuff.
			// Should average out over the iterations, though.
			candidate.interpolate(proposed_state, 0.1 / std::max(distance,1.0), candidate);
		}

		if (lastPath.is_at_target(ix)) {
			candidate = setEndEffectorToPosition(std::move(candidate), getEndEffectorPosition(waypoint));
		}

		candidate.update();

		if (!collision_detector.checkCollisionInterpolated(prev_waypoint, candidate, COLLISION_DETECTION_MAX_STEP) &&
			!collision_detector.checkCollisionInterpolated(candidate, next_waypoint, COLLISION_DETECTION_MAX_STEP)) {

			waypoint = candidate;
		}
	}
}

