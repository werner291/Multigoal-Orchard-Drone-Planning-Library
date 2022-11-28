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

void localOptimizeSegmentedPath(const RobotPastTrace &robot_past,
								SegmentedRobotPath &lastPath,
								const DirectPointCloudCollisionDetection &collision_detector) {


	const double PREFERRED_MINIMUM_CLEARANCE = 1.0;

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

			// TODO: Do proper IK gradients here instead of this translation! We don't consider arm angle changes here.

			moveit::core::RobotState proposed_state = waypoint;

			setBaseTranslation(proposed_state, getBaseTranslation(waypoint) + direction * 0.1);

			proposed_states.push_back(proposed_state);

		}

		{

			moveit::core::RobotState proposed_state = waypoint;
			prev_waypoint.interpolate(next_waypoint, 0.5, proposed_state);
			proposed_states.push_back(proposed_state);

		}

//		{
//
//			Eigen::Vector3d old_ee_motion = (getEndEffectorPosition(next_waypoint) - getEndEffectorPosition(prev_waypoint)).normalized();
//
//			Eigen::Vector3d old_ee_pos = getEndEffectorPosition(waypoint);
//
//			// TODO: This could use some IK gradients as well!
//			proposed_states.push_back(robotStateFromPointAndArmvec(waypoint.getRobotModel(), old_ee_pos, old_ee_motion));
//
//		}

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

