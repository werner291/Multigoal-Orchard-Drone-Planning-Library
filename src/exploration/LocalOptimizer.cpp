//
// Created by werner on 28-11-22.
//

#include "LocalOptimizer.h"

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

		moveit::core::RobotState candidate = sampleStateNearByUpright(waypoint, 0.01);

		if (lastPath.is_at_target(ix)) {
			candidate = setEndEffectorToPosition(std::move(candidate), getEndEffectorPosition(waypoint));
		}

		double old_to_next = waypoint.distance(next_waypoint);
		double old_to_prev = waypoint.distance(prev_waypoint);
		double candidate_to_next = candidate.distance(next_waypoint);
		double candidate_to_prev = candidate.distance(prev_waypoint);
		double prev_to_next = prev_waypoint.distance(next_waypoint);

		const double PREFERRED_MINIMUM_CLEARANCE = 0.5;

		double candidate_clearance = collision_detector.distanceToCollision(candidate, PREFERRED_MINIMUM_CLEARANCE);
		double old_clearance = collision_detector.distanceToCollision(waypoint, PREFERRED_MINIMUM_CLEARANCE);
		double clearance_improvement = candidate_clearance - old_clearance;

		double new_roughness = candidate_to_next + candidate_to_prev - prev_to_next;
		double old_roughness = old_to_next + old_to_prev - prev_to_next;
		double roughness_change = new_roughness - old_roughness;

		double old_distance = waypoint.distance(next_waypoint);
		double new_distance = candidate.distance(next_waypoint);
		double distance_increase = new_distance - old_distance;

		Eigen::Vector3d old_ee_motion = getEndEffectorPosition(next_waypoint) - getEndEffectorPosition(prev_waypoint);
		Eigen::Vector3d old_ee_front = getEndEffectorFacing(waypoint);
		Eigen::Vector3d new_ee_front = getEndEffectorFacing(candidate);
		double old_alignment = old_ee_motion.dot(old_ee_front);
		double candidate_alignment = old_ee_motion.dot(new_ee_front);
		double alignment_improvement = candidate_alignment - old_alignment;

		const double DISTANCE_WEIGHT = 1.0;
		const double SMOOTHNESS_WEIGHT = 0.0;
		const double ALIGNMENT_WEIGHT = 1.0;
		const double CLEARANCE_WEIGHT = 1.0;

		double overall_improvement =
				-SMOOTHNESS_WEIGHT * std::pow(roughness_change, 2) - DISTANCE_WEIGHT * std::pow(distance_increase, 2) +
				ALIGNMENT_WEIGHT * std::pow(alignment_improvement, 2) + CLEARANCE_WEIGHT * std::pow(clearance_improvement, 2);

		if (overall_improvement > 0.0 &&
			!collision_detector.checkCollisionInterpolated(prev_waypoint, candidate, COLLISION_DETECTION_MAX_STEP) &&
			!collision_detector.checkCollisionInterpolated(candidate, next_waypoint, COLLISION_DETECTION_MAX_STEP)) {

			waypoint = candidate;

		}
	}
}
