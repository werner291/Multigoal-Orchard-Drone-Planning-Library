#include <ompl/util/RandomNumbers.h>
#include "DynamicMeshHullAlgorithm.h"
#include "../utilities/trajectory_primitives.h"
#include "../utilities/mesh_utils.h"
#include "CandidatePathGenerator.h"

void DynamicMeshHullAlgorithm::update(const moveit::core::RobotState &current_state,
									  const SegmentedPointCloud::ByType &segmentedPointCloud) {

	// Update last-known robot state
	last_robot_states.push_back(current_state);

	if (last_robot_states.size() > 2) {
		last_robot_states.erase(last_robot_states.begin(), last_robot_states.end() - 2);
	}

	// Compute the cached value of the end-effector position
	last_end_effector_position = last_robot_states.back().getGlobalLinkTransform("end_effector").translation();

	// Process the point cloud
	updatePointCloud(segmentedPointCloud);

	// Delete targets from our to-do list when necessary
	removeVisitedTargets();

	// Recompute the convex hull shell
	updateShell();

	// Call for a trajectory update based on new knowledge.
	updateTrajectory();

}

void DynamicMeshHullAlgorithm::removeVisitedTargets() {
	if (!targetPointsOnChullSurface.empty()) {
		// Remove target points that are close enough to the end effector
		if ((targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[0]].hull_location -
			 last_end_effector_position).norm() < DISTANCE_CONSIDERED_SCANNED) {

			visit_ordering.remove(visit_ordering.getVisitOrdering()[0]);
		}
	}
}

void DynamicMeshHullAlgorithm::updateTrajectory() {

	// Later portions of the trajectory are likely to be invalidated, so we simply stop computing after a time limit.
	std::chrono::high_resolution_clock::time_point deadline = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(ITERATION_COMPUTE_TIME_LIMIT);

	// Get a mesh shell based on the latest hull.
	auto shell = std::make_shared<ArmHorizontalDecorator<CGALMeshPoint>>(cgal_hull);

	// Set up translation between the shell and MoveIt terms.
	MoveItShellSpace<CGALMeshPoint> shell_space(last_robot_states.back().getRobotModel(), shell);

	// If we have targets and a hull that's at least a tetrahedron, we can compute a trajectory.
	if (!visit_ordering.getVisitOrdering().empty() && cgal_hull) {

		// Update the point on the hull for all target points (TODO: Cache if not modified?)
		for (auto &[original, projection]: targetPointsOnChullSurface) {
			projection = shell->surface_point(shell->nearest_point_on_shell(original.position));
		}

		// Update the visit ordering based on the new hull. We only do a single pass,
		// relying on future updates to progressively refine the ordering.
		visit_ordering.iterate();

		// Step 1: Remove the portion of lastPath until we find the robot's current position.
		advance_path_to_current();

		cut_invalid_future();

		extend_plan(deadline, shell, shell_space);

//		optimizePlan();

		emitUpdatedPath();

	} else {
		// Otherwise, just spin in place until finding something of interest.
		trajectoryCallback(turnInPlace(last_robot_states.back(), 0.1));
	}
}

void DynamicMeshHullAlgorithm::updateShell() {// Extract the convex hull from the streaming algorithm.
	auto chull = pointstream_to_hull->toMesh();

	if (chull.triangles.size() >= 4) {
		cgal_hull = std::make_shared<CGALMeshShell>(chull, 1.0, PADDING);
	}
}

void DynamicMeshHullAlgorithm::extend_plan(const std::chrono::high_resolution_clock::time_point &deadline,
										   const std::shared_ptr<ArmHorizontalDecorator<CGALMeshPoint>> &shell,
										   const MoveItShellSpace<CGALMeshPoint> &shell_space) {

	// If lastPath has n segments, that means that n targets in the current visit_ordering have a path planned to them,
	// and anything after that should visit the entries in visit_ordering in order starting form index n (assuming 0-indexing).
	for (size_t i = lastPath.size(); i < visit_ordering.getVisitOrdering().size(); i++) {

		// Extract the target point object from the visit ordering and the hull projection from the targetPointsOnChullSurface map.
		auto target_point = targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[i]];

		// Find out the shell point that's closest to the target point.
		auto to_point = shell->nearest_point_on_shell(target_point.hull_location);

		// Try to find a path from the end of lastPath to the target point.
		auto segment = pathToTargetPoint(shell, shell_space, target_point, to_point,
										 lastPath.empty() ? last_robot_states.back() : lastPath.back().waypoints.back());

		if (segment) {
			lastPath.push_back(*segment);
		} else {
			visit_ordering.remove(visit_ordering.getVisitOrdering()[i]);
			i -= 1; // I don't like doing this?
		}

		if (std::chrono::high_resolution_clock::now() > deadline || lastPath.size() > 10) {
			break;
		}

	}
}

std::optional<RobotPath>
DynamicMeshHullAlgorithm::pathToTargetPoint(const std::shared_ptr<ArmHorizontalDecorator<CGALMeshPoint>> &shell,
											const MoveItShellSpace<CGALMeshPoint> &shell_space,
											const DynamicMeshHullAlgorithm::TargetPoint &target_point,
											const CGALMeshPoint &to_point,
											const moveit::core::RobotState &from_state) {

	auto current_state_shell_proj = shell->nearest_point_on_shell(last_end_effector_position);

	CGALMeshPoint from_point = shell->nearest_point_on_shell(from_state.getGlobalLinkTransform("end_effector").translation());

	RobotPath segment = shell_space.shellPath(from_point, to_point);

	// Edge case: if the from_state is near an edge or a vertex of the mesh, the face normal is not well-defined.
	// This may cause a problem to arise where the robot first moves in one direction, the face normal flips, and
	// the robot moves in the opposite direction, getting stuck in a loop.
	// To avoid this, we explicitly check if the direction the end-effector moves in flips unexpectedly.
	removeFrontReversal(from_state, segment);

	{
		CandidatePathGenerator path_generator(segment.waypoints.back(), target_point.observed_location);
		auto approach_path = path_generator.generateCandidatePath();
		segment.append(approach_path);
	}


	// Collision-check.
	for (auto &waypoint: segment.waypoints) {
		if (collision_detector.checkCollision(waypoint)) {
			return std::nullopt;
		}
	}

	return {segment};
}

void DynamicMeshHullAlgorithm::removeFrontReversal(const moveit::core::RobotState &from_state,
												   RobotPath &path) {

	if (path.waypoints.size() >= 2) {
		Eigen::Vector3d ee_0 = from_state.getGlobalLinkTransform("end_effector").translation();
		Eigen::Vector3d ee_1 = path.waypoints[0].getGlobalLinkTransform("end_effector").translation();
		Eigen::Vector3d ee_2 = path.waypoints[1].getGlobalLinkTransform("end_effector").translation();

		Eigen::Vector3d dir_0 = ee_1 - ee_0;
		Eigen::Vector3d dir_1 = ee_2 - ee_1;

		if (dir_0.dot(dir_1) < 0) {
			path.waypoints.erase(path.waypoints.begin(), path.waypoints.begin() + 1);
		}
	}
}

void DynamicMeshHullAlgorithm::cut_invalid_future() {

	for (size_t segment_i = 0; segment_i < lastPath.size(); segment_i++) {
		// Need to check: is this segment still valid, and if so, does it still lead to the right target?

		bool on_shell = true;

		for (auto & waypoint : lastPath[segment_i].waypoints) {
			// TODO Will want to take some intermediate points into account as well.
			if (collision_detector.checkCollision(waypoint)) {
				on_shell = false;
			}
		}

		Eigen::Vector3d target_point = targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[segment_i]].observed_location.position;
		Eigen::Vector3d end_effector_after_segment = lastPath[segment_i].waypoints.back().getGlobalLinkTransform("end_effector").translation();

		bool goes_to_target = (target_point - end_effector_after_segment).norm() < 1.0e-6;

		if (!on_shell || !goes_to_target) {
			// This segment is no longer valid. Remove it and all subsequent segments.
			lastPath.erase(lastPath.begin() + (int) segment_i, lastPath.end());
			break;
		}

	}
}

void DynamicMeshHullAlgorithm::advance_path_to_current() {

	for (size_t path_i = 0; path_i < lastPath.size(); path_i++) {

		for (size_t segment_i = 0; segment_i + 1 < lastPath.front().waypoints.size(); segment_i++) {

			auto from_wp = lastPath.front().waypoints[segment_i];
			auto to_wp = lastPath.front().waypoints[segment_i + 1];

			double segment_length = from_wp.distance(to_wp);

			double start_to_state = from_wp.distance(last_robot_states.back());
			double state_to_end = to_wp.distance(last_robot_states.back());

			bool on_segment = std::abs(start_to_state + state_to_end - segment_length) < 1.0e-6;

			if (on_segment) {

				lastPath.erase(lastPath.begin(), lastPath.begin() + (int) path_i);

				// We found the current state on the segment. Remove everything before it.
				lastPath.front().waypoints.erase(lastPath.front().waypoints.begin(),
												 lastPath.front().waypoints.begin() + segment_i+1);
			}

		}

	}

}

void DynamicMeshHullAlgorithm::emitUpdatedPath() {
	robot_trajectory::RobotTrajectory upcoming_trajectory(last_robot_states.back().getRobotModel(), "whole_body");

	upcoming_trajectory.addSuffixWayPoint(last_robot_states.back(), 0.0);

	for (const auto &segment: lastPath) {
		for (const auto &substate: segment.waypoints) {
			upcoming_trajectory.addSuffixWayPoint(substate,  upcoming_trajectory.getLastWayPoint().distance(substate));
		}
	}

	trajectoryCallback(upcoming_trajectory);
}

DynamicMeshHullAlgorithm::DynamicMeshHullAlgorithm(const moveit::core::RobotState &initial_state,
												   const std::function<void(robot_trajectory::RobotTrajectory)> &trajectoryCallback,
												   std::shared_ptr<StreamingMeshHullAlgorithm> pointstreamToHull)
		: OnlinePointCloudMotionControlAlgorithm(trajectoryCallback),

		  targetPointsDedupIndex(0.05, 1000),
		  visit_ordering([this](size_t i) {
			  return (targetPointsOnChullSurface[i].hull_location - last_end_effector_position).norm();
		  }, [this](size_t i, size_t j) {
			  return (targetPointsOnChullSurface[i].hull_location - targetPointsOnChullSurface[j].hull_location).norm();
		  }, [](const std::vector<size_t> &indices) {}),
		  pointstream_to_hull(std::move(pointstreamToHull)),
		  last_robot_states({initial_state}) {

	last_end_effector_position = initial_state.getGlobalLinkTransform("end_effector").translation();

}


const std::vector<DynamicMeshHullAlgorithm::TargetPoint> &
DynamicMeshHullAlgorithm::getTargetPointsOnChullSurface() const {
	return targetPointsOnChullSurface;
}

const AnytimeOptimalInsertion<size_t> &DynamicMeshHullAlgorithm::getVisitOrdering() const {
	return visit_ordering;
}

const std::shared_ptr<StreamingMeshHullAlgorithm> &DynamicMeshHullAlgorithm::getConvexHull() const {
	return pointstream_to_hull;
}

void DynamicMeshHullAlgorithm::updatePointCloud(const SegmentedPointCloud::ByType &segmentedPointCloud) {

	const auto& [obstacle, soft_obstacle, target] = segmentedPointCloud;

	processObstaclePoints(soft_obstacle);

	this->collision_detector.addPoints(obstacle);

	processTargetPoints(target);

}

void DynamicMeshHullAlgorithm::processTargetPoints(const std::vector<SegmentedPointCloud::TargetPoint> &target) {
	for (const auto &tgt_pt: target) {
		// If this is a target point, and we haven't seen it before, add it to the list of target points
		if (!targetPointsDedupIndex.any_within(tgt_pt.position, TARGET_POINT_DEDUP_THRESHOLD)) {

			// Add to the list of known points.
			targetPointsOnChullSurface.push_back({tgt_pt.position, Eigen::Vector3d::Zero()});

			// Add to the visit ordering algorithm, using the index of the new point in the list as the key
			visit_ordering.insert(targetPointsOnChullSurface.size() - 1);

			// Add to the dedup index to avoid duplicates in the future
			targetPointsDedupIndex.insert(tgt_pt.position, {});

		}
	}
}

void
DynamicMeshHullAlgorithm::processObstaclePoints(const std::vector<Eigen::Vector3d> &soft_obstacle) {// We assume that leaves are indicative of the tree crown, and thus we can use these by proxy for the hull.
	// We present the leaves to the hull algorithm
	for (const auto &obstacle_point: soft_obstacle) {
		pointstream_to_hull->addPoint(obstacle_point);
	}
}

void DynamicMeshHullAlgorithm::optimizePlan() {

	auto shell = std::make_shared<ArmHorizontalDecorator<CGALMeshPoint>>(cgal_hull);

	MoveItShellSpace<CGALMeshPoint> shell_space(last_robot_states.back().getRobotModel(), shell);

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

	double distance_to_path_head = 0.0;
	moveit::core::RobotState last_waypoint = last_robot_states.back();

	for (size_t segment_i = 0; segment_i < lastPath.size(); segment_i++) { // NOLINT(modernize-loop-convert) (We might use the index later)

		const auto& segment_target = targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[segment_i]];

		for (size_t waypoint_i = 0; waypoint_i < lastPath[segment_i].waypoints.size(); waypoint_i++) { // NOLINT(modernize-loop-convert) (We might use the index later)

			auto &waypoint = lastPath[segment_i].waypoints[waypoint_i];

			double distance_from_last = last_waypoint.distance(waypoint);

			if (distance_from_last > 0.5) {

				moveit::core::RobotState interpolated(waypoint);

				last_waypoint.interpolate(waypoint, 0.5 / distance_from_last, interpolated);

				interpolated.update();

				lastPath[segment_i].waypoints.insert(lastPath[segment_i].waypoints.begin() + waypoint_i, interpolated);

			} else if (distance_to_path_head > 0.5) {
				double distance_to_target = (waypoint.getGlobalLinkTransform("end_effector").translation() -
											 segment_target.hull_location).norm();

				const double BLEND_FACTOR_SMOOTHNESS = 2.0;

				double blend_factor = std::tanh(distance_to_target / BLEND_FACTOR_SMOOTHNESS);

				moveit::core::RobotState on_shell = shell_space.stateFromPoint(shell_space.pointNearState(waypoint));

				moveit::core::RobotState interpolated(on_shell.getRobotModel());

				on_shell.interpolate(waypoint, blend_factor, interpolated);

				interpolated.update();

				if (!collision_detector.checkCollision(interpolated)) {
					waypoint = interpolated;
				} else {
					std::cout << "Rejected." << std::endl;
				}
			}

			distance_to_path_head += waypoint.distance(last_waypoint);
			last_waypoint = waypoint;

		}

	}

}
