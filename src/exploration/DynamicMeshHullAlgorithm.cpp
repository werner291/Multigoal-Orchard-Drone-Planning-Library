#include <ompl/util/RandomNumbers.h>
#include "DynamicMeshHullAlgorithm.h"
#include "../utilities/trajectory_primitives.h"
#include "../utilities/mesh_utils.h"

void DynamicMeshHullAlgorithm::update(const moveit::core::RobotState &current_state,
									  const SegmentedPointCloud::ByType &segmentedPointCloud) {

	// Update last-known robot state
	last_robot_state = current_state;
	// Compute the cached value of the end-effector position
	last_end_effector_position = last_robot_state.getGlobalLinkTransform("end_effector").translation();

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
	MoveItShellSpace<CGALMeshPoint> shell_space(last_robot_state.getRobotModel(), shell);

	// If we have targets and a hull that's at least a tetrahedron, we can compute a trajectory.
	if (!visit_ordering.getVisitOrdering().empty() && cgal_hull) {

		// Update the point on the hull for all target points (TODO: Cache if not modified?)
		for (auto &[original, projection]: targetPointsOnChullSurface) {
			projection = shell->surface_point(shell->nearest_point_on_shell(original));
		}

		// Update the visit ordering based on the new hull. We only do a single pass,
		// relying on future updates to progressively refine the ordering.
		visit_ordering.iterate();



		// Step 1: Remove the portion of lastPath until we find the robot's current position.
		advance_path_to_current();

		cut_invalid_future();

		extend_plan(deadline, shell, shell_space);

		emitUpdatedPath();

	} else {
		// Otherwise, just spin in place until finding something of interest.
		trajectoryCallback(turnInPlace(last_robot_state, 0.1));
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

	auto current_state_shell_proj = shell->nearest_point_on_shell(last_end_effector_position);

	for (size_t i = lastPath.size(); i < visit_ordering.getVisitOrdering().size(); i++) {

		CGALMeshPoint from_point;

		if (i == 0) {
			from_point = current_state_shell_proj;
		} else {
			from_point = shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[i - 1]].hull_location);
		}

		auto to_point = shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[i]].hull_location);

		lastPath.push_back(shell_space.shellPath(from_point, to_point));

		if (std::chrono::high_resolution_clock::now() > deadline || lastPath.size() > 10) {
			break;
		}

	}
}

void DynamicMeshHullAlgorithm::cut_invalid_future() {
	for (size_t segment_i = 0; segment_i < lastPath.size(); segment_i++) {
		// Need to check: is this segment still valid, and if so, does it still lead to the right target?

		bool on_shell = true;

		for (auto & waypoint : lastPath[segment_i].waypoints) {
			Eigen::Vector3d point = waypoint.getGlobalLinkTransform("end_effector").translation();
			Eigen::Vector3d shell_point = cgal_hull->surface_point_unpadded(cgal_hull->nearest_point_on_shell(point));
			if (abs((point - shell_point).norm() - PADDING) > 0.05) {
				on_shell = false;
				break;
			}
		}

		Eigen::Vector3d target_point = targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[segment_i]].hull_location;
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

			double start_to_state = from_wp.distance(last_robot_state);
			double state_to_end = to_wp.distance(last_robot_state);

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
	robot_trajectory::RobotTrajectory upcoming_trajectory(last_robot_state.getRobotModel(), "whole_body");

	upcoming_trajectory.addSuffixWayPoint(last_robot_state, 0.0);

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
		  last_robot_state(initial_state) {

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

	processTargetPoints(target);

}

void DynamicMeshHullAlgorithm::processTargetPoints(const std::vector<Eigen::Vector3d> &target) {
	for (const auto &tgt_pt: target) {
		// If this is a target point, and we haven't seen it before, add it to the list of target points
		if (!targetPointsDedupIndex.any_within(tgt_pt, TARGET_POINT_DEDUP_THRESHOLD)) {

			// Add to the list of known points.
			targetPointsOnChullSurface.push_back({tgt_pt, Eigen::Vector3d::Zero()});

			// Add to the visit ordering algorithm, using the index of the new point in the list as the key
			visit_ordering.insert(targetPointsOnChullSurface.size() - 1);

			// Add to the dedup index to avoid duplicates in the future
			targetPointsDedupIndex.insert(tgt_pt, {});

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
