#include <ompl/util/RandomNumbers.h>
#include "DynamicMeshHullAlgorithm.h"
#include "../utilities/trajectory_primitives.h"
#include "../utilities/mesh_utils.h"
#include "CandidatePathGenerator.h"
#include "VersionedRevalidableKeyValueCache.h"

void DynamicMeshHullAlgorithm::update(const moveit::core::RobotState &current_state,
									  const SegmentedPointCloud::ByType &segmentedPointCloud) {

	// Update last-known robot state
	robot_past.addRobotState(current_state);

	// Compute the cached value of the end-effector position
	last_end_effector_position = robot_past.lastRobotState().getGlobalLinkTransform("end_effector").translation();

	// Process the point cloud
	updatePointCloud(segmentedPointCloud);

	// Recompute the convex hull shell
	updateShell();

	// Call for a trajectory update based on new knowledge.
	updateTrajectory();

}

void DynamicMeshHullAlgorithm::updateTrajectory() {

	// Later portions of the trajectory are likely to be invalidated, so we simply stop computing after a time limit.
	auto deadline = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(ITERATION_COMPUTE_TIME_LIMIT);

	// Get a mesh shell based on the latest hull.
	auto shell = std::make_shared<ArmHorizontalDecorator<CGALMeshShellPoint>>(cgal_hull);

	// Set up translation between the shell and MoveIt terms.
	MoveItShellSpace<CGALMeshShellPoint> shell_space(robot_past.lastRobotState().getRobotModel(), shell);

	// If we have targets and a hull that's at least a tetrahedron, we can compute a trajectory.
	if (!visit_ordering.getVisitOrdering().empty() && cgal_hull) {

		updateTargetPointProjections();

		// Update the visit ordering based on the new hull. We only do a single pass,
		// relying on future updates to progressively refine the ordering.
		visit_ordering.iterate();

		// Step 1: Remove the portion of lastPath until we find the robot's current position.
		advance_path_to_current();

		cut_invalid_future();

		extend_plan(deadline, shell, shell_space);

		optimizePlan();

		trajectoryCallback(lastPath.toConstantSpeedTrajectoryWithPrefix(robot_past.lastRobotState()));

	} else {
		// Otherwise, just spin in place until finding something of interest.
		trajectoryCallback(turnInPlace(robot_past.lastRobotState(), 0.1));
	}
}

void DynamicMeshHullAlgorithm::updateTargetPointProjections() {
	for (target_id target = 0; target < targetPointsOnChullSurface.size(); target++) {
		auto new_projection = cgal_hull->surface_point(cgal_hull->nearest_point_on_shell(targetPointsOnChullSurface[target]
																								 .observed_location
																								 .position));

		if (new_projection != targetPointsOnChullSurface[target].hull_location) {
			targetPointsOnChullSurface[target].hull_location = new_projection;
			approach_path_planner.invalidate(targetPointsOnChullSurface[target].observed_location.position);
		}
	}
}

void DynamicMeshHullAlgorithm::updateShell() {// Extract the convex hull from the streaming algorithm.
	auto chull = pointstream_to_hull->toMesh();

	if (chull.triangles.size() >= 4) {
		cgal_hull = std::make_shared<CGALMeshShell>(chull, 1.0, PADDING);
	}
}

void DynamicMeshHullAlgorithm::extend_plan(const std::chrono::high_resolution_clock::time_point &deadline,
										   const std::shared_ptr<ArmHorizontalDecorator<CGALMeshShellPoint>> &shell,
										   const MoveItShellSpace<CGALMeshShellPoint> &shell_space) {

	// If lastPath has n segments, that means that n targets in the current visit_ordering have a path planned to them,
	// and anything after that should visit the entries in visit_ordering in order starting form index n (assuming 0-indexing).
	for (size_t i = lastPath.segments.size(); i < visit_ordering.getVisitOrdering().size(); i++) {

		if (std::chrono::high_resolution_clock::now() > deadline || lastPath.segments.size() >= 2) {
			break;
		}

		// Try to find a path from the end of lastPath to the target point.

		RobotPath retreat_path;

		if (i == 0) {
			// This is the first target; we need a path from the robot's current state up to the shell.

			retreat_path = computeInitialApproachPath(shell_space);

		} else {

			auto retreat_path_optional = approachPathForTarget(visit_ordering.getVisitOrdering()[i - 1]);

			if (!retreat_path_optional) {
				// We couldn't find a path to the target point, so we stop planning.
				std::cout
						<< "Couldn't find a path retreating from the target point, how did we get there in the first place?"
						<< std::endl;
				break;
			}

			std::reverse(retreat_path_optional->waypoints.begin(), retreat_path_optional->waypoints.end());

			// Changes to the convex hull can cause minor changes to the state-at-target.
			// For now, we patch it by simply prepending the state-at-target form the previous
			// approach path to the new one as the change is usually only minor.
			// TODO But we may want to do something more sophisticated in the future.
			retreat_path_optional->waypoints.push_back(lastPath.segments.back().waypoints.back());

			retreat_path = *retreat_path_optional;

		}

		auto approach_path = approachPathForTarget(visit_ordering.getVisitOrdering()[i]);

		if (!approach_path) {
			std::cout << "Target point " << visit_ordering.getVisitOrdering()[i]
					  << " is unreachable, removing from visit ordering." << std::endl;

			// We couldn't find a path to the target point, so we stop planning.
			visit_ordering.remove(visit_ordering.getVisitOrdering()[i]);
			i -= 1; // I don't like doing this?

			continue;
		}

		// Compute a path across the shell between the ends of the two approach/retreat paths.
		RobotPath shell_path = shell_space.shellPath(shell_space.pointNearState(retreat_path.waypoints.back()),
													 shell_space.pointNearState(approach_path->waypoints.front()));

		// Compose it all together.
		RobotPath segment;
		//		segment.append(retreat_path);
		segment.append(shell_path);
		//		segment.append(*approach_path);

		lastPath.segments.push_back(segment);


	}
}

RobotPath
DynamicMeshHullAlgorithm::computeInitialApproachPath(const MoveItShellSpace<CGALMeshShellPoint> &shell_space) {
	RobotPath retreat_path;
	auto shell_point = shell_space.pointNearState(robot_past.lastRobotState());

	moveit::core::RobotState shell_state = shell_space.stateFromPoint(shell_point);

	retreat_path = {{robot_past.lastRobotState(), shell_state}};
	return retreat_path;
}

void DynamicMeshHullAlgorithm::cut_invalid_future() {

	for (size_t segment_i = 0; segment_i < lastPath.segments.size(); segment_i++) {
		// Need to check: is this segment still valid, and if so, does it still lead to the right target?

		bool has_known_collision = collision_detector.checkCollisionInterpolated(lastPath.segments[segment_i],
																				 COLLISION_DETECTION_MAX_STEP);

		Eigen::Vector3d target_point = targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[segment_i]].observed_location
				.position;

		Eigen::Vector3d end_effector_after_segment = lastPath.segments[segment_i].waypoints
				.back()
				.getGlobalLinkTransform("end_effector")
				.translation();

		bool goes_to_target = (target_point - end_effector_after_segment).norm() < 1.0e-6;

		if (has_known_collision || !goes_to_target) {
			// This segment is no longer valid. Remove it and all subsequent segments.
			lastPath.segments.erase(lastPath.segments.begin() + (int) segment_i, lastPath.segments.end());

			std::cout << "Deleted all segments from " << segment_i << " onwards because of: "
					  << (has_known_collision ? "collision" : "wrong target") << std::endl;
			break;
		}

	}
}

void DynamicMeshHullAlgorithm::advance_path_to_current() {

	double last_step_size = robot_past.lastStepSize();

	while (!lastPath.empty() && lastPath.first_waypoint().distance(robot_past.fromBack(0)) < last_step_size) {

		if (lastPath.is_at_target(lastPath.first_waypoint_index())) {
			std::cout << "Successfully reached target point " << visit_ordering.getVisitOrdering()[0] << std::endl;
			visit_ordering.remove(visit_ordering.getVisitOrdering()[0]);
		}

		lastPath.pop_first();

	}
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
		  robot_past(2, initial_state),
		  approach_path_planner() {

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

	const auto &[obstacle, soft_obstacle, target] = segmentedPointCloud;

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

void DynamicMeshHullAlgorithm::processObstaclePoints(const std::vector<Eigen::Vector3d> &soft_obstacle) {
	// We assume that leaves are indicative of the tree crown, and thus we can use these by proxy for the hull.
	// We present the leaves to the hull algorithm
	for (const auto &obstacle_point: soft_obstacle) {
		pointstream_to_hull->addPoint(obstacle_point);
	}
}

void DynamicMeshHullAlgorithm::optimizePlan() {

	auto shell = std::make_shared<ArmHorizontalDecorator<CGALMeshShellPoint>>(cgal_hull);

	MoveItShellSpace<CGALMeshShellPoint> shell_space(robot_past.lastRobotState().getRobotModel(), shell);

	// First, we break up longer segments by interpolation.
	for (auto &segment: lastPath.segments) {
		segment.split_long_segments(0.2);
	}

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
	moveit::core::RobotState last_state = robot_past.lastRobotState();
	double distance_from_path_start = 0.0;

	for (size_t segment_i = 0; segment_i < lastPath.segments
			.size(); segment_i++) { // NOLINT(modernize-loop-convert) (We might use the index later)

		const auto &segment_target = targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[segment_i]];

		for (size_t waypoint_i = 0; waypoint_i + 1 < lastPath.segments[segment_i].waypoints
				.size(); waypoint_i++) { // NOLINT(modernize-loop-convert) (We might use the index later)

			auto &waypoint = lastPath.segments[segment_i].waypoints[waypoint_i];

			if (distance_from_path_start > 0.5) {

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

					double distance_from_end_effector = (waypoint.getGlobalLinkTransform("end_effector").translation() -
							robot_past.lastRobotState()
																 .getGlobalLinkTransform("end_effector")
																 .translation()).norm();

				}
			}

			distance_from_path_start += waypoint.distance(last_state);
			last_state = waypoint;

		}

	}

}

std::optional<RobotPath>
DynamicMeshHullAlgorithm::approachPathForTarget(DynamicMeshHullAlgorithm::target_id target_index) {
	return approach_path_planner.approachPathForTarget(targetPointsOnChullSurface[target_index].observed_location
															   .position,
													   collision_detector,
													   *cgal_hull,
													   MoveItShellSpace<CGALMeshShellPoint>(robot_past.lastRobotState()
																									.getRobotModel(),
																							std::make_shared<ArmHorizontalDecorator<CGALMeshShellPoint>>(
																									cgal_hull)));
}
