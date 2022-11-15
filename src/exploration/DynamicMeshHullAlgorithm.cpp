#include <ompl/util/RandomNumbers.h>
#include "DynamicMeshHullAlgorithm.h"
#include "../utilities/trajectory_primitives.h"
#include "../shell_space/CGALMeshShell.h"
#include "../utilities/mesh_utils.h"
#include "../utilities/moveit.h"

void DynamicMeshHullAlgorithm::updatePointCloud(const moveit::core::RobotState &current_state,
												const SegmentedPointCloud &segmentedPointCloud) {

	// Update last-known robot state
	last_robot_state = current_state;
	// Compute the cached value of the end-effector position
	last_end_effector_position = last_robot_state.getGlobalLinkTransform("end_effector").translation();

	// Process the points in the point cloud individually
	for (const auto &item: segmentedPointCloud.points) {

		// Below ground? Ignore it.
		if (item.position.z() <= 1.0e-6) {
			continue;
		}

		// We assume that leaves are indicative of the tree crown, and thus we can use these by proxy for the hull.
		// TODO: This is kind of a bad assumption, but it's a good starting point for now.
		if (item.type == SegmentedPointCloud::PT_SOFT_OBSTACLE) {
			// Present the point to the hull algorithm
			pointstream_to_hull->addPoint(item.position);
		}

		// If this is a target point, and we haven't seen it before, add it to the list of target points
		if (item.type == SegmentedPointCloud::PT_TARGET &&
		    !targetPointsDedupIndex.any_within(item.position, TARGET_POINT_DEDUP_THRESHOLD)) {

			// Add to the list of known points.
			targetPointsOnChullSurface.push_back({item.position, Eigen::Vector3d::Zero()});

			// Add to the visit ordering algorithm, using the index of the new point in the list as the key
			visit_ordering.insert(targetPointsOnChullSurface.size() - 1);

			// Add to the dedup index to avoid duplicates in the future
			targetPointsDedupIndex.insert(item.position, {});

		}

	}

	// Remove target points that are close enough to the end effector
	if ((targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[0]].hull_location - last_end_effector_position).norm() <
		DISTANCE_CONSIDERED_SCANNED) {

		visit_ordering.remove(visit_ordering.getVisitOrdering()[0]);
	}

	// Call for a trajectory update based on new knowledge.
	updateTrajectory();

}

void DynamicMeshHullAlgorithm::updateTrajectory() {

	// Extract the convex hull from the streaming algorithm.
	auto chull = pointstream_to_hull->toMesh();

	// Update the point on the hull for all target points (TODO: Cache if not modified?)
	for (auto &[original, projection]: targetPointsOnChullSurface) {

		// TODO: Could probably benefit from a spatial index of the triangles?
		projection = closestPointOnMesh(chull, original);

	}

	// Update the visit ordering based on the new hull. We only do a single pass,
	// relying on future updates to progressively refine the ordering.
	visit_ordering.iterate();

	// If we have targets and a hull that's at least a tetrahedron, we can compute a trajectory.
	if (!visit_ordering.getVisitOrdering().empty() && chull.triangles.size() >= 4) {

		// Get a mesh shell based on the latest hull.
		auto shell =
				std::make_shared<ArmHorizontalDecorator<CGALMeshPoint>>(
						std::make_shared<CGALMeshShell>(chull, 1.0, 0.0));

		// Later portions of the trajectory are likely to be invalidated, so we simply stop computing after a time limit.
		auto deadline = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(ITERATION_COMPUTE_TIME_LIMIT);

		MoveItShellSpace<CGALMeshPoint> shell_space(last_robot_state.getRobotModel(), shell);

		auto current_state_shell_proj = shell->nearest_point_on_shell(last_end_effector_position);

		std::vector<moveit::core::RobotState> states {
				last_robot_state,
//				shell_space.stateFromPoint(current_state_shell_proj)
		};

		for (size_t i = 0; i < visit_ordering.getVisitOrdering().size(); i++) {

			CGALMeshPoint from_point;

			if (i == 0) {
				from_point = current_state_shell_proj;
			} else {
				from_point = shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[
						i - 1]].hull_location);
			}

			auto to_point = shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[i]]
																  .hull_location);

			auto shell_path = shell_space.shellPath(from_point, to_point);

			for (const auto &point: shell_path.waypoints) {
				states.push_back(point);
			}

			auto now = std::chrono::high_resolution_clock::now();

			if (now > deadline || states.size() > 10) {
				break;
			}

		}

		robot_trajectory::RobotTrajectory upcoming_trajectory(last_robot_state.getRobotModel(), "whole_body");

		for (const auto &state: states) {
			upcoming_trajectory.addSuffixWayPoint(state, upcoming_trajectory.getWayPointCount() == 0 ? 0.0 : upcoming_trajectory.getLastWayPoint().distance(state));
		}

		trajectoryCallback(upcoming_trajectory);
	} else {
		// Otherwise, just spin in place until finding something of interest.
		trajectoryCallback(turnInPlace(last_robot_state, 0.1));
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
