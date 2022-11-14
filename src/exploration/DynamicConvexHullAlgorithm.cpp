#include <ompl/util/RandomNumbers.h>
#include "DynamicMeshHullAlgorithm.h"
#include "../utilities/trajectory_primitives.h"
#include "../utilities/math_utils.h"
#include "../shell_space/CGALMeshShell.h"
#include "../utilities/msgs_utilities.h"

Eigen::Vector3d closestPointOnMesh(const shape_msgs::msg::Mesh &chull, const Eigen::Vector3d &original) {
	double closest_distance = std::numeric_limits<double>::max();
	Eigen::Vector3d closest_point;

	for (const auto& tr : chull.triangles) {
		Eigen::Vector3d a = toEigen(chull.vertices[tr.vertex_indices[0]]);
		Eigen::Vector3d b = toEigen(chull.vertices[tr.vertex_indices[1]]);
		Eigen::Vector3d c = toEigen(chull.vertices[tr.vertex_indices[2]]);

		Eigen::Vector3d candidate = closest_point_on_triangle(original,  a, b, c);

		double distance = (original - candidate).norm();
		if (distance < closest_distance) {
			closest_distance = distance;
			closest_point = candidate;
		}
	}
	return closest_point;
}

void DynamicMeshHullAlgorithm::updatePointCloud(const moveit::core::RobotState &current_state,
												  const SegmentedPointCloud &segmentedPointCloud) {

	last_end_effector_position = current_state.getGlobalLinkTransform("end_effector").translation();

	size_t target_points_before = targetPointsOnChullSurface.size();

	for (const auto &item: segmentedPointCloud.points) {

		if (item.position.z() <= 1.0e-6) {
			continue;
		}

		if (item.type == SegmentedPointCloud::PT_SOFT_OBSTACLE) {
			pointstream_to_hull->addPoint(item.position);
		}

		if (item.type == SegmentedPointCloud::PT_TARGET && !targetPoints.any_within(item.position, 0.05)) {

			std::cout << "Adding target point " << item.position.transpose() << std::endl;

			targetPointsOnChullSurface.push_back({item.position, Eigen::Vector3d::Zero(), false});

			targetPoints.insert(item.position, {});

		}

	}

	auto chull = pointstream_to_hull->toMesh();

	for (auto& [original, projection, visited] : targetPointsOnChullSurface) {

		// TODO: Could probably benefit from a spatial index of the triangles?
		projection = closestPointOnMesh(chull, original);

	}

	for (size_t i = target_points_before; i < targetPointsOnChullSurface.size(); i++) {
		visit_ordering.insert(i);
	}

	visit_ordering.iterate();

	if (!visit_ordering.getVisitOrdering().empty() && chull.triangles.size() >= 4) {

		auto shell = std::make_shared<CGALMeshShell>(chull, 1.0, 0.1);

		auto start_time = std::chrono::high_resolution_clock::now();

		robot_trajectory::RobotTrajectory upcoming_trajectory(current_state.getRobotModel(), "whole_body");

		upcoming_trajectory.addSuffixWayPoint(current_state, 0.0);

		MoveItShellSpace<CGALMeshPoint> shell_space(current_state.getRobotModel(), shell);

		// Remove target points that are close enough to the end effector
		if ((targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[0]].hull_location -
			 last_end_effector_position).norm() < 0.01) {

			targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[0]].visited = true;

			visit_ordering.remove(visit_ordering.getVisitOrdering()[0]);
		}

		{
			auto first_on_shell = shell_space
					.stateFromPoint(shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering
					.getVisitOrdering()[0]].hull_location));

			upcoming_trajectory
				.addSuffixWayPoint(first_on_shell,
								   upcoming_trajectory.getLastWayPoint().distance(first_on_shell));
		}

		for (size_t i = 0; i < visit_ordering.getVisitOrdering().size(); i++) {

			CGALMeshPoint from_point;

			if (i == 0) {
				from_point = shell->nearest_point_on_shell(last_end_effector_position);
			} else {
				from_point = shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[
						i - 1]].hull_location);
			}

			auto to_point = shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[
					i]].hull_location);

			auto shell_path = shell_space.shellPath(from_point, to_point);

			for (const auto &point: shell_path.waypoints) {
				upcoming_trajectory.addSuffixWayPoint(point, upcoming_trajectory.getLastWayPoint().distance(point));
			}

			auto now = std::chrono::high_resolution_clock::now();

			if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > 30) {
				break;
			}

		}

		trajectoryCallback(upcoming_trajectory);
	} else {
		trajectoryCallback(turnInPlace(current_state, 0.1));
	}

}

DynamicMeshHullAlgorithm::DynamicMeshHullAlgorithm(const moveit::core::RobotState &initial_state,
												   const std::function<void(robot_trajectory::RobotTrajectory)> &trajectoryCallback,
												   std::shared_ptr<StreamingMeshHullAlgorithm> pointstreamToHull)
		: OnlinePointCloudMotionControlAlgorithm(trajectoryCallback),
		  targetPoints(0.05, 1000),
		  visit_ordering([this](size_t i) { return (targetPointsOnChullSurface[i].hull_location - last_end_effector_position).norm(); },
						 [this](size_t i, size_t j) { return (targetPointsOnChullSurface[i].hull_location - targetPointsOnChullSurface[j].hull_location).norm(); },
						 [](const std::vector<size_t>&indices) { }),
						 pointstream_to_hull(std::move(pointstreamToHull)) {

	last_end_effector_position = initial_state.getGlobalLinkTransform("end_effector").translation();

}


const std::vector<DynamicMeshHullAlgorithm::TargetPoint> & DynamicMeshHullAlgorithm::getTargetPointsOnChullSurface() const {
	return targetPointsOnChullSurface;
}

const AnytimeOptimalInsertion<size_t> &DynamicMeshHullAlgorithm::getVisitOrdering() const {
	return visit_ordering;
}

const std::shared_ptr<StreamingMeshHullAlgorithm> &DynamicMeshHullAlgorithm::getConvexHull() const {
	return pointstream_to_hull;
}
