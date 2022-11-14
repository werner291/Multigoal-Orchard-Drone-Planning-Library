#include <ompl/util/RandomNumbers.h>
#include "DynamicMeshHullAlgorithm.h"
#include "../utilities/trajectory_primitives.h"
#include "../utilities/math_utils.h"
#include "../shell_space/CGALMeshShell.h"

Eigen::Vector3d closestPointOnMesh(const shape_msgs::msg::Mesh &chull, const Eigen::Vector3d &original) {
	double closest_distance = std::numeric_limits<double>::max();
	Eigen::Vector3d closest_point;

	for (const auto& tr : chull.triangles) {
		Eigen::Vector3d a(chull.vertices[tr.vertex_indices[0]].x, chull.vertices[tr.vertex_indices[0]].y, chull.vertices[tr.vertex_indices[0]].z);
		Eigen::Vector3d b(chull.vertices[tr.vertex_indices[1]].x, chull.vertices[tr.vertex_indices[1]].y, chull.vertices[tr.vertex_indices[1]].z);
		Eigen::Vector3d c(chull.vertices[tr.vertex_indices[2]].x, chull.vertices[tr.vertex_indices[2]].y, chull.vertices[tr.vertex_indices[2]].z);

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

			targetPointsOnChullSurface.emplace_back(item.position, Eigen::Vector3d::Zero());

			targetPoints.insert(item.position, {});

		}

	}

	auto chull = pointstream_to_hull->toMesh();

	for (auto& [original, projection] : targetPointsOnChullSurface) {

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

		{
			auto first_on_shell = shell_space
					.stateFromPoint(shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering
					.getVisitOrdering()[0]].second));

			upcoming_trajectory
				.addSuffixWayPoint(first_on_shell,
								   upcoming_trajectory.getLastWayPoint().distance(first_on_shell));
		}


		for (size_t i = 0; i + 1 < visit_ordering.getVisitOrdering().size(); i++) {

			auto from_point = shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[i]]
																	.second);
			auto to_point = shell->nearest_point_on_shell(targetPointsOnChullSurface[visit_ordering.getVisitOrdering()[
					i + 1]].second);

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
		  visit_ordering([this](size_t i) { return (targetPointsOnChullSurface[i].second - last_end_effector_position).norm(); },
						 [this](size_t i, size_t j) { return (targetPointsOnChullSurface[i].second - targetPointsOnChullSurface[j].second).norm(); },
						 [](const std::vector<size_t>&indices) { }),
						 pointstream_to_hull(std::move(pointstreamToHull)) {

	last_end_effector_position = initial_state.getGlobalLinkTransform("end_effector").translation();

}


const std::vector<std::pair<const Eigen::Vector3d, Eigen::Vector3d>> &
DynamicMeshHullAlgorithm::getTargetPointsOnChullSurface() const {
	return targetPointsOnChullSurface;
}

const AnytimeOptimalInsertion<size_t> &DynamicMeshHullAlgorithm::getVisitOrdering() const {
	return visit_ordering;
}

const std::shared_ptr<StreamingMeshHullAlgorithm> &DynamicMeshHullAlgorithm::getConvexHull() const {
	return pointstream_to_hull;
}
