
#include "DynamicConvexHullAlgorithm.h"
#include "../utilities/trajectory_primitives.h"
#include "../utilities/math_utils.h"

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

void DynamicConvexHullAlgorithm::updatePointCloud(const moveit::core::RobotState &current_state,
												  const SegmentedPointCloud &segmentedPointCloud) {

	last_end_effector_position = current_state.getGlobalLinkTransform("end_effector").translation();

	trajectoryCallback(turnInPlace(current_state, 0.1));

	size_t target_points_before = targetPointsOnChullSurface.size();

	for (const auto &item: segmentedPointCloud.points) {

		if (item.position.z() <= 1.0e-6) {
			continue;
		}

		if (item.type == SegmentedPointCloud::PT_SOFT_OBSTACLE) {
			convexHull.addPoint(item.position);
		}

		if (item.type == SegmentedPointCloud::PT_TARGET && !targetPoints.any_within(item.position, 0.05)) {

			targetPointsOnChullSurface.emplace_back(item.position, Eigen::Vector3d::Zero());

			targetPoints.insert(item.position, {});

		}

	}

	convexHull.addPoints(isolateLeafPoints(segmentedPointCloud));

	auto chull = convexHull.toMesh();

	for (auto& [original, projection] : targetPointsOnChullSurface) {

		// TODO: Could probably benefit from a spatial index of the triangles?
		projection = closestPointOnMesh(chull, original);

	}



	for (size_t i = target_points_before; i < targetPointsOnChullSurface.size(); i++) {
		visit_ordering.insert(i);
	}

	visit_ordering.iterate();

}

DynamicConvexHullAlgorithm::DynamicConvexHullAlgorithm(const moveit::core::RobotState &initial_state, const std::function<void(robot_trajectory::RobotTrajectory)> &trajectoryCallback)
		: OnlinePointCloudMotionControlAlgorithm(trajectoryCallback), convexHull(StreamingConvexHull::fromSpherifiedCube(3)),
		  targetPoints(0.05, 1000),
		  visit_ordering([this](size_t i) { return (targetPointsOnChullSurface[i].second - last_end_effector_position).norm(); },
						 [this](size_t i, size_t j) { return (targetPointsOnChullSurface[i].second - targetPointsOnChullSurface[j].second).norm(); }) {

	last_end_effector_position = initial_state.getGlobalLinkTransform("end_effector").translation();

}


const StreamingConvexHull &DynamicConvexHullAlgorithm::getConvexHull() const {
	return convexHull;
}

const std::vector<std::pair<const Eigen::Vector3d, Eigen::Vector3d>> &
DynamicConvexHullAlgorithm::getTargetPointsOnChullSurface() const {
	return targetPointsOnChullSurface;
}

const DynamicOrderOptimization<size_t> &DynamicConvexHullAlgorithm::getVisitOrdering() const {
	return visit_ordering;
}
