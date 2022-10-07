
#include "DynamicBoundingSphereAlgorithm.h"
#include "../utilities/trajectory_primitives.h"

void DynamicBoundingSphereAlgorithm::updatePointCloud(const moveit::core::RobotState &current_state,
													  const SegmentedPointCloud &segmentedPointCloud) {

	trajectoryCallback(std::move(turnInPlace(current_state, 0.1)));

}

bodies::BoundingSphere DynamicBoundingSphereAlgorithm::getBoundingSphere() {
	bodies::BoundingSphere boundingSphere;

	boundingSphere.center = Eigen::Vector3d(0, 0, 0);
	boundingSphere.radius = 0;

	return boundingSphere;
}

DynamicBoundingSphereAlgorithm::DynamicBoundingSphereAlgorithm(const std::function<void(robot_trajectory::RobotTrajectory)> &trajectoryCallback)
		: OnlinePointCloudMotionControlAlgorithm(trajectoryCallback) {
}
