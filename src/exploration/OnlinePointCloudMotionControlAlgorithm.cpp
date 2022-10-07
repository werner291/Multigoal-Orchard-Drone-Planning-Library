
#include "OnlinePointCloudMotionControlAlgorithm.h"
#include "../utilities/msgs_utilities.h"

OnlinePointCloudMotionControlAlgorithm::OnlinePointCloudMotionControlAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback) : trajectoryCallback(std::move(trajectoryCallback)) {
}

TargetBoundingBox computeTargetBoundingBox(const SimplifiedOrchard &orchard) {

	Eigen::AlignedBox3d boundingBox;

	for (const auto &tree: orchard.trees) {
		for (const auto &item: tree.second.fruit_mesh.vertices) {
			boundingBox.extend(toEigen(item) + Eigen::Vector3d(tree.first.x(), tree.first.y(), 0.0));
		}
	}

	return { boundingBox };
}
