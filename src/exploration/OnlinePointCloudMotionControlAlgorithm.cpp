
#include "OnlinePointCloudMotionControlAlgorithm.h"

OnlinePointCloudMotionControlAlgorithm::OnlinePointCloudMotionControlAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback) : trajectoryCallback(std::move(trajectoryCallback)) {
}
