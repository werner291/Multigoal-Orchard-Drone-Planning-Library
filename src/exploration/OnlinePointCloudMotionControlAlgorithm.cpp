
#include "OnlinePointCloudMotionControlAlgorithm.h"
#include "../utilities/msgs_utilities.h"

OnlinePointCloudMotionControlAlgorithm::OnlinePointCloudMotionControlAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback) : trajectoryCallback(std::move(trajectoryCallback)) {
}

