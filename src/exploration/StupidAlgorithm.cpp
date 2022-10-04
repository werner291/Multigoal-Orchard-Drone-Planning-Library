#include "StupidAlgorithm.h"
#include "../DroneStateConstraintSampler.h"
#include "../utilities/moveit.h"

StupidGoToFirstPointAlgorithm::StupidGoToFirstPointAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback)
		: OnlinePointCloudMotionControlAlgorithm(std::move(trajectoryCallback)) {
}

void StupidGoToFirstPointAlgorithm::updatePointCloud(const moveit::core::RobotState &current_state,
													 const SegmentedPointCloud &segmentedPointCloud) {

	if (!firstPointFound) {

		for (const auto &point : segmentedPointCloud.points) {

			if (point.type == PT_TARGET) {

				firstPointFound = true;

				robot_trajectory::RobotTrajectory trajectory(current_state.getRobotModel(), "whole_body");
				trajectory.addSuffixWayPoint(current_state, 0.0);

				moveit::core::RobotState targetState(current_state);
				moveEndEffectorToGoal(targetState, 0.0, point.position);
				trajectory.addSuffixWayPoint(targetState, current_state.distance(targetState));

				trajectoryCallback(std::move(trajectory));

				std::cout << "Found!" << std::endl;

				return;

			}
		}
		{
			// Rotate in place

			moveit::core::RobotState targetState(current_state);

			Eigen::Quaterniond base_pose(targetState.getGlobalLinkTransform("base_link").rotation());
			Eigen::Quaterniond target_pose = base_pose * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ());

			setBaseOrientation(targetState, target_pose);

			robot_trajectory::RobotTrajectory trajectory(current_state.getRobotModel(), "whole_body");
			trajectory.addSuffixWayPoint(current_state, 0.0);
			trajectory.addSuffixWayPoint(targetState, current_state.distance(targetState));

			trajectoryCallback(std::move(trajectory));
			return;
		}
	}
}
