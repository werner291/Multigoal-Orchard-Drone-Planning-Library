#include "StupidAlgorithm.h"
#include "../DroneStateConstraintSampler.h"
#include "../utilities/moveit.h"
#include "../utilities/trajectory_primitives.h"

StupidGoToFirstPointAlgorithm::StupidGoToFirstPointAlgorithm(std::function<void(robot_trajectory::RobotTrajectory)> trajectoryCallback)
		: OnlinePointCloudMotionControlAlgorithm(std::move(trajectoryCallback)) {
}



void StupidGoToFirstPointAlgorithm::update(const moveit::core::RobotState &current_state,
										   const SegmentedPointCloud &segmentedPointCloud) {

	if (!firstPointFound) {

		for (const auto &point : segmentedPointCloud.points) {

			if (point.type == SegmentedPointCloud::PT_TARGET) {

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
	}
}
