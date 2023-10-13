
#include "trajectory_primitives.h"
#include "moveit.h"

robot_trajectory::RobotTrajectory turnInPlace(const moveit::core::RobotState &current_state, double angle) {

	moveit::core::RobotState targetState(current_state);

	Eigen::Quaterniond base_pose(targetState.getGlobalLinkTransform("base_link").rotation());
	Eigen::Quaterniond target_pose = base_pose * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ());

	setBaseOrientation(targetState, target_pose);

	robot_trajectory::RobotTrajectory trajectory(current_state.getRobotModel(), "whole_body");
	trajectory.addSuffixWayPoint(current_state, 0.0);
	trajectory.addSuffixWayPoint(targetState, current_state.distance(targetState));

	return trajectory;
}
