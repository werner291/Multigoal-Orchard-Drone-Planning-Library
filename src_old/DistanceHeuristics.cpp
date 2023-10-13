#include "DistanceHeuristics.h"

#include <utility>

double EuclideanOmplDistanceHeuristics::state_to_goal(const ompl::base::State *a, const ompl::base::Goal *b) const {

	// Convert to a MoveIt state.
	moveit::core::RobotState sta(state_space_->getRobotModel());
	state_space_->copyToRobotState(sta, a);

	// Apply forward kinematics to get the end-effector position.
	Eigen::Vector3d end_effector_pos = sta.getGlobalLinkTransform("end_effector").translation();

	// Take the Euclidean distance.
	return (end_effector_pos - b->as<DroneEndEffectorNearTarget>()->getTarget()).norm();
}

double EuclideanOmplDistanceHeuristics::goal_to_goal(const ompl::base::Goal *a, const ompl::base::Goal *b) const {

	// Upcast the two goals to DroneEndEffectorNearTarget, get the target positions
	Eigen::Vector3d a_tgt = a->as<DroneEndEffectorNearTarget>()->getTarget();
	Eigen::Vector3d b_tgt = b->as<DroneEndEffectorNearTarget>()->getTarget();

	// Then take the Euclidean distance.
	return (a_tgt - b_tgt).norm();
}

EuclideanOmplDistanceHeuristics::EuclideanOmplDistanceHeuristics(std::shared_ptr<DroneStateSpace> stateSpace)
		: state_space_(std::move(stateSpace)) {
}

std::string EuclideanOmplDistanceHeuristics::name() const {
	return "Euclidean";
}

double GreatCircleOmplDistanceHeuristics::state_to_goal(const ompl::base::State *a, const ompl::base::Goal *b) const {

	// Convert to a MoveIt state.
	moveit::core::RobotState sta(state_space_->getRobotModel());
	state_space_->copyToRobotState(sta, a);

	// Apply forward kinematics to get the end-effector position.
	Eigen::Vector3d end_effector_pos = sta.getGlobalLinkTransform("end_effector").translation();

	// Extract the target position.
	Eigen::Vector3d b_tgt = b->as<DroneEndEffectorNearTarget>()->getTarget();

	// Hand off to GreatCircleMetric
	return gcm.measure(end_effector_pos, b_tgt);
}

double GreatCircleOmplDistanceHeuristics::goal_to_goal(const ompl::base::Goal *a, const ompl::base::Goal *b) const {

	// Extract the target positions.
	Eigen::Vector3d a_tgt = a->as<DroneEndEffectorNearTarget>()->getTarget();
	Eigen::Vector3d b_tgt = b->as<DroneEndEffectorNearTarget>()->getTarget();

	return gcm.measure(a_tgt, b_tgt);
}

GreatCircleOmplDistanceHeuristics::GreatCircleOmplDistanceHeuristics(GreatCircleMetric gcm,
																	 std::shared_ptr<DroneStateSpace> stateSpace) : gcm(
		std::move(gcm)), state_space_(std::move(stateSpace)) {
}

std::string GreatCircleOmplDistanceHeuristics::name() const {
	return "GreatCircle";
}
