#include <ompl/util/RandomNumbers.h>
#include "moveit.h"

void setBaseTranslation(moveit::core::RobotState &st, const Eigen::Vector3d &offset) {
	st.setVariablePosition(0, offset.x());
	st.setVariablePosition(1, offset.y());
	st.setVariablePosition(2, offset.z());
}

void setBaseOrientation(moveit::core::RobotState &robotState, const Eigen::Quaterniond &q) {
	robotState.setVariablePosition(3, q.x());
	robotState.setVariablePosition(4, q.y());
	robotState.setVariablePosition(5, q.z());
	robotState.setVariablePosition(6, q.w());
}

void setStateToTrajectoryPoint(moveit::core::RobotState &state,
							   double t,
							   const robot_trajectory::RobotTrajectory &currentTrajectory) {

	// RobotTrajectory::getStateAtDurationFromStart annoyingly requires a shared_ptr,
	// so we make a fake shared pointer through the aliasing constructor of std::shared_ptr
	// to get a shared_ptr pointing to our robotstate without surrendering ownership.
	moveit::core::RobotStatePtr state_fakeshared(moveit::core::RobotStatePtr{}, &state);
	currentTrajectory.getStateAtDurationFromStart(t, state_fakeshared);
	state.update(true);

}

bool checkCollision(const moveit::core::RobotState &current_state,
					const collision_detection::CollisionEnvFCL &collision_env) {
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	collision_env.checkRobotCollision(collision_request, collision_result, current_state);

	bool collision = collision_result.collision;
	return collision;
}

moveit::core::RobotState setEndEffectorToPosition(moveit::core::RobotState state,
												  const Eigen::Vector3d &position,
												  const std::string &endEffectorName) {

	Eigen::Vector3d offset = position - state.getGlobalLinkTransform(endEffectorName).translation();

	state.setVariablePosition(0, state.getVariablePosition(0) + offset.x());
	state.setVariablePosition(1, state.getVariablePosition(1) + offset.y());
	state.setVariablePosition(2, state.getVariablePosition(2) + offset.z());

	state.update(true);

	return state;

}

Eigen::Quaterniond getBaseOrientation(const moveit::core::RobotState &robotState) {
	return Eigen::Quaterniond(robotState.getVariablePosition(6),
							  robotState.getVariablePosition(3),
							  robotState.getVariablePosition(4),
							  robotState.getVariablePosition(5));
}

Eigen::Vector3d getBaseTranslation(const moveit::core::RobotState &robotState) {
	return Eigen::Vector3d(robotState.getVariablePosition(0),
						   robotState.getVariablePosition(1),
						   robotState.getVariablePosition(2));
}

moveit::core::RobotState sampleStateNearByUpright(const moveit::core::RobotState &st, double distance) {

	ompl::RNG rng;

	moveit::core::RobotState out = st;

	// Sample a point uniformly in a sphere, then add to the translation of the reference state.
	std::vector<double> translation_delta(3);
	rng.uniformInBall(distance, translation_delta);
	// Also write it into the result variables.
	setBaseTranslation(out, getBaseTranslation(st) + Eigen::Vector3d(translation_delta[0], translation_delta[1], translation_delta[2]));

	// Extract the rotation from the current set of variables.
	// Add a random yaw-rotation to the reference rotation.
	// Write it into the result.
	setBaseOrientation(out, getBaseOrientation(st) * Eigen::AngleAxisd(rng.uniformReal(-distance, distance), Eigen::Vector3d::UnitZ()));

	// Add random angles to the arm joints.
	out.setVariablePosition(7, std::clamp(out.getVariablePosition(7) + rng.uniformReal(-distance, distance), -1.0, 1.0));
	out.setVariablePosition(8, std::clamp(out.getVariablePosition(8) + rng.uniformReal(-distance, distance), -1.0, 1.0));
	out.setVariablePosition(9, std::clamp(out.getVariablePosition(8) + rng.uniformReal(-distance, distance), -1.0, 1.0));
	out.setVariablePosition(10, out.getVariablePosition(10) + rng.uniformReal(-distance, distance));

	// Force-update
	out.update(true);

	return out;

}

RobotPastTrace::RobotPastTrace(size_t keep_count, const moveit::core::RobotState &initial_state)
		: keep_count(keep_count), last_robot_states({initial_state}) {
}

void RobotPastTrace::addRobotState(const moveit::core::RobotState &robot_state) {
	last_robot_states.push_back(robot_state);
	if (last_robot_states.size() > keep_count) {
		last_robot_states.erase(last_robot_states.begin());
	}
}

const moveit::core::RobotState &RobotPastTrace::lastRobotState() const {
	return last_robot_states.back();
}

const moveit::core::RobotState &RobotPastTrace::fromBack(size_t i) const {

	assert(i < last_robot_states.size());

	return last_robot_states[last_robot_states.size() - i - 1];
}

double RobotPastTrace::lastStepSize() const {
	return last_robot_states.size() >= 2 ? fromBack(0).distance(fromBack(1)) : 0.0;
}
