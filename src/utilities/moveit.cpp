
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
