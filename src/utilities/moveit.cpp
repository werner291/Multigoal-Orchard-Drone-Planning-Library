#include <moveit/robot_state/robot_state.h>
#include "moveit.h"

void setBaseTranslation(moveit::core::RobotState &st, const Eigen::Vector3d &offset) {
	st.setVariablePosition(0, offset.x());
	st.setVariablePosition(1, offset.y());
	st.setVariablePosition(2, offset.z());
}

void setBaseOrientation(moveit::core::RobotState &robotState, Eigen::Quaterniond &q) {
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
