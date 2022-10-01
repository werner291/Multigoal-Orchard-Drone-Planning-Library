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
