#include <moveit/robot_state/robot_state.h>
#include "moveit.h"

void setBaseTranslation(moveit::core::RobotState &st, const Eigen::Vector3d &offset) {
	st.setVariablePosition(0, offset.x());
	st.setVariablePosition(1, offset.y());
	st.setVariablePosition(2, offset.z());
}
