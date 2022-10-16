
#include "WorkspaceSpec.h"

moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone) {
	moveit::core::RobotState current_state(drone);
	current_state.setVariablePositions({
											   5.0, 0.0, 1.5,
											   0.0, 0.0, 0.0, 1.0,
											   0.0, 0.0, 0.0, 0.0
									   });
	current_state.update();
	return current_state;
}
