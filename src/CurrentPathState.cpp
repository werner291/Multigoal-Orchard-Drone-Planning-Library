
#include "CurrentPathState.h"
#include "utilities/moveit.h"

CurrentPathState::CurrentPathState(const moveit::core::RobotState &initialState) : current_state(initialState) {}

const moveit::core::RobotState &CurrentPathState::getCurrentState() const {
	return current_state;
}

void CurrentPathState::newPath(const robot_trajectory::RobotTrajectory &trajectory) {
	currentTrajectory = trajectory;
	pathProgressT = 0.0;
}

void CurrentPathState::advance(double dt) {

	// TODO: Add some validation here to stop the robot from magically teleporting all over the place

	if (currentTrajectory) {
		pathProgressT += dt;
		if (pathProgressT > currentTrajectory->getWayPointDurationFromStart(currentTrajectory->getWayPointCount() - 1)) {
			currentTrajectory = std::nullopt;
		}
		setStateToTrajectoryPoint(current_state, pathProgressT, *currentTrajectory);
	}
}
