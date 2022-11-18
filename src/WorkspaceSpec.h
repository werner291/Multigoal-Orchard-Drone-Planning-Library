
#ifndef NEW_PLANNERS_WORKSPACESPEC_H
#define NEW_PLANNERS_WORKSPACESPEC_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "TreeMeshes.h"

/**
 * A simple workspace specification.
 */
struct WorkspaceSpec {
	/// Which robot to use.
	moveit::core::RobotModelConstPtr robotModel;
	/// The initial state of that robot
	const moveit::core::RobotState initialState;
	/// The orchard to plan in.
	const SimplifiedOrchard orchard;

};

moveit::core::RobotState mkInitialState(const moveit::core::RobotModelPtr &drone);

#endif //NEW_PLANNERS_WORKSPACESPEC_H
