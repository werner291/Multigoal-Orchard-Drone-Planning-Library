// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "visualize_path.h"
#include "VtkRobotModel.h"
#include "SimpleVtkViewer.h"
#include "../utilities/vtk.h"
#include "../utilities/moveit.h"

void quickVisualizePath(const TreeMeshes &meshes, const RobotPath &path) {
	// Convert the robot path to a constant speed robot trajectory.
	robot_trajectory::RobotTrajectory traj = robotPathToConstantSpeedRobotTrajectory(path, 1.0);

	// Create a VtkRobotmodel object to visualize the robot itself.
	VtkRobotmodel robotModel(path.waypoints.front().getRobotModel(), path.waypoints.front());

	SimpleVtkViewer viewer;

	// Add the robot model to the viewer.
	viewer.addActorCollection(robotModel.getLinkActors());

	// Add the tree meshes to the viewer.
	viewer.addActorCollection(buildTreeActors(meshes, false));

	double time = 0.0;

	// The "main loop" of the program, called every frame.
	auto callback = [&]() {
		time += 0.01;

		// Update the robot's visualization to match the current state.
		moveit::core::RobotState state(path.waypoints.front().getRobotModel());
		setStateToTrajectoryPoint(state, time, traj);
		robotModel.applyState(state);
	};

	viewer.addTimerCallback(callback);

	viewer.start();
}