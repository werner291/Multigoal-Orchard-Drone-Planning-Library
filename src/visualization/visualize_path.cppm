module;
// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../planning/RobotPath.h"
#include "../planning/RobotState.h"
#include "robot_state.h"
#include "../planning/RobotModel.h"
#include "SimpleVtkViewer.h"
#include <optional>

export module visualize_path;

export namespace mgodpl::visualization {

	/**
	 * \brief Visualizes a robot path by interpolating states at regular intervals.
	 *
	 * This function takes a robot model, a viewer, and an optional robot path. It interpolates
	 * the robot states along the path at regular intervals and visualizes each state using the viewer.
	 *
	 * \param robot The robot model to visualize.
	 * \param viewer The viewer used to visualize the robot states.
	 * \param path The optional robot path to visualize.
	 */
	void visualize_path_by_interval(const robot_model::RobotModel &robot,
									SimpleVtkViewer &viewer,
									std::optional<mgodpl::RobotPath> &path) {
		// Initialize the path point
		PathPoint pt{0, 0.0};

		do {
			// Interpolate the robot state at the current path point
			RobotState st = interpolate(pt, *path);
			// Compute the forward kinematics for the interpolated state
			auto fk = robot_model::forwardKinematics(robot, st);
			// Visualize the robot state
			visualization::vizualize_robot_state(viewer, robot, fk, {1, 0, 1});

			// Advance the path point by a fixed interval, quitting if we reach the end of the path
		} while (!advancePathPointClamp(*path, pt, 0.1, equal_weights_distance));
	}
}