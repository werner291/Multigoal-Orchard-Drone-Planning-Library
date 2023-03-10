// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-3-23.
//

#ifndef NEW_PLANNERS_ROBOTCAMERATRACKER_H
#define NEW_PLANNERS_ROBOTCAMERATRACKER_H

#include <moveit/robot_state/robot_state.h>
#include <vtkCamera.h>

/**
 * @brief Class that handles tracking a robot with a camera in VTK
 */
class RobotCameraTracker {
public:
	/**
	 * @brief Constructor
	 * @param camera Pointer to the VTK camera to control
	 * @param initial_robot_state The initial robot state used to position the camera
	 */
	RobotCameraTracker(vtkCamera *camera, const moveit::core::RobotState &initial_robot_state);

	/**
	 * @brief Updates the camera position to track the robot state
	 * @param state The robot state to track
	 */
	void update(const moveit::core::RobotState &state);

private:
	vtkCamera *camera; ///< Pointer to the VTK camera to control
	Eigen::Vector3d robot_position; ///< Current position of the robot in world coordinates
};


#endif //NEW_PLANNERS_ROBOTCAMERATRACKER_H
