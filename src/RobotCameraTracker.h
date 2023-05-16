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
	 * @param state The initial robot state used to position the camera
	 */
	RobotCameraTracker(vtkCamera *camera,
					   const moveit::core::RobotState &initial_robot_state,
					   Eigen::AlignedBox3d sceneBounds);

	/**
	 * @brief Updates the camera position to track the robot state
	 * @param state The robot state to track
	 */
	void update(const moveit::core::RobotState &state);

private:
	vtkCamera *camera; ///< Pointer to the VTK camera to control
	Eigen::Vector3d previous_camera_position; ///< Current position of the robot in world coordinates
	Eigen::AlignedBox3d scene_bounds; ///< Bounding box of the most important scene objects
};


#endif //NEW_PLANNERS_ROBOTCAMERATRACKER_H
