// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

/**
 * @file occlusion_models.h
 * @brief Header file for occlusion related functionality.
 */

#include <functional>
#include <moveit/robot_state/robot_state.h>
#include "procedural_tree_generation.h"

/**
 * @brief Typedef for function taking a RobotState and an Apple as parameters and returning a boolean.
 */
using CanSeeAppleFn = std::function<bool(const moveit::core::RobotState &state, const Apple &apple)>;

/**
 * @brief Function that checks the squared norm of the difference between robot's end effector position and apple's center.
 *
 * @param state The current state of the robot.
 * @param apple An object of type Apple.
 * @return Boolean value as per the condition.
 */
bool distance_occlusion(const moveit::core::RobotState &state, const Apple &apple);

/**
 * @brief Function that creates an occlusion model from a given mesh and returns a function for occlusion checking.
 *
 * @param mesh The mesh to be used for occlusion checking.
 * @param link_name The name of the link whose origin is the eye center for occlusion checking.
 * @return Returns a function of type CanSeeAppleFn.
 */
CanSeeAppleFn mesh_occludes_vision(const shape_msgs::msg::Mesh &mesh, const std::string& link_name);

/**
 * @brief Function that computes an alpha shape from a given mesh of leaves and passes it to mesh_occludes_vision.
 *
 * @param leaves_mesh The mesh object representing leaves.
 * @param link_name The name of the link whose origin is the eye center for occlusion checking.
 * @return Returns a function of type CanSeeAppleFn.
 */
CanSeeAppleFn leaves_alpha_shape_occludes_vision(shape_msgs::msg::Mesh leaves_mesh, const std::string& link_name);



/**
 * @brief A function that always returns true, signifying vision that is never occluded.
 *
 * @param state The current state of the robot.
 * @param apple An object of type Apple.
 * @return Boolean value true.
 */
bool omniscient_occlusion(const moveit::core::RobotState &state, const Apple &apple);

/**
 * @brief Creates a function that checks if an apple is visible according to both provided functions.
 *
 * @param fn1 A function of type CanSeeAppleFn to check visibility of the apple.
 * @param fn2 A function of type CanSeeAppleFn to check visibility of the apple.
 *
 * @return Returns a function of type CanSeeAppleFn.
 */
CanSeeAppleFn only_if_both(const CanSeeAppleFn &fn1, const CanSeeAppleFn &fn2);

/**
 * @brief Creates a function that checks if an apple is within the field of view of a specified link of the robot.
 *
 * @param fov Field of view in radians.
 * @param front The direction of the front of the field of view.
 * @param link_name The name of the robot's link to check the field of view from.
 *
 * @return Returns a function of type CanSeeAppleFn.
 */
CanSeeAppleFn in_angle(double fov, Eigen::Vector3d front, std::string link_name);
