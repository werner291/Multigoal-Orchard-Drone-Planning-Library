// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-5-23.
//

#ifndef NEW_PLANNERS_QUICKPLAN_H
#define NEW_PLANNERS_QUICKPLAN_H

#include "RobotPath.h"
#include "AppleTreePlanningScene.h"
#include "planner_allocators.h"

/**
 * @brief Quickly plans a path for a robot in a given scene from a start state to a set of apple locations.
 *
 * The name "quickPlan" refers to the function's aim to simplify the process of creating a path plan.
 * It does this by internally handling the necessary OMPL-related conversions and setting up the planner.
 * Thus, it provides a quick and easy way to generate a plan without needing to manually handle these steps.
 *
 * Note: this is probably not the most efficient way to plan a path, as it creates a new state space and space information
 * for each call. If you want to plan multiple paths, it is recommended to create the state space and space information
 *
 * @param scene The planning scene for the robot.
 * @param start_state The start state for the robot.
 * @param planner_allocator A function that allocates an OMPL planner for the given space information.
 *
 * @return A RobotPath representing the planned path.
 */
RobotPath quickPlan(AppleTreePlanningScene &scene,
					const moveit::core::RobotState &start_state,
					const StaticPlannerAllocatorFn &planner_allocator);

#endif //NEW_PLANNERS_QUICKPLAN_H
