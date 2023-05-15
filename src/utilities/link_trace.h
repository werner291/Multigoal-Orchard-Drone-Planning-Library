// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-5-23.
//

#ifndef NEW_PLANNERS_LINK_TRACE_H
#define NEW_PLANNERS_LINK_TRACE_H

#include <vector>
#include <Eigen/Core>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include "../planners/MultiGoalPlanner.h"
#include "../RobotPath.h"

/**
 * @brief Compute the trace of a specific link along a given path.
 *
 * @param ss The state space representing the robot's configuration space.
 * @param robot The robot model.
 * @param path The path along which to compute the trace.
 * @param link_name The name of the link for which to compute the trace.
 *
 * @return The trace of the link along the path, represented as a vector of positions.
 */
std::vector<Eigen::Vector3d> computeLinkTrace(
		const ompl_interface::ModelBasedStateSpace& ss,
		const moveit::core::RobotModelPtr& robot,
		const MultiGoalPlanner::PlanResult& path,
		const std::string& link_name);

/**
 * @brief Compute the trace of a specific link along a given path.
 *
 * @param path The path along which to compute the trace.
 * @param link_name The name of the link for which to compute the trace.
 *
 * @return The trace of the link along the path, represented as a vector of positions.
 */
std::vector<Eigen::Vector3d> computeLinkTrace(
		const RobotPath& path,
		const std::string& link_name);

#endif //NEW_PLANNERS_LINK_TRACE_H
