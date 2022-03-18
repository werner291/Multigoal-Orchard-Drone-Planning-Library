
#ifndef NEW_PLANNERS_PROBE_RETREAT_MOVE_H
#define NEW_PLANNERS_PROBE_RETREAT_MOVE_H

#include <Eigen/Core>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include "procedural_tree_generation.h"
#include "multigoal/PointToPointPlanner.h"

moveit::core::RobotState
state_outside_tree(const moveit::core::RobotModelPtr &drone, const Apple &a, const Eigen::Vector3d &sphere_center,
                   double sphere_radius);

std::vector<moveit::core::RobotState> sphericalInterpolatedPath(const moveit::core::RobotState& ra, const moveit::core::RobotState& rb, const Eigen::Vector3d& sphere_center);

#endif //NEW_PLANNERS_PROBE_RETREAT_MOVE_H
