
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

ompl::geometric::PathGeometric plan_probe_retreat_slide(const std::vector<Apple>& apples_in_order,
                                                        const ompl::base::State* initial_state,
                                                        const ompl::base::SpaceInformationPtr& si,
                                                        const std::function<void(const Apple& apple, ompl::base::State*)>& state_outside_tree,
                                                        const std::function<std::optional<ompl::geometric::PathGeometric>(ompl::base::State*, ompl::base::State*)>& plan_state_to_state,
                                                        const std::function<std::optional<ompl::geometric::PathGeometric>(ompl::base::State*, const Apple& apple)>& plan_state_to_apple);

std::vector<moveit::core::RobotState> sphericalInterpolatedPath(const moveit::core::RobotState& ra, const moveit::core::RobotState& rb, const Eigen::Vector3d& sphere_center);

#endif //NEW_PLANNERS_PROBE_RETREAT_MOVE_H
