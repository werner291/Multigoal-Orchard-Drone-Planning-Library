

#ifndef NEW_PLANNERS_MOVEIT_CONVERSIONS_H
#define NEW_PLANNERS_MOVEIT_CONVERSIONS_H

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

ompl::geometric::PathGeometric omplPathFromMoveitTrajectory(const robot_trajectory::RobotTrajectory &trajectory,
                                                            const ompl::base::SpaceInformationPtr &si);

ompl::geometric::PathGeometric omplPathFromMoveitTrajectory(const std::vector<moveit::core::RobotState> & trajectory,
                                                            const ompl::base::SpaceInformationPtr &si);

robot_trajectory::RobotTrajectory
omplPathToRobotTrajectory(const ompl_interface::ModelBasedStateSpace &state_space,
                          const ompl::geometric::PathGeometric &result_path);

#endif //NEW_PLANNERS_MOVEIT_CONVERSIONS_H
