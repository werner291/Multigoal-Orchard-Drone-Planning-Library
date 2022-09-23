
#ifndef NEW_PLANNERS_MOVEIT_H
#define NEW_PLANNERS_MOVEIT_H

#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>

void setBaseTranslation(moveit::core::RobotState &st, const Eigen::Vector3d &offset);

#endif //NEW_PLANNERS_MOVEIT_H
