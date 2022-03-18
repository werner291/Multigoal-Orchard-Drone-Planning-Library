#ifndef NEW_PLANNERS_SPHERESHELL_H
#define NEW_PLANNERS_SPHERESHELL_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include "procedural_tree_generation.h"

class SphereShell {

    Eigen::Vector3d center;
    double radius;

public:
    SphereShell(Eigen::Vector3d center, double radius);

    moveit::core::RobotState state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) const;

    std::vector<moveit::core::RobotState>
    path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) const;

};

#endif //NEW_PLANNERS_SPHERESHELL_H
