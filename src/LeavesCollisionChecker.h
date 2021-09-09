//
// Created by werner on 09-09-21.
//

#ifndef NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H
#define NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H

#include <fcl/fcl.h>
#include <moveit/robot_state/robot_state.h>

class LeavesCollisionChecker {

    std::vector<size_t> checkLeafCollisions(const moveit::core::RobotState &state);

    fcl::BVHModel<fcl::OBBRSSf> leaves;

public:
    LeavesCollisionChecker(const std::vector<Eigen::Vector3d> &leaf_vertices);

};


#endif //NEW_PLANNERS_LEAVESCOLLISIONCHECKER_H
