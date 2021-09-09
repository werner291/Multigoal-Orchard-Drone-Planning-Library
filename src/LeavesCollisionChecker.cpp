//
// Created by werner on 09-09-21.
//

#include <vector>
#include <Eigen/Core>
#include "LeavesCollisionChecker.h"

LeavesCollisionChecker::LeavesCollisionChecker(const std::vector<Eigen::Vector3d> &leaf_vertices) {
    leaves.beginModel();
    for (size_t i = 0; i < leaf_vertices.size(); i += 3) {
        leaves.addTriangle(
                leaf_vertices[i],
                leaf_vertices[i + 1],
                leaf_vertices[i + 2]
        );
    }
    leaves.endModel();
}

std::vector<size_t> LeavesCollisionChecker::checkLeafCollisions(const moveit::core::RobotState &state) {
    state.getRobotModel()->getLinkModelNamesWithCollisionGeometry()
}
