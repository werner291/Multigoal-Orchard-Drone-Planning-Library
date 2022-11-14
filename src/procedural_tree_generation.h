#ifndef PLANNING_PROCEDURAL_TREE_GENERATION_H
#define PLANNING_PROCEDURAL_TREE_GENERATION_H

#include <cstddef>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <Eigen/Geometry>
#include <random>

/**
 * Same as TreeNode, but the transform value is in world coordinates, and this structure does not own its children.
 */
struct DetachedTreeNode {
    Eigen::Isometry3d root_at_absolute;
    double length;
    double radius;
};

/**
 * Represents a single apple on the tree, hold the center of the apple,
 * and owns the CollisionObject.
 */
struct Apple {
    Eigen::Vector3d center;
    Eigen::Vector3d branch_normal;
};

struct TreeSceneData {
    std::vector<DetachedTreeNode> branches;
    std::vector<Apple> apples;
    std::vector<Eigen::Vector3d> leaf_vertices;
};

#endif //PLANNING_PROCEDURAL_TREE_GENERATION_H
