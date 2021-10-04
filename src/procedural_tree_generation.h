#ifndef PLANNING_PROCEDURAL_TREE_GENERATION_H
#define PLANNING_PROCEDURAL_TREE_GENERATION_H

#include <cstddef>

const size_t NUM_LEAVES_PER_BRANCH = 20;

static const float BRANCH_ANGULAR_RANGE = 0.7;

static const float BRANCH_RADIUS_REDUCTION_FACTOR = 0.9;

#include <moveit_msgs/PlanningScene.h>
#include <Eigen/Geometry>
//#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <random>

/**
 * Represents a branch of the tree, within a hierarchical context.
 */
struct TreeNode {
    Eigen::Isometry3d root_at{};
    std::vector<TreeNode> children{};
    double length{};
    double radius{};
};

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
//    std::unique_ptr<fcl::CollisionObject> collision_object;
};

/**
 * Generate the branching structure of the tree, including trunk radius, lengths, branching points, etc...
 *
 * The positive Y-axis is the "up" direction.
 *
 * @param root_at           The position within the parent frame of reference where this (sub)tree will be rooted.
 * @param branching_depth   How many times to let this function recurse. Tree size is exponential in this parameter.
 * @param root_radius       The radius of the trunk at the root of this (sub) tree.
 * @return                  TreeNode representing this (sub)tree.
 */
void make_tree_branches(const Eigen::Isometry3d &root_at,
                        unsigned int branching_depth,
                        double root_radius,
                        std::vector<DetachedTreeNode> &nodes);

/**
 *
 * Generate a number of apples in the tree, making sure that they do not overlap.
 *
 * @param flattened                 The value returned by the `flatten_tree` method.
// * @param tree_model_broadphase     The fcl broadphase datastructure to sue for collision checking.
 * @param NUMBER_OF_APPLES          How many apples to generate in the tree.
 * @param apple_radius              The size of the apples.
 * @return                          A vector of apples, containing their center point and CollisionObject
 */
std::vector<Apple> spawn_apples(const std::vector<DetachedTreeNode> &flattened,
//                                fcl::DynamicAABBTreeCollisionManager &tree_model_broadphase,
                                const size_t NUMBER_OF_APPLES,
                                const double apple_radius);

/**
 * Returns a transformation that represents a frame affixed to the surface of the tree branch,
 * Z-vector pointing outward, and Y-vector pointing down the branch.
 *
 * @param azimuth   The angular position on the branch, in radians.
 * @param treeNode  The DetachedTreeNode representing the tree branch.
 * @param t         How far along the branch we are, parameter between 0 and 1.
 * @return          The transform.
 */
Eigen::Isometry3d frame_on_branch(double azimuth, double t, const DetachedTreeNode &treeNode);

/**
 * Generate the geometry of the tree leaves.
 *
 * @param flattened     The value returned by the `flatten_tree` method.
 * @param eng           The random engine used.
 * @return              The vertices, where each chunk of 3 vertices is a triangle.
 */
std::vector<Eigen::Vector3d>
generateLeafVertices(std::vector<DetachedTreeNode> &flattened);

struct TreeScene {
    moveit_msgs::PlanningScene moveit_diff;
    std::vector<Apple> apples;
    std::vector<Eigen::Vector3d> leaf_vertices;
};

#endif //PLANNING_PROCEDURAL_TREE_GENERATION_H
