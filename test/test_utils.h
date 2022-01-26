
#ifndef NEW_PLANNERS_TEST_UTILS_H
#define NEW_PLANNERS_TEST_UTILS_H

#include "../src/ompl_custom.h"
#include "../src/procedural_tree_generation.h"
#include "../src/multigoal/ClusterTable.h"

/**
 * By default, MoveIt's random state generation doesn't randomize the translation
 * component of floating joints.
 *
 * Here, we give it a uniform translation in the [-100.0,100.0] range in every dimension.
 */
std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<moveit::core::RobotModel> &drone);

/**
 * Generate 100 DroneEndEffectorNearTarget goals, with targets distributed uniformly in a [-50,50] cube.
 */
std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> genGoals(const ompl::base::SpaceInformationPtr &si);

/**
 * Adds an axis-aligned box with center (0.0, 0.0, 0.0) and dimensions (0.1, 10.0, 200.0).
 *
 * It thus spans from (-0.05,-5.0,-100) to (0.05,5.0,100).
 */
void spawn_wall(moveit_msgs::PlanningScene& planning_scene_diff);

/**
 * Spawn apples in a roughly U-shaped pattern, wrapping around one edge of the wall from
 * spawn_wall(...). The optimal visitation order is thus identical to the returned order.
 */
std::vector<Apple> apples_around_wall();

/**
 * Dump the clusters to a file.
 */
void dump_clusters(const std::vector<std::vector<clustering::Cluster>> clusters,
                   const std::shared_ptr<DroneStateSpace> state_space);

#endif //NEW_PLANNERS_TEST_UTILS_H
