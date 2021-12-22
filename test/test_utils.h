
#ifndef NEW_PLANNERS_TEST_UTILS_H
#define NEW_PLANNERS_TEST_UTILS_H

#include "../src/ompl_custom.h"
#include "../src/procedural_tree_generation.h"

std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<moveit::core::RobotModel> &drone);

std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> genGoals(const ompl::base::SpaceInformationPtr &si);

void spawn_wall(moveit_msgs::PlanningScene& planning_scene_diff);

std::vector<Apple> apples_around_wall();

#endif //NEW_PLANNERS_TEST_UTILS_H
