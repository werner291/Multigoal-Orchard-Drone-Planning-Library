

#ifndef NEW_PLANNERS_BUILD_REQUEST_H
#define NEW_PLANNERS_BUILD_REQUEST_H

#include <moveit_msgs/Constraints.h>
#include <robowflex_library/robot.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include "procedural_tree_generation.h"

moveit_msgs::Constraints makeReachAppleGoalConstraints(std::vector<Apple> &apples);

moveit_msgs::MotionPlanRequest makeAppleReachRequest(const std::shared_ptr<robowflex::Robot> &drone,
                                                     std::vector<Apple> &apples,
                                                     const std::string& planner_id);
#endif //NEW_PLANNERS_BUILD_REQUEST_H
