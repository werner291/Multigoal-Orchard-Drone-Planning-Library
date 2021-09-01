

#ifndef NEW_PLANNERS_BUILD_REQUEST_H
#define NEW_PLANNERS_BUILD_REQUEST_H

#include <moveit_msgs/Constraints.h>
#include <robowflex_library/robot.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include "procedural_tree_generation.h"

moveit_msgs::Constraints makeReachAppleGoalConstraints(Apple &apple);

moveit_msgs::MotionPlanRequest
makeAppleReachRequest(const std::shared_ptr<robowflex::Robot> &drone, const std::string &planner_id,
                      double planning_time, const Apple &apple);

Apple selectAppleNearCoG(std::vector<Apple> &apples);

#endif //NEW_PLANNERS_BUILD_REQUEST_H
