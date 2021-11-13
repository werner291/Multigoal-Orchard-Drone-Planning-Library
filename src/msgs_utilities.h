

#ifndef NEW_PLANNERS_BUILD_REQUEST_H
#define NEW_PLANNERS_BUILD_REQUEST_H

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include "procedural_tree_generation.h"
#include "multigoal/approach_clustering.h"

geometry_msgs::Point pointMsg(const Eigen::Vector3d &ee_pt);

std_msgs::ColorRGBA colorMsgRGBA(const Eigen::Vector4f &ee_pt);

visualization_msgs::Marker
buildApproachTableVisualization(const moveit::core::RobotModelConstPtr &robot,
                                multigoal::GoalApproachTable &approach_table);

#endif //NEW_PLANNERS_BUILD_REQUEST_H
