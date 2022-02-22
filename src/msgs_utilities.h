

#ifndef NEW_PLANNERS_BUILD_REQUEST_H
#define NEW_PLANNERS_BUILD_REQUEST_H

#include <moveit_msgs/Constraints.h>
#include "procedural_tree_generation.h"
#include "multigoal/approach_table.h"

geometry_msgs::Point pointMsg(const Eigen::Vector3d &ee_pt);

std_msgs::ColorRGBA colorMsgRGBA(const Eigen::Vector4f &ee_pt);

[[maybe_unused]]
visualization_msgs::Marker
buildApproachTableVisualization(const moveit::core::RobotModelConstPtr &robot,
                                multigoal::GoalApproachTable &approach_table);

shape_msgs::Mesh meshMsgFromResource(const std::string &resource);

void addColoredMeshCollisionShape(moveit_msgs::PlanningScene &planning_scene_message, const Eigen::Vector3f &rgb,
                                  const std::string &id, const shape_msgs::Mesh &mesh);

#endif //NEW_PLANNERS_BUILD_REQUEST_H
