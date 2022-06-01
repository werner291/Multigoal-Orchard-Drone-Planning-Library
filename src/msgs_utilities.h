

#ifndef NEW_PLANNERS_BUILD_REQUEST_H
#define NEW_PLANNERS_BUILD_REQUEST_H

#include <moveit_msgs/msg/constraints.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/color_rgba.h>
#include "procedural_tree_generation.h"
#include "multigoal/approach_table.h"
#include <fstream>

geometry_msgs::msg::Point pointMsg(const Eigen::Vector3d &ee_pt);

std_msgs::msg::ColorRGBA colorMsgRGBA(const Eigen::Vector4f &ee_pt);

[[maybe_unused]]
visualization_msgs::msg::Marker
buildApproachTableVisualization(const moveit::core::RobotModelConstPtr &robot,
                                multigoal::GoalApproachTable &approach_table);

shape_msgs::msg::Mesh meshMsgFromResource(const std::string &resource);

void addColoredMeshCollisionShape(moveit_msgs::msg::PlanningScene &planning_scene_message,
                                  const Eigen::Vector3f &rgb,
                                  const std::string &id,
                                  const shape_msgs::msg::Mesh &mesh);

void save_ros_msg(const std::string& filename, const moveit_msgs::msg::PlanningScene& msg) {
    // Write to File
    std::ofstream ofs(filename, std::ios::out|std::ios::binary);

    rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;

    rclcpp::SerializedMessage serialized_msg_;

    serializer.serialize_message(&msg, &serialized_msg_);

    ofs.write((char*)serialized_msg_.get_rcl_serialized_message().buffer, serialized_msg_.get_rcl_serialized_message().buffer_length);

    ofs.close();
}


std::optional<moveit_msgs::msg::PlanningScene> read_ros_msg(const std::string& filename){
    // Read from File to msg_scan_
    std::ifstream ifs(filename, std::ios::in|std::ios::binary);

    if (!ifs.good()) {
        return std::nullopt;
    }

    moveit_msgs::msg::PlanningScene msg;

    static rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
    serializer.deserialize_message()

    ifs.close();

    return {msg};
}

#endif //NEW_PLANNERS_BUILD_REQUEST_H
