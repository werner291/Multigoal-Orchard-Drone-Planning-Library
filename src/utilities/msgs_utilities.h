
#ifndef NEW_PLANNERS_BUILD_REQUEST_H
#define NEW_PLANNERS_BUILD_REQUEST_H

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <fstream>
#include <shape_msgs/msg/mesh.hpp>
#include <optional>

#include "../procedural_tree_generation.h"

void addColoredMeshCollisionShape(moveit_msgs::msg::PlanningScene &planning_scene_message,
                                  const Eigen::Vector3f &rgb,
                                  const std::string &id,
                                  const shape_msgs::msg::Mesh &mesh);

template<typename Msg>
void save_ros_msg(const std::string& filename, const Msg& msg) {
    // Write to File
    std::ofstream ofs(filename, std::ios::out|std::ios::binary);

    rclcpp::Serialization<Msg> serializer;

    rclcpp::SerializedMessage serialized_msg_;

    serializer.serialize_message(&msg, &serialized_msg_);

    ofs.write((char*)serialized_msg_.get_rcl_serialized_message().buffer, (long) serialized_msg_.get_rcl_serialized_message().buffer_length);

    ofs.close();
}

template<typename Msg>
std::optional<Msg> read_ros_msg(const std::string& filename){
    // Read from File to msg_scan_
    std::ifstream ifs(filename, std::ios::in|std::ios::binary);

    if (!ifs.good()) {
        return std::nullopt;
    }


    ifs.seekg(0, std::ios::end);
    std::streamsize size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    rclcpp::SerializedMessage serialized_msg_;
    serialized_msg_.reserve(size);
    serialized_msg_.get_rcl_serialized_message().buffer_length = size;

    ifs.read((char*)serialized_msg_.get_rcl_serialized_message().buffer, size);

    Msg msg;

    static rclcpp::Serialization<Msg> serializer;

    serializer.deserialize_message(&serialized_msg_, &msg);

    ifs.close();

    return {msg};
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &point);

geometry_msgs::msg::Point msgFromEigen(const Eigen::Vector3d &v);

geometry_msgs::msg::Quaternion msgFromEigen(const Eigen::Quaterniond& q);

#endif //NEW_PLANNERS_BUILD_REQUEST_H
