
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

/**
 * @brief Adds a collision shape for a mesh with a specified color to a MoveIt planning scene message.
 *
 * @param planning_scene_message The MoveIt planning scene message to add the collision shape to.
 * @param rgb The color of the collision shape in RGB format as an Eigen Vector3f.
 * @param id The ID of the collision shape.
 * @param mesh The mesh to use as the collision shape.
 */
void addColoredMeshCollisionShape(moveit_msgs::msg::PlanningScene &planning_scene_message,
								  const Eigen::Vector3f &rgb,
								  const std::string &id,
								  const shape_msgs::msg::Mesh &mesh);

/**
 * @brief Serializes a ROS message and writes it to a binary file.
 *
 * @tparam Msg The type of the ROS message to serialize and save.
 * @param filename The name of the file to save the serialized message to.
 * @param msg The ROS message to serialize and save.
 */
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

/**
 * @brief Reads a ROS message from a binary file and deserializes it.
 *
 * @tparam Msg The type of the ROS message to read and deserialize.
 * @param filename The name of the file to read the serialized message from.
 * @return An optional containing the deserialized ROS message, or std::nullopt if the file cannot be opened or the message cannot be deserialized.
 */
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

/**
 * @brief Converts a geometry_msgs Point message to an Eigen Vector3d.
 *
 * @param point The Point message to convert.
 * @return The equivalent Eigen Vector3d.
 */
Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &point);

/**
 * @brief Converts an Eigen Vector3d to a geometry_msgs Point message.
 *
 * @param v The Eigen Vector3d to convert.
 * @return The equivalent Point message.
 */
geometry_msgs::msg::Point msgFromEigen(const Eigen::Vector3d &v);

/**
 * @brief Converts an Eigen Quaterniond to a geometry_msgs Quaternion message.
 *
 * @param q The Eigen Quaterniond to convert.
 * @return The equivalent Quaternion message.
 */
geometry_msgs::msg::Quaternion msgFromEigen(const Eigen::Quaterniond& q);

#endif //NEW_PLANNERS_BUILD_REQUEST_H
