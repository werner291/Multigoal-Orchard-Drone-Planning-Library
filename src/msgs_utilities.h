

#ifndef NEW_PLANNERS_BUILD_REQUEST_H
#define NEW_PLANNERS_BUILD_REQUEST_H

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "procedural_tree_generation.h"
#include "multigoal/approach_table.h"
#include <fstream>

geometry_msgs::Point pointMsg(const Eigen::Vector3d &ee_pt);

std_msgs::ColorRGBA colorMsgRGBA(const Eigen::Vector4f &ee_pt);

[[maybe_unused]]
visualization_msgs::Marker
buildApproachTableVisualization(const moveit::core::RobotModelConstPtr &robot,
                                multigoal::GoalApproachTable &approach_table);

shape_msgs::Mesh meshMsgFromResource(const std::string &resource);

void addColoredMeshCollisionShape(moveit_msgs::PlanningScene &planning_scene_message, const Eigen::Vector3f &rgb,
                                  const std::string &id, const shape_msgs::Mesh &mesh);

moveit_msgs::DisplayTrajectory robotTrajectoryToDisplayTrajectory(const robot_trajectory::RobotTrajectory &moveit_trajectory);

template<typename Msg>
void save_ros_msg(const std::string& filename, const Msg& msg) {
    // Write to File
    std::ofstream ofs(filename, std::ios::out|std::ios::binary);

    uint32_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, msg);
    ofs.write((char*) obuffer.get(), serial_size);
    ofs.close();
}

template<typename Msg>
std::optional<Msg> read_ros_msg(const std::string& filename){
    // Read from File to msg_scan_
    std::ifstream ifs(filename, std::ios::in|std::ios::binary);

    if (!ifs.good()) {
        return std::nullopt;
    }

    ifs.seekg (0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg (0, std::ios::beg);
    std::streampos begin = ifs.tellg();

    uint32_t file_size = end-begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read((char*) ibuffer.get(), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);

    Msg msg;

    ros::serialization::deserialize(istream, msg);
    ifs.close();

    return {msg};
}

#endif //NEW_PLANNERS_BUILD_REQUEST_H
