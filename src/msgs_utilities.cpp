#include "../src/experiment_utils.h"
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <gtest/gtest.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <fcl/fcl.h>
#include "ompl_custom.h"
#include "InverseClearanceIntegralObjective.h"
#include <moveit/robot_state/conversions.h>
#include "msgs_utilities.h"
#include "multigoal/approach_table.h"


geometry_msgs::Point pointMsg(const Eigen::Vector3d &ee_pt) {
    geometry_msgs::Point pt;
    pt.x = ee_pt.x();
    pt.y = ee_pt.y();
    pt.z = ee_pt.z();
    return pt;
}

geometry_msgs::Vector3 eigenVectorMsg(const Eigen::Vector3d &ee_pt) {
    geometry_msgs::Vector3 v3;
    v3.x = ee_pt.x();
    v3.y = ee_pt.y();
    v3.z = ee_pt.z();
    return v3;
}

geometry_msgs::Quaternion eigenQuaternionMsg(const Eigen::Quaterniond &q) {
    geometry_msgs::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

std_msgs::ColorRGBA colorMsgRGBA(const Eigen::Vector4f &ee_pt) {
    std_msgs::ColorRGBA msg;
    msg.r = ee_pt[0];
    msg.g = ee_pt[1];
    msg.b = ee_pt[2];
    msg.a = ee_pt[3];
    return msg;
}


visualization_msgs::Marker
buildApproachTableVisualization(const moveit::core::RobotModelConstPtr &robot,
                                multigoal::GoalApproachTable &approach_table) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> color_range(0.0, 1.0);

    visualization_msgs::Marker mrk;
    mrk.header.frame_id = robot->getModelFrame();
    mrk.type = visualization_msgs::Marker::LINE_LIST;
    mrk.action = 0; // Add/Modify
    mrk.pose.orientation = eigenQuaternionMsg(Eigen::Quaterniond::Identity());
    mrk.pose.position = pointMsg(Eigen::Vector3d(0.0, 0.0, 0.0));
    mrk.scale = eigenVectorMsg(Eigen::Vector3d(0.02, 1.0, 1.0));

    for (const auto &target_approaches: approach_table) {

        Eigen::Vector3f rgb(abs(color_range(gen)), abs(color_range(gen)), abs(color_range(gen)));
        rgb.normalize();

        Eigen::Vector4f color(rgb.x(), rgb.y(), rgb.z(), 1.0);

        for (const ompl::base::ScopedStatePtr &state: target_approaches) {
            auto rs = std::make_shared<moveit::core::RobotState>(robot);
            rs->setVariablePositions(state->get()->as<DroneStateSpace::StateType>()->values);
            rs->update(true);

            mrk.points.push_back(pointMsg(rs->getGlobalLinkTransform("end_effector").translation()));
            mrk.points.push_back(pointMsg(rs->getGlobalLinkTransform("base_link").translation()));

            mrk.colors.push_back(colorMsgRGBA(color));
            mrk.colors.push_back(colorMsgRGBA(color));

        }
    }
    return mrk;
}

shape_msgs::Mesh meshMsgFromResource(const std::string &resource) {
    std::shared_ptr<shapes::Mesh> mesh_shape(shapes::createMeshFromResource(
            resource));
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(mesh_shape.get(), mesh_msg);
    return boost::get<shape_msgs::Mesh>(mesh_msg);
}

void addColoredMeshCollisionShape(moveit_msgs::PlanningScene &planning_scene_message, const Eigen::Vector3f &rgb,
                                  const std::string &id, const shape_msgs::Mesh &mesh) {

    moveit_msgs::CollisionObject tree_obj;
    tree_obj.id = id;
    tree_obj.header.frame_id = "world";
    tree_obj.meshes.push_back(mesh);
    tree_obj.pose.orientation.x = 0;
    tree_obj.pose.orientation.y = 0;
    tree_obj.pose.orientation.z = 0;
    tree_obj.pose.orientation.w = 1;

    planning_scene_message.world.collision_objects.push_back(tree_obj);

    moveit_msgs::ObjectColor oc;
    oc.id = id;
    oc.color.r = rgb[0];
    oc.color.g = rgb[1];
    oc.color.b = rgb[2];
    oc.color.a = 1.0;
    planning_scene_message.object_colors.push_back(oc);
}

moveit_msgs::DisplayTrajectory robotTrajectoryToDisplayTrajectory(const robot_trajectory::RobotTrajectory &moveit_trajectory) {
    moveit_msgs::DisplayTrajectory msg;
    msg.model_id = moveit_trajectory.getRobotModel()->getName();
    msg.trajectory.resize(1);
    moveit_trajectory.getRobotTrajectoryMsg(msg.trajectory[0]);
    moveit::core::robotStateToRobotStateMsg(moveit_trajectory.getFirstWayPoint(), msg.trajectory_start);
    return msg;
}
