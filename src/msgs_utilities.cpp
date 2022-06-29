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


geometry_msgs::msg::Vector3 eigenVectorMsg(const Eigen::Vector3d &ee_pt) {
    geometry_msgs::msg::Vector3 v3;
    v3.x = ee_pt.x();
    v3.y = ee_pt.y();
    v3.z = ee_pt.z();
    return v3;
}

geometry_msgs::msg::Quaternion eigenQuaternionMsg(const Eigen::Quaterniond &q) {
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}


shape_msgs::msg::Mesh meshMsgFromResource(const std::string &resource) {
    std::shared_ptr<shapes::Mesh> mesh_shape(shapes::createMeshFromResource(
            resource));
    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(mesh_shape.get(), mesh_msg);
    return boost::get<shape_msgs::msg::Mesh>(mesh_msg);
}

void addColoredMeshCollisionShape(moveit_msgs::msg::PlanningScene &planning_scene_message, const Eigen::Vector3f &rgb,
                                  const std::string &id, const shape_msgs::msg::Mesh &mesh) {

    moveit_msgs::msg::CollisionObject tree_obj;
    tree_obj.id = id;
    tree_obj.header.frame_id = "world";
    tree_obj.meshes.push_back(mesh);
    tree_obj.pose.orientation.x = 0;
    tree_obj.pose.orientation.y = 0;
    tree_obj.pose.orientation.z = 0;
    tree_obj.pose.orientation.w = 1;

    planning_scene_message.world.collision_objects.push_back(tree_obj);

    moveit_msgs::msg::ObjectColor oc;
    oc.id = id;
    oc.color.r = rgb[0];
    oc.color.g = rgb[1];
    oc.color.b = rgb[2];
    oc.color.a = 1.0;
    planning_scene_message.object_colors.push_back(oc);
}

