#include <json/json.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <fcl/fcl.h>
#include "multigoal/approach_clustering.h"
#include "LeavesCollisionChecker.h"
#include "ompl_custom.h"
#include "multigoal/random_order.h"
#include "multigoal/uknn.h"
#include "multigoal/knn.h"
#include "multigoal/multi_goal_planners.h"
#include "BulletContinuousMotionValidator.h"
#include "InverseClearanceIntegralObjective.h"
#include "make_robot.h"
#include "build_planning_scene.h"
#include <robowflex_library/trajectory.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/util.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/tf.h>
#include <moveit/robot_state/conversions.h>
#include <robowflex_library/geometry.h>
#include "msgs_utilities.h"

using namespace robowflex;

Apple selectAppleNearCoG(std::vector<Apple> &apples) {// setup a random engine
    std::default_random_engine rng(std::random_device{}());

    // setup a uniform distribution
    std::uniform_int_distribution<size_t> dis(0, apples.size() - 1);

    Eigen::Vector3d cog(0, 0, 0);
    for (Apple &apple: apples) {
        cog += apple.center;
    }
    cog /= (double) apples.size();

    Apple apple = apples[dis(rng)];

    for (int i = 0; i < 4; i++) {
        Apple apple2 = apples[dis(rng)];

        if ((apple2.center - cog).norm() < (apple.center - cog).norm()) {
            apple = apple2;
        }
    }
    return apple;
}


moveit_msgs::Constraints makeReachAppleGoalConstraints(const Apple &apple) {

    Eigen::Vector3d ee_pos = apple.center;// + apple.branch_normal * 0.4;
    Eigen::Quaterniond ee_rot;
    ee_rot.setFromTwoVectors(Eigen::Vector3d(0.0, 0.0, 1.0),
                             Eigen::Vector3d(apple.branch_normal.x(),
                                             apple.branch_normal.z(),
                                             0.0));

    Eigen::Isometry3d iso;
    iso.setIdentity();
    iso.translate(ee_pos);
    iso.rotate(ee_rot);

    moveit_msgs::Constraints goal_constraints;
    moveit_msgs::PositionConstraint positionConstraint = TF::getPositionConstraint(
            "end_effector", "world", iso, Geometry::makeSphere(0.2));
    goal_constraints.position_constraints.push_back(positionConstraint);

    goal_constraints.name = "reach_for_apple";

    return goal_constraints;
}

moveit_msgs::WorkspaceParameters makeDefaultWorkspaceParameters() {
    moveit_msgs::WorkspaceParameters wp;

    wp.min_corner.x = -20.0;
    wp.min_corner.y = -20.0;
    wp.min_corner.z = 0.0;
    wp.max_corner.x = 20.0;
    wp.max_corner.y = 20.0;
    wp.max_corner.z = 20.0;
    return wp;
}

moveit_msgs::OrientationConstraint
mkUprightLinkConstraint(const std::string &frame_id, const std::string &link_name) {
    moveit_msgs::OrientationConstraint oc;
    oc.header.frame_id = frame_id;
    oc.orientation.x = 0.0;
    oc.orientation.y = 0.0;
    oc.orientation.z = 0.0;
    oc.orientation.w = 1.0;
    oc.link_name = link_name;
    oc.absolute_x_axis_tolerance = 1.0e-5;
    oc.absolute_y_axis_tolerance = 1.0e-5;
    oc.absolute_z_axis_tolerance = M_PI;
    oc.weight = 1.0;
    return oc;
}

robot_state::RobotState genStartState(const std::shared_ptr<robowflex::Robot> &drone) {
    robot_state::RobotState start_state(drone->getModelConst());
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(drone->getModelConst()->getJointModelGroup("whole_body"),
                                       {
                                               -10.0, -10.0, 10.0,
                                               0.0, 0.0, 0.0, 1.0,
                                               0.0, 0.0, 0.0, 0.0
                                       });
    start_state.update(true);
    return start_state;
}

moveit_msgs::MotionPlanRequest
makeAppleReachRequest(const std::shared_ptr<robowflex::Robot> &drone,
                      const std::string &planner_id,
                      double planning_time,
                      const Apple &apple,
                      const robot_state::RobotState &start_state) {

    moveit_msgs::MotionPlanRequest request;

    request.planner_id = planner_id;
    request.group_name = "whole_body";
    request.workspace_parameters = makeDefaultWorkspaceParameters();
    request.allowed_planning_time = planning_time;
    request.path_constraints.orientation_constraints.push_back(
            mkUprightLinkConstraint(drone->getModelConst()->getModelFrame(),
                                    "base_link"));

    request.goal_constraints.push_back(makeReachAppleGoalConstraints(apple));

    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);

    return request;
}


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
buildApproachTableVisualization(const std::shared_ptr<Robot> &drone, multigoal::GoalApproachTable &approach_table) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> color_range(0.0, 1.0);

    visualization_msgs::Marker mrk;
    mrk.header.frame_id = drone->getModelConst()->getModelFrame();
    mrk.type = visualization_msgs::Marker::LINE_LIST;
    mrk.action = 0; // Add/Modify
    mrk.pose.orientation = eigenQuaternionMsg(Eigen::Quaterniond::Identity());
    mrk.pose.position = pointMsg(Eigen::Vector3d(0.0, 0.0, 0.0));
    mrk.scale = eigenVectorMsg(Eigen::Vector3d(0.02, 1.0, 1.0));

    for (const auto &target_approaches: approach_table) {

        Eigen::Vector3d rgb(abs(color_range(gen)), abs(color_range(gen)), abs(color_range(gen)));
        rgb.normalize();

        Eigen::Vector4d color(rgb.x(), rgb.y(), rgb.z(), 1.0);

        for (const ompl::base::ScopedStatePtr &state: target_approaches) {
            auto rs = std::make_shared<moveit::core::RobotState>(drone->getModelConst());
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
