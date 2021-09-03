#include <robowflex_library/tf.h>
#include <moveit/robot_state/conversions.h>
#include <robowflex_library/geometry.h>
#include <cmath>
#include "msgs_utilities.h"

moveit_msgs::WorkspaceParameters getOrchardWorkspaceParameters();

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

moveit_msgs::OrientationConstraint makeUprightConstraint(const std::string &link_name, const std::string &frame_id) {
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

moveit_msgs::Constraints makeReachAppleGoalConstraints(const Apple &apple) {

    Eigen::Vector3d ee_pos = apple.center;
    Eigen::Quaterniond ee_rot;
    ee_rot.setFromTwoVectors(Eigen::Vector3d(0.0, 0.0, 1.0),
                             Eigen::Vector3d(apple.branch_normal.x(), apple.branch_normal.z(), 0.0));

    Eigen::Isometry3d iso;
    iso.setIdentity();
    iso.translate(ee_pos);
    iso.rotate(ee_rot);

    moveit_msgs::Constraints goal_constraints;
    moveit_msgs::PositionConstraint positionConstraint = TF::getPositionConstraint(
            "end_effector", GLOBAL_FRAME_ID, iso, Geometry::makeSphere(0.2));
    goal_constraints.position_constraints.push_back(positionConstraint);
    goal_constraints.name = "reach_for_apple";

    return goal_constraints;
}

moveit_msgs::MotionPlanRequest
makeAppleReachRequest(const std::shared_ptr<robowflex::Robot> &drone,
                      const std::string &planner_id,
                      double planning_time,
                      const Apple &apple) {

    moveit_msgs::MotionPlanRequest request;

    request.planner_id = planner_id;
    request.group_name = "whole_body";

    request.workspace_parameters = getOrchardWorkspaceParameters();

    request.allowed_planning_time = planning_time;

    moveit_msgs::OrientationConstraint oc = makeUprightConstraint("base_link", GLOBAL_FRAME_ID);

    request.path_constraints.orientation_constraints.push_back(oc);

    request.goal_constraints.push_back(makeReachAppleGoalConstraints(apple));

    robot_state::RobotState start_state(drone->getModelConst());
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(drone->getModelConst()->getJointModelGroup("whole_body"),
                                       {
                                               -10.0, -10.0, 10.0,
                                               0.0, 0.0, 0.0, 1.0,
                                               0.0, 0.0, 0.0, 0.0
                                       });

    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);

    return request;
}

moveit_msgs::WorkspaceParameters getOrchardWorkspaceParameters() {
    moveit_msgs::WorkspaceParameters wp;
    wp.min_corner.x = -20.0;
    wp.min_corner.y = -20.0;
    wp.min_corner.z = 0.0;
    wp.max_corner.x = 20.0;
    wp.max_corner.y = 20.0;
    wp.max_corner.z = 20.0;
    return wp;
}




