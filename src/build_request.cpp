#include <robowflex_library/tf.h>
#include <moveit/robot_state/conversions.h>
#include <robowflex_library/geometry.h>
#include "build_request.h"

using namespace robowflex;

moveit_msgs::MotionPlanRequest
makeAppleReachRequest(const std::shared_ptr<robowflex::Robot> &drone,
                      std::vector<Apple> &apples,
                      const std::string& planner_id) {

    moveit_msgs::MotionPlanRequest request;

    request.planner_id = planner_id;
    request.group_name = "whole_body";

    request.workspace_parameters.min_corner.x = -20.0;
    request.workspace_parameters.min_corner.y = -20.0;
    request.workspace_parameters.min_corner.z = -20.0;
    request.workspace_parameters.max_corner.x = 20.0;
    request.workspace_parameters.max_corner.y = 20.0;
    request.workspace_parameters.max_corner.z = 20.0;

    request.goal_constraints.push_back(makeReachAppleGoalConstraints(apples));

    robot_state::RobotState start_state(drone->getModelConst());
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(drone->getModelConst()->getJointModelGroup("whole_body"),
                                       {-10.0, -10.0, 10.0, 0.0, 0.0, 0.0, 1.0, 0.5});

    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);

    return request;
}

moveit_msgs::Constraints makeReachAppleGoalConstraints(std::vector<Apple> &apples) {
    // setup a random engine
    std::default_random_engine rng(std::random_device{}());

    // setup a uniform distribution
    std::uniform_int_distribution<size_t> dis(0, apples.size() - 1);

    Apple apple = apples[dis(rng)];

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
            "end_effector", "world", iso, Geometry::makeSphere(0.25));
    goal_constraints.position_constraints.push_back(positionConstraint);
    return goal_constraints;
}
