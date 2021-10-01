//
// Created by werner on 30-09-21.
//

#include "random_order.h"
#include "../json_utils.h"

std::string RandomPlanner::getName() {
    return "Random";
}

MultiGoalPlanResult RandomPlanner::plan(const TreeScene &apples, const moveit::core::RobotState &start_state,
                                        const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                                        ompl::base::Planner &point_to_point_planner) {

    std::vector<Eigen::Vector3d> targets;

    for (const Apple &apple: apples.apples) {
        targets.push_back(apple.center);
    }

    std::shuffle(targets.begin(), targets.end(), std::mt19937(std::random_device()()));

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    Json::Value root;

    for (const auto &target: targets) {
        auto pointToPointPlanResult = planPointToPoint(robot,
                                                       point_to_point_planner,
                                                       std::make_shared<DroneEndEffectorNearTarget>(
                                                               point_to_point_planner.getSpaceInformation(),
                                                               0.2,
                                                               target),
                                                       full_trajectory.getTrajectory()->getLastWayPoint(),
                                                       MAX_TIME_PER_TARGET_SECONDS);


        root["segments"].append(makePointToPointJson(target, pointToPointPlanResult));
    }

    root["ordering"] = "random";

    return {full_trajectory, root};
}