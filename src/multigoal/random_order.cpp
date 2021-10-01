//
// Created by werner on 30-09-21.
//

#include "random_order.h"
#include "../json_utils.h"

std::string RandomPlanner::getName() {
    return "Random";
}

MultiGoalPlanResult RandomPlanner::plan(const TreeScene &apples,
                                        const moveit::core::RobotState &start_state,
                                        const robowflex::SceneConstPtr &scene,
                                        const robowflex::RobotConstPtr &robot,
                                        PointToPointPlanner &point_to_point_planner) {

    std::vector<Eigen::Vector3d> targets;

    for (const Apple &apple: apples.apples) {
        targets.push_back(apple.center);
    }

    std::shuffle(targets.begin(), targets.end(), std::mt19937(std::random_device()()));

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    Json::Value root;

    for (const auto &target: targets) {
        auto pointToPointPlanResult = point_to_point_planner.planPointToPoint(full_trajectory.getTrajectory()->getLastWayPoint(), target);
        root["segments"].append(makePointToPointJson(pointToPointPlanResult));
    }

    root["ordering"] = "random";

    return {full_trajectory, root};
}