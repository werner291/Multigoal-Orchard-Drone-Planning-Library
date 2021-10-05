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

    MultiGoalPlanResult result;

    for (const auto &target: targets) {

        const auto segment_start_state = result.segments.empty() ? start_state
                                                                 : result.segments.back().point_to_point_trajectory.getTrajectory()->getLastWayPoint();

        auto ptp_result = point_to_point_planner.planPointToPoint(segment_start_state, target,
                                                                  MAX_TIME_PER_TARGET_SECONDS);

        if (ptp_result.has_value()) {
            result.segments.push_back(
                    ptp_result.value()
            );
        }

    }

    return result;
}