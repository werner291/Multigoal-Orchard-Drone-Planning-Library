//
// Created by werner on 05-10-21.
//

#include "TwoOpt.h"
#include "../json_utils.h"

MultiGoalPlanResult
TwoOpt::plan(const TreeScene &apples, const moveit::core::RobotState &start_state,
             const robowflex::SceneConstPtr &scene,
             const robowflex::RobotConstPtr &robot, PointToPointPlanner &point_to_point_planner) {

    std::vector<Eigen::Vector3d> targets;

    for (const Apple &apple: apples.apples) {
        targets.push_back(apple.center);
    }

    std::shuffle(targets.begin(), targets.end(), std::mt19937(std::random_device()()));

    auto full_trajectory = tryOrder(start_state, robot, point_to_point_planner, targets);

    std::chrono::steady_clock::time_point pre_solve = std::chrono::steady_clock::now();

    std::cout << "2opt start" << std::endl;

    while ((std::chrono::steady_clock::now() - pre_solve) < max_time) {
        std::cout << "2opt-iter" << std::endl;
        for (size_t i = 0; i < targets.size() - 1; i++) {
            for (size_t j = i + 1; j < targets.size(); j++) {

                if ((std::chrono::steady_clock::now() - pre_solve) > max_time) {
                    break;
                }

                std::swap(targets[i], targets[j]);

                auto candidate = tryOrder(start_state, robot, point_to_point_planner, targets);

                if (candidate.computeTotalLength() < full_trajectory.computeTotalLength()) {
                    full_trajectory = candidate;
                } else {
                    std::swap(targets[i], targets[j]);
                }

            }
        }
    }

    return full_trajectory;
}

MultiGoalPlanResult TwoOpt::tryOrder(const moveit::core::RobotState &start_state, const robowflex::RobotConstPtr &robot,
                                     PointToPointPlanner &point_to_point_planner,
                                     std::vector<Eigen::Vector3d> &targets) {
    MultiGoalPlanResult result;

    for (const auto &target: targets) {

        const auto segment_start_state = result.segments.empty() ? start_state
                                                                 : result.segments.back().point_to_point_trajectory.getTrajectory()->getLastWayPoint();

        auto ptp_result = point_to_point_planner.planPointToPoint(segment_start_state, target,
                                                                  MAX_TIME_PER_TARGET_SECONDS);
        if (ptp_result.has_value()) { result.segments.push_back(ptp_result.value()); }
    }

    return result;
}

std::string TwoOpt::getName() {
    return "2-Opt";
}

TwoOpt::TwoOpt(const std::chrono::duration<double> &maxTime) : max_time(maxTime) {}
