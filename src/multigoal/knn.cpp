//
// Created by werner on 30-09-21.
//

#include "knn.h"
#include "../json_utils.h"
#include "PointToPointPlanner.h"

KNNPlanner::KNNPlanner(size_t k) : k(k) {}

MultiGoalPlanResult KNNPlanner::plan(const TreeScene &apples,
                                     const moveit::core::RobotState &start_state,
                                     const robowflex::SceneConstPtr &scene,
                                     const robowflex::RobotConstPtr &robot,
                                     PointToPointPlanner &point_to_point_planner) {

    ompl::NearestNeighborsGNAT<Eigen::Vector3d> unvisited_nn;
    unvisited_nn.setDistanceFunction([](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        return (a - b).norm();
    });

    for (const Apple &apple: apples.apples) {
        unvisited_nn.add(apple.center);
    }

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    Json::Value root;

    while (unvisited_nn.size() > 0) {

        const Eigen::Vector3d start_eepos = full_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                "end_effector").translation();

        std::vector<Eigen::Vector3d> knn;
        unvisited_nn.nearestK(start_eepos, k, knn);

        std::optional<PointToPointPlanResult> bestResult;
        double best_length = INFINITY;
        Eigen::Vector3d best_target;

        for (const auto &target: knn) {

            auto pointToPointResult = point_to_point_planner.planPointToPoint(
                    full_trajectory.getTrajectory()->getLastWayPoint(), knn, MAX_TIME_PER_TARGET_SECONDS / (double) k);

            if (pointToPointResult.has_value() && pointToPointResult.value().solution_length < best_length) {
                bestResult = pointToPointResult;
                best_length = pointToPointResult.value().solution_length;
                best_target = target;
            }
        }

        if (bestResult.has_value()) {
            unvisited_nn.remove(best_target);
            extendTrajectory(full_trajectory, bestResult.value().point_to_point_trajectory);
            root["segments"].append(makePointToPointJson(bestResult));
        } else {
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }
    }

    std::ostringstream stringStream;
    stringStream << k;
    stringStream << "-NN";

    root["ordering"] = stringStream.str();

    return {full_trajectory, root};
}