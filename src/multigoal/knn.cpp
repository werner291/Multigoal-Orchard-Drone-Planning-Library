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

    // Place all apples into a Geometric Nearest-Neighbour access tree, using Euclidean distance.
    ompl::NearestNeighborsGNAT<Eigen::Vector3d> unvisited_nn;
    unvisited_nn.setDistanceFunction([](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        return (a - b).norm();
    });

    for (const Apple &apple: apples.apples) {
        unvisited_nn.add(apple.center);
    }

    // Initialize the final trajectory with just the start state/
    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    // For keeping track of statistics.
    Json::Value root;

    // Keep going until the GNAT is empty.
    while (unvisited_nn.size() > 0) {

        // Use forward kinematics to compute the end-effector position.
        const Eigen::Vector3d start_eepos = full_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                "end_effector").translation();

        // Look up k targets closest to it.
        std::vector<Eigen::Vector3d> knn;
        unvisited_nn.nearestK(start_eepos, k, knn);

        // Keep track of the best planner result so far.
        std::optional<PointToPointPlanResult> bestResult;
        double best_length = INFINITY;
        Eigen::Vector3d best_target;

        // Try to plan to each target.
        for (const auto &target: knn) {

            // Try planning such that the end-effector is near the given target/
            auto pointToPointResult = point_to_point_planner.planPointToPoint(
                    full_trajectory.getTrajectory()->getLastWayPoint(), knn, MAX_TIME_PER_TARGET_SECONDS / (double) k);

            // If an improvement, store it.
            if (pointToPointResult.has_value() && pointToPointResult.value().solution_length < best_length) {
                bestResult = pointToPointResult;
                best_length = pointToPointResult.value().solution_length;
                best_target = target;
            }
        }

        // If at least one attempt was successful...
        if (bestResult.has_value()) {
            // Delete the target since we've reached it.
            unvisited_nn.remove(best_target);
            // Extend the trajectory with the segment.
            extendTrajectory(full_trajectory, bestResult.value().point_to_point_trajectory);
            // Record the relevant statistics.
            root["segments"].append(makePointToPointJson(bestResult));
        } else {
            // Just delete the first of the k nearest neighbours.
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }
    }

    // Record the name of the ordering strategy.
    root["ordering"] = this->getName();

    return {full_trajectory, root};
}