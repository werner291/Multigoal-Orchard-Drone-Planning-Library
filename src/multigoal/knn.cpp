//
// Created by werner on 30-09-21.
//

#include "knn.h"

#include <utility>
#include "../json_utils.h"
#include "PointToPointPlanner.h"

KNNPlanner::KNNPlanner(size_t k,
                       std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection,
                       std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection)
        : k(k), goalProjection_(std::move(goalProjection)), stateProjection_(std::move(stateProjection)) {}

MultiGoalPlanResult KNNPlanner::plan(const std::vector<GoalSamplerPtr> &goals,
                                     const ompl::base::State *start_state,
                                     PointToPointPlanner &point_to_point_planner) {

    // Place all apples into a Geometric Nearest-Neighbour access tree, using Euclidean distance.
    ompl::NearestNeighborsGNAT<GNATNode> unvisited_nn;
    unvisited_nn.setDistanceFunction([](const GNATNode &a, const GNATNode &b) {
        return (a.goal_pos - b.goal_pos).norm();
    });

    for (size_t idx = 0; idx < goals.size(); ++idx) {
        unvisited_nn.add({
                                 .goal = idx,
                                 .goal_pos = goalProjection_(goals[idx].get()),
                         });
    }

    MultiGoalPlanResult result;

    // Keep going until the GNAT is empty.
    while (unvisited_nn.size() > 0) {

        const ompl::base::State *segment_start_state = result.segments.empty() ? start_state
                                                                               : result.segments.back().path.getStates().back();

        const Eigen::Vector3d start_eepos = stateProjection_(segment_start_state);

        // Look up k targets closest to it.
        std::vector<GNATNode> knn;
        unvisited_nn.nearestK({.goal = 0 /* A bit hackish, just use 0 here. */, .goal_pos = start_eepos}, k, knn);

        // Keep track of the best planner result so far.
        std::optional<ompl::geometric::PathGeometric> bestResult;
        double best_length = INFINITY;
        GNATNode best_target;

        // Try to plan to each target.
        for (const auto &target: knn) {

            // Try planning such that the end-effector is near the given target/
            auto pointToPointResult = point_to_point_planner.planToOmplGoal(
                    MAX_TIME_PER_TARGET_SECONDS / (double) k,
                    segment_start_state,
                    goals[target.goal]);

            // If an improvement, store it.
            if (pointToPointResult.has_value() && pointToPointResult.value().length() < best_length) {
                best_length = pointToPointResult.value().length();
                bestResult = {pointToPointResult};
                best_target = target;
            };
        }

        // If at least one attempt was successful...
        if (bestResult.has_value()) {
            // Delete the target since we've reached it.
            unvisited_nn.remove(best_target);

            result.segments.push_back({
                                              best_target.goal,
                                              bestResult.value(),
                                      });

        } else {
            // Just delete the first of the k nearest neighbours.
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }
    }

    return result;
}

bool GNATNode::operator==(const GNATNode &other) const {
    return goal == other.goal && goal_pos == other.goal_pos;
}

bool GNATNode::operator!=(const GNATNode &other) const {
    return !(*this == other);
}
