//
// Created by werner on 30-09-21.
//

#include <ompl/base/goals/GoalSampleableRegion.h>
#include "uknn.h"
#include "../UnionGoalSampleableRegion.h"
#include "../json_utils.h"
#include "knn.h"

MultiGoalPlanResult UnionKNNPlanner::plan(const std::vector<GoalSamplerPtr> &goals,
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

        std::vector<std::shared_ptr<const ompl::base::GoalSampleableRegion>> knn_goals;
        for (size_t idx = 0; idx < knn.size(); ++idx) {
            knn_goals.push_back(goals[idx]);
        }
        auto union_goal = std::make_shared<UnionGoalSampleableRegion>(
                point_to_point_planner.getPlanner()->getSpaceInformation(), knn_goals);

        auto ptp_result = point_to_point_planner.planToOmplGoal(MAX_TIME_PER_TARGET_SECONDS, segment_start_state,
                                                                union_goal);

        // If at least one attempt was successful...
        if (ptp_result.has_value()) {

            size_t ith_nn = union_goal->whichSatisfied(
                    ptp_result.value().getStates().back()).value();

            // Delete the target since we've reached it.
            unvisited_nn.remove(knn[ith_nn]);

            result.segments.push_back({
                                              ith_nn,
                                              ptp_result.value(),
                                      });

        } else {
            // Just delete the first of the k nearest neighbours.
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }
    }

    return result;
}

UnionKNNPlanner::UnionKNNPlanner(size_t k, std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection,
                                 std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection)
        : k(k), goalProjection_(std::move(goalProjection)), stateProjection_(std::move(stateProjection)) {}
