//
// Created by werner on 30-09-21.
//

#include "knn.h"

#include <utility>
#include "../json_utils.h"
#include "PointToPointPlanner.h"
#include "goals_gnat.h"

//double budgetFractionFn(size_t n, size_t i, double biasFactor) {
//
//    double t = (double) i / (double) n;
//
//    return (1.0/(double)n) * (biasFactor * t)
//
//}

KNNPlanner::KNNPlanner(size_t k, std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection,
                       std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection,
                       double budgetBiasFactor)
        : k(k), goalProjection_(std::move(goalProjection)), stateProjection_(std::move(stateProjection)),
          budgetBiasFactor(budgetBiasFactor) {}

MultiGoalPlanResult KNNPlanner::plan(const std::vector<GoalSamplerPtr> &goals,
                                     const ompl::base::State *start_state,
                                     PointToPointPlanner &point_to_point_planner,
                                     std::chrono::milliseconds time_budget) {


    auto start = std::chrono::steady_clock::now();
    auto deadline = start + time_budget;

    auto first_ptp_time = time_budget * budgetBiasFactor / (double) (goals.size() * k);
    auto other_ptp_time = (time_budget - first_ptp_time) / (double) ((goals.size() - 1) * k);

    assert(abs(first_ptp_time * k + (goals.size() - 1) * k * other_ptp_time - time_budget) <
           std::chrono::milliseconds(1));

    // Place all apples into a Geometric Nearest-Neighbour access tree, using Euclidean distance.
    auto unvisited_nn = buildGoalGNAT(goals, goalProjection_);

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

//        auto time_remaining = deadline - std::chrono::steady_clock::now();
//        auto expected_time_remaining = expected_time_per_ptp * (double)(unvisited_nn.size() * k);
//        auto delta = time_remaining - expected_time_remaining;
////        assert(time_remaining.count() > 0);
//        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms since start" << std::endl;
//        std::cout << "Delta: ms " << std::chrono::duration_cast<std::chrono::milliseconds>(delta).count() << std::endl;
//        std::cout << "Expected total: ms " << std::chrono::duration_cast<std::chrono::milliseconds>(expected_time_remaining).count() << std::endl;
//        std::cout << "Actual total: ms " << std::chrono::duration_cast<std::chrono::milliseconds>(time_remaining).count() << std::endl;
//        std::chrono::duration<double, std::ratio<1>> time_budget_per_ptp = (time_remaining+delta) / (double)(k*unvisited_nn.size());
//        if (time_budget_per_ptp < expected_time_remaining*0.5) time_budget_per_ptp = expected_time_remaining*0.5 / (double)(k*unvisited_nn.size());;
//        if (time_budget_per_ptp > expected_time_remaining*1.5) time_budget_per_ptp = expected_time_remaining*1.5 / (double)(k*unvisited_nn.size());;

        // Try to plan to each target.
        for (const auto &target: knn) {

            std::chrono::duration<double, std::ratio<1>> ptp_budget = result.segments.empty() ? first_ptp_time
                                                                                              : other_ptp_time;

            auto ptp_start = std::chrono::steady_clock::now();


//            std::cout << time_budget_per_ptp.count() << "/" << (k*unvisited_nn.size()) <<  std::endl;
            // Try planning such that the end-effector is near the given target/
            auto pointToPointResult = point_to_point_planner.planToOmplGoal(
                    ptp_budget.count(),
                    segment_start_state,
                    goals[target.goal]);

            auto ptp_end = std::chrono::steady_clock::now();

//            std::cout << "PTP took " << std::chrono::duration_cast<std::chrono::milliseconds>(ptp_end-ptp_start).count()
//                    << " ms (" << std::chrono::duration_cast<std::chrono::milliseconds>(ptp_budget).count() << "ms budget)" << std::endl;

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

    // TODO: Try occasionally re-computing the full path perhaps? Maybe after a successful swap?

    return result;
}


