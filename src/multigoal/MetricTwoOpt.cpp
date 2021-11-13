//
// Created by werner on 08-11-21.
//

#include "MetricTwoOpt.h"

#include <utility>
#include "goals_gnat.h"

MultiGoalPlanResult
MetricTwoOpt::plan(GoalSet &goals, const ompl::base::State *start_state, PointToPointPlanner &point_to_point_planner,
                   std::chrono::milliseconds time_budget) {

    auto start_time = std::chrono::steady_clock::now();

    // Place all apples into a Geometric Nearest-Neighbour access tree, using Euclidean distance.
    auto unvisited_nn = buildGoalGNAT(goals, goalProjection_);

    std::vector<size_t> goals_in_order;

    Eigen::Vector3d current_pos = stateProjection_(start_state);

    while (unvisited_nn.size() > 0) {
        auto nearest = unvisited_nn.nearest({0, current_pos});
        unvisited_nn.remove(nearest);
        goals_in_order.push_back(nearest.goal);
        current_pos = nearest.goal_pos;
    }

    assert(goals_in_order.size() >= 2);

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    std::chrono::milliseconds swapping_budget((long) ((double) time_budget.count() * swapping_budget_portion));

    while (std::chrono::steady_clock::now() < start_time + swapping_budget) {

        // Alternatively, we do this deterministically, or with some kind of "candidate" heuristic
        size_t i = std::uniform_int_distribution<size_t>(0, goals_in_order.size() - 1)(gen);
        size_t j = std::uniform_int_distribution<size_t>(0, goals_in_order.size() - 2)(gen);

        if (j >= i) j += 1; // This *should* produce uniformly random numbers without replacement.
        else std::swap(i, j);
        assert(i < j && j < goals_in_order.size());

        auto before_i = i == 0 ? stateProjection_(start_state) : goalProjection_(goals[goals_in_order[i - 1]].get());
        auto i_proj = goalProjection_(goals[goals_in_order[i]].get());
        auto j_proj = goalProjection_(goals[goals_in_order[j]].get());

        std::optional<Eigen::Vector3d> after_j{};

        if (j + 1 < goals_in_order.size()) after_j = {goalProjection_(goals[goals_in_order[j + 1]].get())};

        double oldCost = (before_i - i_proj).norm() + (after_j ? (j_proj - *after_j).norm() : 0.0);
        double newCost = (before_i - j_proj).norm() + (after_j ? (i_proj - *after_j).norm() : 0.0);

        if (i + 1 < j) {

            auto after_i = goalProjection_(goals[goals_in_order[i + 1]].get());
            auto before_j = goalProjection_(goals[goals_in_order[j - 1]].get());

            oldCost += (i_proj - after_i).norm() + (j_proj - before_j).norm();
            newCost += (j_proj - after_i).norm() + (i_proj - before_j).norm();
        }

        if (oldCost > newCost) {
            std::swap(goals_in_order[i], goals_in_order[j]);
            std::cout << "Swap " << i << " " << j << std::endl;
        }
    }

    MultiGoalPlanResult result;

    for (const auto &item: goals_in_order) {

//        std::cout << "Plan to :" << goalProjection_(goals[item].get()) << std::endl;

        auto ptp_result = point_to_point_planner.planToOmplGoal(
                (1.0 - swapping_budget_portion) * (double) time_budget.count() /
                ((double) goals_in_order.size() * 1000.0),
                result.state_after_segments(result.segments.size(), start_state),
                goals[item]
        );

        if (ptp_result) {
            result.segments.push_back(PointToPointPath{item, *ptp_result});
        }
    }

    return result;
}

std::string MetricTwoOpt::getName() {
    std::stringstream ss;
    ss << "M-NN+2Opt" << swapping_budget_portion;
    return ss.str();
}

MetricTwoOpt::MetricTwoOpt(std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection,
                           std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection,
                           double swappingBudgetPortion)
        : goalProjection_(std::move(goalProjection)), stateProjection_(std::move(stateProjection)),
          swapping_budget_portion(swappingBudgetPortion) {}
