//
// Created by werner on 30-09-21.
//

#include "random_order.h"
#include "../json_utils.h"

std::string RandomPlanner::getName() {
    return "Random";
}

MultiGoalPlanResult RandomPlanner::plan(const std::vector<GoalSamplerPtr> &goals,
                                        const ompl::base::State *start_state,
                                        PointToPointPlanner &point_to_point_planner,
                                        std::chrono::milliseconds time_budget) {

    std::vector<size_t> goals_visitation_order(goals.size());
    for (size_t i = 0; i < goals.size(); ++i) {
        goals_visitation_order[i] = i;
    }
    std::shuffle(goals_visitation_order.begin(), goals_visitation_order.end(), std::mt19937(std::random_device()()));

    MultiGoalPlanResult result;

    for (const auto next_goal: goals_visitation_order) {

        auto ptp_result = point_to_point_planner.planToOmplGoal(MAX_TIME_PER_TARGET_SECONDS,
                                                                result.segments.empty() ? start_state
                                                                                        : result.segments.back().path.getStates().back(),
                                                                goals[next_goal]);

        if (ptp_result.has_value()) {
            result.segments.push_back(
                    PointToPointPath{
                            next_goal,
                            ptp_result.value()
                    }
            );
        }
    }

    return result;
}