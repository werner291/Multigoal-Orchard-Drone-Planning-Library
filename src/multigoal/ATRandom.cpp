//
// Created by werner on 12-10-21.
//

#include "ATRandom.h"
#include "approach_table.h"

using namespace multigoal;

MultiGoalPlanResult ATRandom::plan(const std::vector<GoalSamplerPtr> &goals, const ompl::base::State *start_state,
                                   PointToPointPlanner &point_to_point_planner) {

    GoalApproachTable table = takeGoalSamples(point_to_point_planner.getPlanner()->getSpaceInformation(), goals, 50);

    keepBest(*point_to_point_planner.getOptimizationObjective(), table, 5);

    assert(goals.size() == table.size());

    ATSolution solution;

    for (Visitation v: random_initial_order(table)) {
        auto ptp_result = point_to_point_planner.planToOmplState(MAX_TIME_PER_TARGET_SECONDS,
                                                                 solution.getLastState().value_or(start_state),
                                                                 table[v.target_idx][v.approach_idx]->get());
        if (ptp_result) {
            solution.getSegments().push_back(GoalApproach{
                    v.target_idx, v.approach_idx, ptp_result.value()
            });
        }
    }

    return toMultiGoalResult(solution);

}

MultiGoalPlanResult ATRandom::toMultiGoalResult(ATSolution &solution) {
    MultiGoalPlanResult result;

    for (const auto &item: solution.getSegments()) {
        result.segments.push_back(
                {
                        item.goal_id,
                        item.approach_path
                });
    }

    return result;
}

std::string ATRandom::getName() {
    return "ATRandom";
}
