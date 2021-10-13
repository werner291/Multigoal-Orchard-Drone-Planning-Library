
#include "AT2Opt.h"
#include "multi_goal_planners.h"
#include "approach_table.h"

using namespace multigoal;

MultiGoalPlanResult AT2Opt::plan(const std::vector<GoalSamplerPtr> &goals,
                                 const ompl::base::State *start_state,
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

    solution.check_valid();

    std::unordered_set<size_t> missing_targets;
    for (size_t i = 0; i < goals.size(); i++) {
        missing_targets.insert(i);
    }
    for (const auto &segment: solution.getSegmentsConst()) {
        missing_targets.erase(segment.goal_id);
    }

    std::default_random_engine generator;

    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(10.0);

    while (!ptc) {
        for (size_t i = 0; i < solution.getSegments().size(); i++) {
            for (size_t j = i + 1; j < solution.getSegments().size(); j++) {


                std::vector<Visitation> pathThroughTable;
                const ompl::base::State *new_path_start = solution.getSegments()[i].approach_path.getState(0);

                pathThroughTable.push_back(Visitation{
                        solution.getSegments()[j].goal_id,
                        solution.getSegments()[j].approach_id
                });

                if (j == i + 1) {

                    // TODO Very carefully check all the array indices.
                    std::optional<ompl::geometric::PathGeometric> ptp1 = point_to_point_planner.planToOmplState(0.1,
                                                                                                                new_path_start,
                                                                                                                table[solution.getSegments()[j].goal_id][solution.getSegments()[j].approach_id]->get());
                    if (!ptp1) continue;

                    std::optional<ompl::geometric::PathGeometric> ptp2 = point_to_point_planner.planToOmplState(0.1,
                                                                                                                ptp1.value().getStates().back(),
                                                                                                                table[solution.getSegments()[i].goal_id][solution.getSegments()[i].approach_id]->get());
                    if (!ptp2) continue;

                    std::optional<ompl::geometric::PathGeometric> ptp3;
                    if (i + 2 < solution.getSegments().size()) {
                        ptp3 = point_to_point_planner.planToOmplState(0.1,
                                                                      ptp2.value().getStates().back(),
                                                                      table[solution.getSegments()[i +
                                                                                                   2].goal_id][solution.getSegments()[
                                                                              i + 1].approach_id]->get());
                        if (!ptp3) continue;
                    }

                    if (solution.getSegments()[i].approach_path.length() +
                        solution.getSegments()[i + 1].approach_path.length() +
                        (ptp3.has_value() ? solution.getSegments()[i + 2].approach_path.length() : 0.0) >
                        ptp1->length() + ptp2->length() + (ptp3.has_value() ? ptp3->length() : 0.0)) {
                        std::swap(solution.getSegments()[i].goal_id, solution.getSegments()[j].goal_id);
                        std::swap(solution.getSegments()[j].approach_id, solution.getSegments()[i].approach_id);
                        solution.getSegments()[i].approach_path = ptp1.value();
                        solution.getSegments()[i + 1].approach_path = ptp2.value();
                        if (i + 2 < solution.getSegments().size())
                            solution.getSegments()[i + 3].approach_path = ptp3.value();
                    }

                } else {

                    std::optional<ompl::geometric::PathGeometric> ptp1 = point_to_point_planner.planToOmplState(0.1,
                                                                                                                new_path_start,
                                                                                                                table[solution.getSegments()[j].goal_id][solution.getSegments()[j].approach_id]->get());
                    if (!ptp1) continue;
                    std::optional<ompl::geometric::PathGeometric> ptp2 = point_to_point_planner.planToOmplState(0.1,
                                                                                                                ptp1.value().getStates().back(),
                                                                                                                table[solution.getSegments()[
                                                                                                                        i +
                                                                                                                        1].goal_id][solution.getSegments()[
                                                                                                                        i +
                                                                                                                        1].approach_id]->get());
                    if (!ptp2) continue;

                    const ompl::base::State *new_path_start_j = solution.getSegments()[i].approach_path.getState(0);

                    std::optional<ompl::geometric::PathGeometric> ptp3 = point_to_point_planner.planToOmplState(0.1,
                                                                                                                new_path_start_j,
                                                                                                                table[solution.getSegments()[i].goal_id][solution.getSegments()[i].approach_id]->get());
                    if (!ptp3) continue;
                    std::optional<ompl::geometric::PathGeometric> ptp4;
                    if (j + 1 < solution.getSegments().size()) {
                        ptp4 = point_to_point_planner.planToOmplState(0.1, ptp3.value().getStates().back(),
                                                                      table[solution.getSegments()[j +
                                                                                                   1].goal_id][solution.getSegments()[
                                                                              j + 1].approach_id]->get());
                        if (!ptp4) continue;
                    }

                    if (solution.getSegments()[i].approach_path.length() +
                        solution.getSegments()[i + 1].approach_path.length() +
                        solution.getSegments()[j].approach_path.length() +
                        (ptp4 ? solution.getSegments()[j + 1].approach_path.length() : 0.0) >
                        ptp1->length() + ptp2->length() + ptp3->length() + (ptp4 ? ptp4->length() : 0.0)) {
                        std::swap(solution.getSegments()[i].goal_id, solution.getSegments()[j].goal_id);
                        std::swap(solution.getSegments()[j].approach_id, solution.getSegments()[i].approach_id);
                        solution.getSegments()[i].approach_path = ptp1.value();
                        solution.getSegments()[i + 1].approach_path = ptp2.value();
                        solution.getSegments()[j].approach_path = ptp3.value();
                        if (ptp4) solution.getSegments()[j + 1].approach_path = ptp4.value();
                    }
                }

                solution.check_valid();

            }

            // TODO: Try to insert one of the missing goals after i.
        }
    }

    return toMultiGoalResult(solution);

}

MultiGoalPlanResult AT2Opt::toMultiGoalResult(ATSolution &solution) {
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

std::string AT2Opt::getName() {
    return "AT2Opt";
}
