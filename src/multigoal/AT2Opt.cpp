
#include "AT2Opt.h"
#include "multi_goal_planners.h"
#include "approach_table.h"

using namespace multigoal;


MultiGoalPlanResult AT2Opt::plan(const std::vector<GoalSamplerPtr> &goals,
                                 const ompl::base::State *start_state,
                                 PointToPointPlanner &point_to_point_planner,
                                 std::chrono::milliseconds time_budget) {

    // Build a goal approach table.
    GoalApproachTable table = takeGoalSamples(point_to_point_planner.getPlanner()->getSpaceInformation(), goals, 50);

    // Delete any but the five best samples. (TODO: This is rather naive, surely we could vary this, maybe?)
    keepBest(*point_to_point_planner.getOptimizationObjective(), table, 5);

    // Start with a randomized solution (TODO: Or nearest-neighbours maybe?)
    ATSolution solution = multigoal::random_initial_solution(point_to_point_planner, table, start_state);

    // Sanity check. TODO: Remove once the code works, maybe move to a test somewhere.
    solution.check_valid(table);

    // Keep track of missing targets.
    std::unordered_set<size_t> missing_targets = find_missing_targets(solution, table);

    // Run until 10 seconds have passed (Maybe something about tracking convergence rates?)
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(20.0);

    // TODO: A heuristic to try: try swapping pairs that are already close by first, maybe with some probabilistic bias.
    // That, or maybe try it with a shorter point-to-point planning time.

    while (!ptc) {
        for (size_t i = 0; i < solution.getSegments().size(); i++) {
            for (size_t j = i + 1; j < solution.getSegments().size(); j++) {
                // TODO Approach changes
                solution.try_swap(start_state, point_to_point_planner, table, i, j);
            }
            if (ptc) break;
        }

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

        for (size_t i = 0; i < solution.getSegments().size() + 1; i++) {
            if (ptc) break;
            for (const auto &item: missing_targets) {
                size_t approaches = table[item].size();

                if (approaches > 0) {
                    solution.try_insert(
                            start_state, point_to_point_planner, table, i, Visitation{
                                    item,
                                    std::uniform_int_distribution<size_t>(0, approaches - 1)(gen)
                            }
                    );
                }
            }
            missing_targets = find_missing_targets(solution, table);
        }
    }

    return solution.toMultiGoalResult();

}


std::string AT2Opt::getName() {
    return "AT2Opt";
}

AT2Opt::AT2Opt() {}
