
#include "AT2Opt.h"
#include "multi_goal_planners.h"
#include "approach_table.h"

using namespace multigoal;


MultiGoalPlanResult AT2Opt::plan(const std::vector<GoalSamplerPtr> &goals,
                                 const ompl::base::State *start_state,
                                 PointToPointPlanner &point_to_point_planner) {

    // Build a goal approach table.
    GoalApproachTable table = takeGoalSamples(point_to_point_planner.getPlanner()->getSpaceInformation(), goals, 50);

    // Delete any but the five best samples. (TODO: This is rather naive, surely we could vary this, maybe?)
    keepBest(*point_to_point_planner.getOptimizationObjective(), table, 5);

    // Start with a randomized solution (TODO: Or nearest-neighbours maybe?)
    ATSolution solution = multigoal::random_initial_solution(point_to_point_planner, table, start_state);

    // Sanity check. TODO: Remove once the code works, maybe move to a test somewhere.
    solution.check_valid(table);

    // Keep track of missing targets. TODO Actually use this.
    std::unordered_set<size_t> missing_targets = find_missing_targets(solution, table);

    // Run until 10 seconds have passed (Maybe something about tracking convergence rates?)
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(10.0);

    while (!ptc) {
        for (size_t i = 0; i < solution.getSegments().size(); i++) {
            for (size_t j = i + 1; j < solution.getSegments().size(); j++) {

                std::vector<Replacement> replacements = replacements_for_swap(solution, i, j);

                // Validity checking.
                check_replacements_validity(replacements);

                auto computed_replacements =
                        multigoal::computeNewPathSegments(start_state,
                                                          point_to_point_planner,
                                                          table,
                                                          solution,
                                                          replacements);

                if (computed_replacements) {
                    double old_cost = 0.0;
                    double new_cost = 0.0;
                    for (const auto &cr: computed_replacements.value()) {
                        old_cost += solution.getSegments()[cr.index].approach_path.length(); // TODO Cache this, maybe?
                        new_cost += cr.ga.approach_path.length();
                    }

                    if (old_cost > new_cost) {
                        for (auto &cr: computed_replacements.value()) {
                            solution.getSegments()[cr.index] = cr.ga;
                        }
                    }

                    solution.check_valid(table);
                }

            }

            // TODO: Try to insert one of the missing goals after i.
        }
    }

    return solution.toMultiGoalResult();

}


std::string AT2Opt::getName() {
    return "AT2Opt";
}

AT2Opt::AT2Opt() {}
