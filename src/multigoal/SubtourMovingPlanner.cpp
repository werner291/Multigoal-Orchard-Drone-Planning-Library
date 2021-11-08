
#include "SubtourMovingPlanner.h"

MultiGoalPlanResult multigoal::SubtourMovingPlanner::plan(GoalSet &goals,
                                                          const ompl::base::State *start_state,
                                                          PointToPointPlanner &point_to_point_planner,
                                                          std::chrono::milliseconds time_budget) {

    auto solution = initialAttemptPlanner_->plan(goals, start_state, point_to_point_planner, time_budget / 10);

    auto ptc = ompl::base::timedPlannerTerminationCondition(10.0);

    while (!ptc) {

        // First: select the range. This is just a random pair for now, but we'll want some kind of heuristic
        // based on segment lengths in the future.
        // Perhaps include a bias towards smaller ranges, too.
        // Alternatively, we start out with a target that is currently unvisited.

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

        size_t i = std::uniform_int_distribution<size_t>(0, solution.segments.size())(gen);
        size_t j = std::uniform_int_distribution<size_t>(0, solution.segments.size() - 1)(gen);
        if (j >= i) j += 1; // This *should* produce uniformly random numbers without replacement.
        else std::swap(i, j);

        assert(i < j && j < goals.size());

        size_t range_length = j - i;
        size_t remaining_after_removal = solution.segments.size() - range_length;
        size_t insertion_point = std::uniform_int_distribution<size_t>(0, remaining_after_removal)(gen);

        double original_cost = 0.0; // The sum of the costs of the segments that will no longer be in the solution.

        original_cost += solution.segments[i].path.length(); // FIXME these indices are probably wrong
        if (j + 1 < goals.size()) original_cost += solution.segments[j]

        auto skip_path = point_to_point_planner.planToOmplState(0.1, solution.state_after_segments(i, start_state),
                                                                solution.state_after_segments(j, start_state));
        if (!skip_path.has_value()) continue;

        auto reconnect_1 = point_to_point_planner.planToOmplGoal(0.1, solution.state_after_segments(insertion_point,
                                                                                                    start_state),
                                                                 solution.state_after_segments(j, start_state));
        if (!reconnect_1.has_value()) continue;

//        std::optional<ompl::geometric::PathGeometric> reconnect_2;
//        if (insertion_point<solution.segments.size()) {
//            reconnect_2 = point_to_point_planner.planToOmplGoal(0.1, solution.state_after_segments(j, start_state), solution.state_after_segments(insertion_point+1, start_state));
//            if (!reconnect_2.has_value()) continue;
//        }
//
//        for (int idx = i; idx < j; idx++) original_cost + solution.segments[idx].path.length();

//        ReplacementSpec repl {
//
//        };

        // We find an insertion point for the range, perhaps based on minimum heuristic path lengths.

        // We compute the paths necessary to fill the gap left by removing the range, and to reattach the range

        // If the total resultig length is better, we apply the changes.


    }

    return MultiGoalPlanResult();
}

std::string multigoal::SubtourMovingPlanner::getName() {
    return "Subtour Moving";
}
