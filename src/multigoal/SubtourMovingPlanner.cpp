
#include "SubtourMovingPlanner.h"

MultiGoalPlanResult multigoal::SubtourMovingPlanner::plan(GoalSet &goals,
                                                          const ompl::base::State *start_state,
                                                          PointToPointPlanner &point_to_point_planner,
                                                          std::chrono::milliseconds time_budget) {

    auto solution = initialAttemptPlanner_->plan(goals, start_state, point_to_point_planner, time_budget / 10);

    auto ptc = ompl::base::timedPlannerTerminationCondition(10.0);

    while (!ptc) {

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

        size_t i = std::uniform_int_distribution<size_t>(0, goals.size() - 1)(gen);
        size_t j = std::uniform_int_distribution<size_t>(0, goals.size() - 2)(gen);
        if (j >= i) j += 1; // This *should* produce uniformly random numbers without replacement.
        else std::swap(i, j);

        assert(i < j && j < goals.size());

//        trySwap(goals, solution, point_to_point_planner, i, j, start_state);

    }

    return MultiGoalPlanResult();
}

std::string multigoal::SubtourMovingPlanner::getName() {
    return "Subtour Moving";
}
