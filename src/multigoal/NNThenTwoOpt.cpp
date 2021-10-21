//
// Created by werner on 20-10-21.
//

#include "NNThenTwoOpt.h"

#include <utility>

MultiGoalPlanResult NNThenTwoOpt::plan(const std::vector<GoalSamplerPtr> &goals,
                                       const ompl::base::State *start_state,
                                       PointToPointPlanner &point_to_point_planner) {

    auto solution = UnionKNNPlanner(1, goalProjection_, stateProjection_)
            .plan(goals, start_state, point_to_point_planner);

    auto ptc = ompl::base::timedPlannerTerminationCondition(10.0);

    while (!ptc) {

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

        size_t i = std::uniform_int_distribution<size_t>(0, goals.size() - 1)(gen);
        size_t j = std::uniform_int_distribution<size_t>(0, goals.size() - 2)(gen);
        if (j >= i) j += 1; // This *should* produce uniformly random numbers without replacement.
        else std::swap(i, j);

        assert(i < j && j < goals.size());

        trySwap(goals, solution, point_to_point_planner, i, j, start_state);

    }

    return solution;
}

std::string NNThenTwoOpt::getName() {
    return "NN+Prob2OPT";
}

NNThenTwoOpt::NNThenTwoOpt(GoalProjectionFn goalProjection, StateProjectionFn stateProjection)
        : goalProjection_(std::move(goalProjection)), stateProjection_(std::move(stateProjection)) {}

void NNThenTwoOpt::trySwap(const std::vector<GoalSamplerPtr> &goals, MultiGoalPlanResult &result,
                           PointToPointPlanner &planner, size_t i, size_t j,
                           const ompl::base::State *start_state) {

    std::cout << "Swapping " << i << ", " << j << std::endl;

    auto replacement_spec = result.replacements_for_swap(goals.size(), i, j);

    auto computed = computeNewPathSegments(
            start_state,
            planner,
            goals,
            result,
            replacement_spec
    );

    if (computed) {
        if (result.is_improvement(replacement_spec, *computed)) {
            double before = result.total_length();
            result.apply_replacements(replacement_spec, *computed);
            double after = result.total_length();

            std::cout << "Improved from " << before << " to " << after << std::endl;
            result.check_valid(goals, *planner.getPlanner()->getSpaceInformation());

        }
    }

}
