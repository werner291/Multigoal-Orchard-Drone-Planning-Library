
#include "approach_table.h"

using namespace multigoal;
namespace ob = ompl::base;

GoalApproachTable multigoal::takeGoalSamples(const ompl::base::SpaceInformationPtr &si,
                                             const SampleableGoals &goals,
                                             int k) {

    GoalApproachTable goal_states(goals.size());

    for (size_t idx = 0; idx < goals.size(); ++idx) {
        const auto &goal = goals[idx];
        for (int i = 0; i < k; i++) {
            ob::ScopedStatePtr state(new ompl::base::ScopedState(si));

            goal->sampleGoal(state->get());
            if (si->isValid(state->get())) {
                goal_states[idx].push_back(state);
            }
        }
    }

    return goal_states;
}

void keepBest(const ompl::base::OptimizationObjective &opt, GoalApproachTable &table, int keep_k) {

    for (auto &samples: table) {

        if (samples.size() > keep_k) {
            std::sort(samples.begin(), samples.end(), [&](const ob::ScopedStatePtr &a, const ob::ScopedStatePtr &b) {
                return opt.isCostBetterThan(opt.stateCost(a->get()), opt.stateCost(b->get()));
            });

            samples.resize(keep_k);

        }
    }
}

std::vector<Visitation> multigoal::random_initial_order(const GoalApproachTable &goal_samples) {

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    std::vector<Visitation> best_solution(goal_samples.size());
    for (size_t idx = 0; idx < goal_samples.size(); ++idx) {
        best_solution[idx] = {
                idx, std::uniform_int_distribution<size_t>(0, goal_samples[idx].size() - 1)(gen)
        };
    }
    std::shuffle(best_solution.begin(), best_solution.end(), gen);
    return best_solution;
}

std::vector<GoalApproach> &ATSolution::getSegments() {
    return solution_;
}
