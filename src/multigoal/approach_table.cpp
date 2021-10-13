
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

void ATSolution::check_valid() const {

    const double EPSILON = 1e-5;

    std::unordered_set<size_t> targets_visited;

    for (size_t i = 0; i < solution_.size(); i++) {

        auto approach = solution_[i];

        targets_visited.insert(approach.goal_id);

        const ompl::base::State *last_in_path = approach.approach_path.getState(
                approach.approach_path.getStateCount() - 1);

        // Make sure the IDs point to a valid table entry.
        assert(approachTable_->size() > approach.goal_id);
        assert((*approachTable_)[approach.goal_id].size() > approach.approach_id);

        // Last state in the path must match the entry in the approach table.
        assert(si_->distance(last_in_path, (*approachTable_)[approach.goal_id][approach.approach_id]->get()) <
               EPSILON);

        if (i + 1 < solution_.size()) {
            // If there is another segment after this, the start-and-end-states must match up.
            assert(si_->distance(last_in_path,
                                 approach.approach_path.getState(
                                         solution_[i + 1].approach_path.getStateCount() - 1)) < EPSILON);
        }

        for (size_t motion_idx = 0; motion_idx + 1 < approach.approach_path.getStateCount(); ++motion_idx) {
            // Every state-to-state motion in the approach path must be valid.
            assert(si_->checkMotion(approach.approach_path.getState(motion_idx),
                                    approach.approach_path.getState(motion_idx + 1)));
        }

    }

    // Every target must be visited exactly once.
    // It is possible that the end-effector passes by a goal twice,
    // but this should not be represented at this level of abstraction.
    assert(targets_visited.size() == solution_.size());
}

std::optional<const ompl::base::State *> ATSolution::getLastState() const {
    if (solution_.empty()) {
        return {};
    } else {
        return {solution_.back().approach_path.getState(
                solution_.back().approach_path.getStateCount() - 1)};
    }
}

const std::vector<GoalApproach> &ATSolution::getSegmentsConst() const {
    return solution_;
}
