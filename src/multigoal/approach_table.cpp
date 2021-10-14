
#include "approach_table.h"

using namespace multigoal;
namespace ob = ompl::base;

GoalApproachTable multigoal::takeGoalSamples(const ompl::base::SpaceInformationPtr &si,
                                             const SampleableGoals &goals,
                                             int k) {

    // One inner vector for every goal
    GoalApproachTable goal_states(goals.size());

    for (size_t idx = 0; idx < goals.size(); ++idx) {
        const auto &goal = goals[idx];
        // Attempt to get a sample k times.
        for (int i = 0; i < k; i++) {
            // Allocate a scoped pointer, because RAII is way better than manual pointer management.
            ob::ScopedStatePtr state(new ompl::base::ScopedState(si));

            // Sample, and keep if valid.
            goal->sampleGoal(state->get());
            if (si->isValid(state->get())) {
                goal_states[idx].push_back(state);
            }
        }
        // FIXME what about empty results? Kick them out?
    }

    return goal_states;
}

void multigoal::keepBest(const ompl::base::OptimizationObjective &opt, GoalApproachTable &table, int keep_k) {

    // Treat every inner vector separately.
    for (auto &samples: table) {

        // Only remove if we have more than k.
        if (samples.size() > keep_k) {
            // Sort, better cost to the beginning of the vector.
            std::sort(samples.begin(), samples.end(), [&](const ob::ScopedStatePtr &a, const ob::ScopedStatePtr &b) {
                return opt.isCostBetterThan(opt.stateCost(a->get()), opt.stateCost(b->get()));
            });

            // Truncate off anything after k samples.
            samples.resize(keep_k);
        }
    }
}

std::vector<Visitation> multigoal::random_initial_order(const GoalApproachTable &goal_samples) {

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    std::vector<Visitation> best_solution(goal_samples.size()); // One for every goal
    for (size_t idx = 0; idx < goal_samples.size(); ++idx) {
        // Pick an approach at random.
        best_solution[idx] = {
                idx, std::uniform_int_distribution<size_t>(0, goal_samples[idx].size() - 1)(gen)
        };
    }
    // Randomize the order.
    std::shuffle(best_solution.begin(), best_solution.end(), gen);
    return best_solution;
}

std::unordered_set<size_t> find_missing_targets(const ATSolution &solution, const GoalApproachTable &goals) {
    std::unordered_set<size_t> missing_targets;
    for (size_t i = 0; i < goals.size(); i++) {
        missing_targets.insert(i);
    }
    for (const auto &segment: solution.getSegmentsConst()) {
        missing_targets.erase(segment.visitation.target_idx);
    }
    return missing_targets;
}

std::vector<GoalApproach> &ATSolution::getSegments() {
    return solution_;
}

void ATSolution::check_valid(const GoalApproachTable &table) const {

    // In case something weird happened that goals aren't sampled exactly...
    const double EPSILON = 1e-5;

    // Keep track of which ones are actually visited.
    std::unordered_set<size_t> targets_visited;

    for (size_t i = 0; i < solution_.size(); i++) {

        auto approach = solution_[i];

        // Paths must be non-empty; they should at least have the goal state in them.
        assert(approach.approach_path.getStateCount() > 0);

        targets_visited.insert(approach.visitation.target_idx);

        const ompl::base::State *last_in_path = approach.approach_path.getState(
                approach.approach_path.getStateCount() - 1);

        // Make sure the IDs point to a valid table entry.
        assert(table.size() > approach.visitation.target_idx);
        assert(table[approach.visitation.target_idx].size() > approach.visitation.approach_idx);

        // Last state in the path must match the entry in the approach table.
        assert(si_->distance(last_in_path,
                             table[approach.visitation.target_idx][approach.visitation.approach_idx]->get()) <
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

MultiGoalPlanResult ATSolution::toMultiGoalResult() {
    MultiGoalPlanResult result;

    for (const auto &item: solution_) {
        result.segments.push_back(
                {
                        item.visitation.target_idx,
                        item.approach_path
                });
    }

    return result;
}
