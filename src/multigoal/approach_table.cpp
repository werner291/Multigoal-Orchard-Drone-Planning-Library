
#include "approach_table.h"

#include <utility>

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

    std::vector<Visitation> best_solution;
    best_solution.reserve(goal_samples.size()); // One for every goal
    for (size_t idx = 0; idx < goal_samples.size(); ++idx) {
        if (!goal_samples[idx].empty()) {
            // Pick an approach at random.
            best_solution.push_back({
                                            idx,
                                            std::uniform_int_distribution<size_t>(0, goal_samples[idx].size() - 1)(gen)
                                    });
        }
    }
    // Randomize the order.
    std::shuffle(best_solution.begin(), best_solution.end(), gen);
    return best_solution;
}

std::unordered_set<size_t> multigoal::find_missing_targets(const ATSolution &solution, const GoalApproachTable &goals) {

    // Initialize an empty set.
    std::unordered_set<size_t> missing_targets;

    // Insert the indices of the inner vectors into the set.
    for (size_t i = 0; i < goals.size(); i++) {
        missing_targets.insert(i);
    }

    // Remove all those visited according to the ATSolution.
    for (const auto &segment: solution.getSegmentsConst()) {
        missing_targets.erase(segment.visitation.target_idx);
    }

    return missing_targets;
}

void multigoal::check_replacements_validity(const std::vector<Replacement> &replacements) {
    for (size_t ridx = 0; ridx < replacements.size(); ridx++) {
        // The last segment must be later than the first.
        assert(replacements[ridx].from <= replacements[ridx].until);
        // Subsequent replacements must be strictly non-overlapping.
        if (ridx + 1 < replacements.size())
            assert(replacements[ridx].until <= replacements[ridx + 1].from);
    }
}

std::vector<Replacement>
multigoal::replacements_for_swap(const GoalApproachTable &goals,
                                 const multigoal::ATSolution &solution, size_t i,
                                 size_t j) {
    // Check precondition that i < j.
    assert(i < j);

    if (j == i + 1) {
        // This is a special case, since the goals are adjacent.
        //
        // An ordering like:  `i-1 -> i -> j -> j+1` will become `i-1 -> j -> i -> j+1`,
        // with every -> representing a point-to-point motion that must be re-planned.

        // The `j+1` will be omitted if j is the last target in the current ATSolution,
        // assuming we don't have some preferred end state. ALso, note that `i-1` may
        // be the initial state.

        Replacement repl;

        repl.from = i; // Replace the point-to-point motions from (i-1 -> i)
        repl.until = std::min(i + 3, solution.getSegmentsConst().size()); // until at most (j -> j+1)

        repl.visitations.push_back(solution.getSegmentsConst()[j].visitation); // Recompute (i-1 -> j)
        repl.visitations.push_back(solution.getSegmentsConst()[i].visitation); // (j -> i)

        if (j + 1 < solution.getSegmentsConst().size()) { // If not at the end, recompute (i -> j+1) as well
            repl.visitations.push_back(solution.getSegmentsConst()[j + 1].visitation);
        }

        return {repl};

    } else {
        // Regular case: the two areas in the ATSolution are disjoint and can be treated more-or-less separately.
        // The original solution will be like: ... -> i-1 -> i -> i+1 -> ... -> j-1 -> j -> j+1 -> ...
        // and will be transformed to be like: ... -> i-1 -> j -> i+1 -> ... -> j-1 -> i -> j+1 -> ...
        //
        // Note that `j+1` might not exist if `j` is last in the solution, but `i+1` always exists.

        // This replaces the movements (i-1 -> i) and (i -> i+1) with...
        Replacement repl1{i, i + 2, {
                solution.getSegmentsConst()[j].visitation, // (i-1 -> j)
                solution.getSegmentsConst()[i + 1].visitation} // (j -> i+1)
        };

        // This replaces the movements (j-1 -> j) and, if applicable, (j -> j+1) with...
        Replacement repl;
        repl.from = j;
        repl.until = std::min(j + 2, solution.getSegmentsConst().size());

        // (j-1 -> i)
        repl.visitations.push_back(solution.getSegmentsConst()[i].visitation);

        // If applicable, (i -> j+1)
        if (j + 1 < solution.getSegmentsConst().size()) {
            repl.visitations.push_back(solution.getSegmentsConst()[j + 1].visitation);
        }

        // Return both of those ranges, in sequence.
        return {repl1, repl};
    }
}

std::vector<Replacement>
multigoal::replacements_for_insertion(const GoalApproachTable &goals,
                                      const ATSolution &solution,
                                      size_t i,
                                      Visitation v) {
    /*
     * These replacements will transform a sequence of the form (i-1 -> i)
     * into something of the form (i-1 -> v -> i), where i is optional if
     * the solution has no ith visitation.
     */

    Replacement repl;
    repl.from = i; // Corresponds to the (-> i) movement, which needs to be replaced.
    repl.until = std::min(i + 1,
                          solution.getSegmentsConst().size()); // If at the end, we get from == until, signaling an empty range to be replaced.

    repl.visitations.push_back(v); // (i-1 -> v)
    if (i < solution.getSegmentsConst().size()) {
        repl.visitations.push_back(solution.getSegmentsConst()[i].visitation); // (v -> i), if i exists.
    }

    return {repl};
}

multigoal::ATSolution
multigoal::random_initial_solution(const PointToPointPlanner &point_to_point_planner, const GoalApproachTable &table,
                                   const ompl::base::State *&start_state) {

    ATSolution solution(point_to_point_planner.getPlanner()->getSpaceInformation());
    // Create an empty solution.
    // Visit goals in random order and with random approach.
    for (Visitation v: random_initial_order(table)) {
        auto ptp_result = point_to_point_planner.planToOmplState(MAX_TIME_PER_TARGET_SECONDS,
                                                                 solution.getLastState().value_or(start_state),
                                                                 table[v.target_idx][v.approach_idx]->get());
        // Drop any goals where planning failed.
        if (ptp_result) {
            solution.getSegments().push_back(GoalApproach{
                    v.target_idx, v.approach_idx, ptp_result.value()
            });
        }
    }

    return solution;
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

        // Every target must be visited exactly once.
        // It is possible that the end-effector passes by a goal twice,
        // but this should not be represented at this level of abstraction.
        assert(targets_visited.find(approach.visitation.target_idx) == targets_visited.end());
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
            assert(si_->distance(last_in_path, solution_[i + 1].approach_path.getState(0)) < EPSILON);
        }

//        assert(approach.approach_path.check());
//        for (size_t motion_idx = 0; motion_idx + 1 < approach.approach_path.getStateCount(); ++motion_idx) {
//            // Every state-to-state motion in the approach path must be valid.
//            assert(si_->checkMotion(approach.approach_path.getState(motion_idx),
//                                    approach.approach_path.getState(motion_idx + 1)));
//        } TODO: Look into this later maybe?

    }


}

std::optional<const ompl::base::State *> ATSolution::getLastState() const {
    if (solution_.empty()) {
        return {};
    } else {
        return {solution_.back().approach_path.getState(
                solution_.back().approach_path.getStateCount() - 1)};
    }
}

std::optional<std::vector<ompl::geometric::PathGeometric>>
multigoal::computeNewPathSegments(const ompl::base::State *start_state,
                                  PointToPointPlanner &point_to_point_planner,
                                  const multigoal::GoalApproachTable &table,
                                  const multigoal::ATSolution &solution,
                                  const std::vector<Replacement> &replacements) {

    std::vector<ompl::geometric::PathGeometric> computed_replacements;

    for (const auto &repl: replacements) {
        const ompl::base::State *from_state;

        from_state =
                repl.from == 0 ? start_state : table[solution.getSegmentsConst()[repl.from - 1].visitation.target_idx]
                [solution.getSegmentsConst()[repl.from - 1].visitation.approach_idx]->get();

        for (auto viz: repl.visitations) {
            ompl::base::State *goal = table[viz.target_idx][viz.approach_idx]->get();
            auto ptp = point_to_point_planner.planToOmplState(0.01, from_state, goal);

            if (!ptp) return {};

            computed_replacements.push_back(ptp.value());

            from_state = goal;
        }
    }
    return {computed_replacements};
}

void multigoal::ATSolution::try_swap(const ompl::base::State *start_state,
                                     PointToPointPlanner &point_to_point_planner,
                                     const GoalApproachTable &table,
                                     size_t i,
                                     size_t j) {
    try_replacements(start_state, point_to_point_planner, table, replacements_for_swap(table, *this, i, j));
}

void
multigoal::ATSolution::try_insert(const ompl::base::State *start_state, PointToPointPlanner &point_to_point_planner,
                                  const GoalApproachTable &table, size_t i, multigoal::Visitation v) {
    try_replacements(start_state, point_to_point_planner, table, replacements_for_insertion(table, *this, i, v));
}

void multigoal::ATSolution::try_replacements(const ompl::base::State *start_state,
                                             PointToPointPlanner &point_to_point_planner,
                                             const GoalApproachTable &table,
                                             const std::vector<Replacement> &replacements) { // Validity checking.

    check_replacements_validity(replacements);

    auto computed_replacements =
            computeNewPathSegments(start_state,
                                   point_to_point_planner,
                                   table,
                                   *this,
                                   replacements);

    if (computed_replacements) {
        if (is_improvement(replacements, computed_replacements.value())) {
            apply_replacements(replacements, computed_replacements.value());
        }
        check_valid(table);
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

ATSolution::ATSolution(ompl::base::SpaceInformationPtr si) : si_(std::move(si)) {}

void ATSolution::apply_replacements(const std::vector<Replacement> &replacement_specs,
                                    const std::vector<ompl::geometric::PathGeometric> &computed_replacements) {

    if (replacement_specs.empty()) return;

    auto cr_itr = computed_replacements.crbegin();

    for (auto itr = replacement_specs.crbegin(); itr != replacement_specs.crend(); ++itr) {

        solution_.erase(solution_.begin() + (long) itr->from,
                        solution_.begin() + (long) itr->until);

        long insert_pos = 0;

        for (auto itr2 = itr->visitations.crbegin(); itr2 != itr->visitations.crend(); ++itr2) {
            GoalApproach ga{
                    *itr2,
                    *(cr_itr++)// TODO: Is this slow? Perhaps use a unique_ptr, or use std::move with iterators somehow?
            };
            solution_.insert(solution_.begin() + (long) itr->from/* + (insert_pos++)*/, ga);
        }
    }
    assert(cr_itr == computed_replacements.crend());
}

bool ATSolution::is_improvement(const std::vector<Replacement> &replacement_specs,
                                const std::vector<ompl::geometric::PathGeometric> &computed_replacements) const {

    double old_cost = 0.0;
    size_t old_targets_visited = 0;

    for (const auto &rs: replacement_specs) {
        for (size_t idx = rs.from; idx < rs.until; ++idx) {
            old_cost += solution_[idx].approach_path.length(); // TODO Cache this, maybe?
            ++old_targets_visited;
        }
    }

    double new_cost = 0.0;
    size_t new_targets_visited = computed_replacements.size();

    for (const auto &item: computed_replacements) new_cost += item.length();

    return old_cost / (double) old_targets_visited > new_cost / (double) new_targets_visited;
}
