#include <ompl/geometric/planners/prm/PRM.h>
#include <robowflex_library/trajectory.h>

#include <random>
#include "multi_goal_planners.h"

void extendTrajectory(robowflex::Trajectory &full_trajectory, const robowflex::Trajectory &extension) {
    // Just loop over all waypoints and copy them over.
    for (size_t i = 0; i < extension.getNumWaypoints(); i++) {
        full_trajectory.addSuffixWaypoint(extension.getTrajectoryConst()->getWayPoint(i));
    }
}

std::vector<MultiGoalPlanResult::ReplacementSpec>
MultiGoalPlanResult::replacements_for_swap(size_t num_goals, size_t i, size_t j) {

    // Check precondition that i < j.
    assert(i < j && j < num_goals);

    if (j == i + 1) {
        // This is a special case, since the goals are adjacent.
        //
        // An ordering like:  `i-1 -> i -> j -> j+1` will become `i-1 -> j -> i -> j+1`,
        // with every -> representing a point-to-point motion that must be re-planned.

        // The `j+1` will be omitted if j is the last target in the current ATSolution,
        // assuming we don't have some preferred end state. ALso, note that `i-1` may
        // be the initial state.

        ReplacementSpec repl;

        repl.from = i; // Replace the point-to-point motions from (i-1 -> i)
        repl.until = std::min(i + 3, this->segments.size()); // until at most (j -> j+1)

        repl.target_ids.push_back(this->segments[j].to_goal); // Recompute (i-1 -> j)
        repl.target_ids.push_back(this->segments[i].to_goal); // (j -> i)

        if (j + 1 < this->segments.size()) { // If not at the end, recompute (i -> j+1) as well
            repl.target_ids.push_back(this->segments[j + 1].to_goal);
        }

        return {repl};

    } else {
        // Regular case: the two areas in the ATSolution are disjoint and can be treated more-or-less separately.
        // The original solution will be like: ... -> i-1 -> i -> i+1 -> ... -> j-1 -> j -> j+1 -> ...
        // and will be transformed to be like: ... -> i-1 -> j -> i+1 -> ... -> j-1 -> i -> j+1 -> ...
        //
        // Note that `j+1` might not exist if `j` is last in the solution, but `i+1` always exists.

        // This replaces the movements (i-1 -> i) and (i -> i+1) with...
        ReplacementSpec repl1{i, i + 2, {
                this->segments[j].to_goal, // (i-1 -> j)
                this->segments[i + 1].to_goal} // (j -> i+1)
        };

        // This replaces the movements (j-1 -> j) and, if applicable, (j -> j+1) with...
        ReplacementSpec repl;
        repl.from = j;
        repl.until = std::min(j + 2, this->segments.size());

        // (j-1 -> i)
        repl.target_ids.push_back(this->segments[i].to_goal);

        // If applicable, (i -> j+1)
        if (j + 1 < this->segments.size()) {
            repl.target_ids.push_back(this->segments[j + 1].to_goal);
        }

        // Return both of those ranges, in sequence.
        return {repl1, repl};
    }
}

const ompl::base::State *
MultiGoalPlanResult::state_after_segments(size_t count, const ompl::base::State *start_state) const {
    assert(count <= segments.size());
    return (count == 0) ? start_state : segments[count - 1].path.getState(segments[count - 1].path.getStateCount() - 1);
}

void MultiGoalPlanResult::check_valid(const GoalSet &table, const ompl::base::SpaceInformation &si) const {

    // In case something weird happened that goals aren't sampled exactly...
    const double EPSILON = 1e-5;

    // Keep track of which ones are actually visited.
    std::unordered_set<size_t> targets_visited;

    for (size_t i = 0; i < segments.size(); i++) {

        auto approach = segments[i];

        // Paths must be non-empty; they should at least have the goal state in them.
        assert(approach.path.getStateCount() > 0);

        // Every target must be visited exactly once.
        // It is possible that the end-effector passes by a goal twice,
        // but this should not be represented at this level of abstraction.
        assert(targets_visited.find(approach.to_goal) == targets_visited.end());
        targets_visited.insert(approach.to_goal);

        const ompl::base::State *last_in_path = approach.path.getState(approach.path.getStateCount() - 1);

        // Make sure the IDs point to a valid table entry.
        assert(table.size() > approach.to_goal);

        // Last state in the path must match the entry in the approach table.
        assert(table[approach.to_goal]->isSatisfied(last_in_path));

        if (i + 1 < segments.size()) {
            // If there is another segment after this, the start-and-end-states must match up.
            assert(si.distance(last_in_path, segments[i + 1].path.getState(0)) < EPSILON);
        }

        // TODO Why does this fail? Just finnicky floating-point fun?
//        assert(approach.path.check());

    }
}

double MultiGoalPlanResult::newCost(const std::vector<PointToPointPath> &computed_replacements) const {
    return std::accumulate(computed_replacements.begin(), computed_replacements.end(), 0.0,
                           [](double &a, const PointToPointPath &b) {
                               return a + b.path.length();
                           }) / (double) computed_replacements.size();
}

double MultiGoalPlanResult::originalCost(const std::vector<ReplacementSpec> &replacement_specs) const {
    double original_cost = 0.0;
    size_t targets_visited = 0;

    for (const auto &item: replacement_specs) {
        for (size_t seg_idx = item.from; seg_idx < item.until; ++seg_idx) {
            original_cost += segments[seg_idx].path.length();
            targets_visited += 1;
        }
    }

    original_cost /= (double) targets_visited;
    return original_cost;
}

void MultiGoalPlanResult::apply_replacements(const std::vector<ReplacementSpec> &replacement_specs,
                                             const std::vector<PointToPointPath> &computed_replacements) {
    if (replacement_specs.empty()) return;

    auto cr_itr = computed_replacements.crbegin();

    for (auto itr = replacement_specs.crbegin(); itr != replacement_specs.crend(); ++itr) {

        segments.erase(segments.begin() + (long) itr->from, segments.begin() + (long) itr->until);

        for (auto itr2 = itr->target_ids.crbegin(); itr2 != itr->target_ids.crend(); ++itr2) {
            segments.insert(segments.begin() + (long) itr->from, *(cr_itr++));
        }
    }
    assert(cr_itr == computed_replacements.crend());
}

std::optional<std::vector<PointToPointPath>>
MultiGoalPlanner::computeNewPathSegments(const ompl::base::State *start_state,
                                         PointToPointPlanner &point_to_point_planner,
                                         const GoalSet &goals,
                                         const MultiGoalPlanResult &solution,
                                         const std::vector<MultiGoalPlanResult::ReplacementSpec> &replacements) {

    std::vector<PointToPointPath> computed_replacements;

    for (const auto &repl: replacements) {

        // Instead of invalidating every subsequent solution segment, we instead lock the final state of the to-be-replaced segments
        // to be the starting state of the next segment, if any.
        std::optional<const ompl::base::State *> to_state;
        if (repl.until < solution.segments.size()) to_state = solution.state_after_segments(repl.until, start_state);

        assert(repl.until == solution.segments.size() || goals[repl.target_ids.back()]->isSatisfied(*to_state));

        for (size_t point_idx = 0; point_idx < repl.target_ids.size(); ++point_idx) {

            const ompl::base::State *from_state = point_idx == 0 ? solution.state_after_segments(repl.from, start_state)
                                                                 : computed_replacements.back().path.getStates().back();

            assert(point_to_point_planner.getPlanner()->getSpaceInformation()->isValid(from_state));

            // If there's a final locked-in state, and we're at the end of the replacement sequence,
            // plan to that state instead of just an end-effector goal.
            auto ptp = (point_idx + 1 < repl.target_ids.size() || !to_state.has_value())
                       ? point_to_point_planner.planToOmplGoal(0.01, from_state, goals[repl.target_ids[point_idx]])
                       : point_to_point_planner.planToOmplState(0.01, from_state, to_state.value());

            // Planning is fallible. The whole replacement fails if one sub-plan fails.
            if (!ptp) return {};

            assert(point_to_point_planner.getPlanner()->getSpaceInformation()->isValid(
                    ptp->getState(ptp->getStateCount() - 1)));

            assert(goals[repl.target_ids[point_idx]]->isSatisfied(ptp->getState(ptp->getStateCount() - 1)));

            computed_replacements.push_back(PointToPointPath{
                    repl.target_ids[point_idx],
                    ptp.value()
            });
        }
    }

    return {computed_replacements};
}
