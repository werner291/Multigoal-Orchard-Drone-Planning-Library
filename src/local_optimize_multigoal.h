
#ifndef NEW_PLANNERS_LOCAL_OPTIMIZE_MULTIGOAL_H
#define NEW_PLANNERS_LOCAL_OPTIMIZE_MULTIGOAL_H

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Goal.h>
#include <ompl/geometric/PathSimplifier.h>

void local_optimize_multigoal(ompl::geometric::PathGeometric& path,
                              std::vector<std::pair<size_t, ompl::base::GoalPtr>>& pinned_to_goals,
                              std::function<void(ompl::base::State*, ompl::base::Goal*)>& project_to_goal,
                              const ompl::base::OptimizationObjective& objective,
                              const ompl::base::SpaceInformationPtr& si) {

    ompl::RNG rng;
    auto sampler = si->allocStateSampler();

    std::vector<std::optional<ompl::base::GoalPtr>> state_pins(path.getStateCount(), std::nullopt);
    for (auto& [state_index, goal] : pinned_to_goals) {
        state_pins[state_index] = goal;
    }

    // Allocate some scoped states to work with
    ompl::base::ScopedState new_state(si);

    for (size_t i = 1; i < path.getStateCount(); ++i) {

        // Sample a new state near the current state
        sampler->sampleUniformNear(new_state.get(), path.getState(i), 0.1);

        // If it is a pinned state, apply the projection
        if (state_pins[i]) {
            project_to_goal(new_state.get(), state_pins[i].value().get());
        }

        // Compare the old and new path qualities for that section
        auto oldQuality = objective.combineCosts(
                objective.motionCost(path.getState(i - 1), path.getState(i)),
                ((i < path.getStateCount() - 1) ? objective.motionCost(path.getState(i), path.getState(i+1)) : objective.identityCost())
        )
        auto newQuality = objective.motionCost(path.getState(i - 1), new_state.get()) + (i < path.getStateCount() - 1) ? objective.motionCost(new_state.get(), path.getState(i+1)) : 0.0;
    }


}

#endif //NEW_PLANNERS_LOCAL_OPTIMIZE_MULTIGOAL_H
