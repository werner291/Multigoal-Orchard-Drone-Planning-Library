
#include "UnionGoalSampleableRegion.h"

double UnionGoalSampleableRegion::distanceGoal(const ompl::base::State *st) const {
    double distance = INFINITY;

    for (const auto &goal: goals) {
        distance = std::min(distance, goal->distanceGoal(st));
    }

    return distance;
}

void UnionGoalSampleableRegion::sampleGoal(ompl::base::State *st) const {

    ompl::RNG rng;

    for (size_t i = 0; i < goals.size(); i++) {

        const std::shared_ptr<const GoalSampleableRegion> &goalToTry = goals[next_goal];
        next_goal = (next_goal + 1) % goals.size();

        if (goalToTry->canSample()) {
            goalToTry->sampleGoal(st);
            return;
        }
    }

    OMPL_ERROR("UnionGoalSampleableRegion : No goals can sample.");

}

unsigned int UnionGoalSampleableRegion::maxSampleCount() const {
    unsigned long total = 0;

    for (const auto &goal: goals) {
        unsigned long new_total = total + goal->maxSampleCount();
        if (new_total < total) {
            // Check for overflow, since maxSampleCount on infinite goals is often INT_MAX or UINT_MAX.
            total = UINT_MAX;
        } else {
            total = new_total;
        }
    }

    return total;
}
