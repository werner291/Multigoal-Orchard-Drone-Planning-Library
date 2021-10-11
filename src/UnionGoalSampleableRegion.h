//
// Created by werner on 9/21/21.
//

#ifndef NEW_PLANNERS_UNIONGOALSAMPLEABLEREGION_H
#define NEW_PLANNERS_UNIONGOALSAMPLEABLEREGION_H

#include <ompl/base/goals/GoalSampleableRegion.h>

/**
 * Represents the union of all goal sampling regions.
 * Samples are drawn sequentially from each sub-region.
 */
class UnionGoalSampleableRegion : public ompl::base::GoalSampleableRegion {

    std::vector<std::shared_ptr<const GoalSampleableRegion>> goals;

    // sampleGoal really shouldn't be const... Oh well.
    mutable size_t next_goal = 0;

public:
    UnionGoalSampleableRegion(const ompl::base::SpaceInformationPtr &si,
                              std::vector<std::shared_ptr<const GoalSampleableRegion>> goals);

    double distanceGoal(const ompl::base::State *st) const override;

    void sampleGoal(ompl::base::State *st) const override;

    [[nodiscard]] unsigned int maxSampleCount() const override;

    /**
     * Checks which of the subgoals is satisfied by the state, if any.
     * @param st The state to check against.
     * @return Index of the first match, or none if no matches are found.
     */
    std::optional<size_t> whichSatisfied(ompl::base::State *st);

};

#endif //NEW_PLANNERS_UNIONGOALSAMPLEABLEREGION_H
