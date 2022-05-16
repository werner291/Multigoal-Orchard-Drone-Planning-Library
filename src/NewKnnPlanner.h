
#ifndef NEW_PLANNERS_NEWKNNPLANNER_H
#define NEW_PLANNERS_NEWKNNPLANNER_H

#include <variant>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include "NewMultiGoalPlanner.h"
#include "DistanceHeuristics.h"

class NewKnnPlanner : public NewMultiGoalPlanner {

    const size_t START_STATE_REF = SIZE_MAX;

    const std::shared_ptr<OmplDistanceHeuristics> distance_heuristics_;

private:
    size_t k;

    typedef std::pair<size_t,const ompl::base::Goal*> GoalIdAndGoal;

    typedef std::variant<
            const ompl::base::State *,
            GoalIdAndGoal
    > StateOrGoal;

    ompl::NearestNeighborsGNAT<StateOrGoal>
    buildGNAT(const ompl::base::State *start, const std::vector<ompl::base::GoalPtr> &goals) const;

public:
    NewKnnPlanner(const std::shared_ptr<OmplDistanceHeuristics> &distanceHeuristics, size_t k);

    PlanResult plan(const ompl::base::SpaceInformationPtr& si,
                    const ompl::base::State* start,
                    const std::vector<ompl::base::GoalPtr> &goals,
                    StateToGoalFn plan_state_to_goal,
                    StateToStateFn plan_state_to_state) override;
};


#endif //NEW_PLANNERS_NEWKNNPLANNER_H
