
#ifndef NEW_PLANNERS_NEWKNNPLANNER_H
#define NEW_PLANNERS_NEWKNNPLANNER_H

#include <variant>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <json/value.h>
#include "NewMultiGoalPlanner.h"
#include "DistanceHeuristics.h"

class NewKnnPlanner : public NewMultiGoalPlanner {

    const size_t START_STATE_REF = SIZE_MAX;

    const std::shared_ptr<const OmplDistanceHeuristics> distance_heuristics_;

    size_t k;

    const std::shared_ptr<SingleGoalPlannerMethods> methods;

    typedef std::pair<size_t,const ompl::base::Goal*> GoalIdAndGoal;

    typedef std::variant<
            const ompl::base::State *,
            GoalIdAndGoal
    > StateOrGoal;

    ompl::NearestNeighborsGNAT<StateOrGoal>
    buildGNAT(const ompl::base::State *start, const std::vector<ompl::base::GoalPtr> &goals) const;

public:
    NewKnnPlanner(const std::shared_ptr<const OmplDistanceHeuristics> &distanceHeuristics,
                  size_t k,
                  const std::shared_ptr<SingleGoalPlannerMethods> methods);

    PlanResult plan(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                    const std::vector<ompl::base::GoalPtr> &goals,
                    const AppleTreePlanningScene &planning_scene) override;

    [[nodiscard]] Json::Value parameters() const override;

    std::string name() const override;
};


#endif //NEW_PLANNERS_NEWKNNPLANNER_H
