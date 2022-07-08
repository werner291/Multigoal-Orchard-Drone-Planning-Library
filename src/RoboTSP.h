//
// Created by werner on 8-7-22.
//

#ifndef NEW_PLANNERS_ROBOTSP_H
#define NEW_PLANNERS_ROBOTSP_H

// See https://arxiv.org/abs/1709.09343

#include "NewMultiGoalPlanner.h"
#include "DistanceHeuristics.h"

class RoboTSP : public NewMultiGoalPlanner {

    std::shared_ptr<OmplDistanceHeuristics> distance_heuristics;

    std::shared_ptr<SingleGoalPlannerMethods> methods;

    size_t k;

public:
    PlanResult plan(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                    const std::vector<ompl::base::GoalPtr> &goals) override;

    Json::Value parameters() const override;

    std::string name() const override;

};


#endif //NEW_PLANNERS_ROBOTSP_H
