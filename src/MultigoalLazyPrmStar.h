
#ifndef NEW_PLANNERS_MULTIGOALLAZYPRMSTAR_H
#define NEW_PLANNERS_MULTIGOALLAZYPRMSTAR_H

#include "NewMultiGoalPlanner.h"

class MultigoalLazyPrmStar : public NewMultiGoalPlanner {

    double prm_build_time;
public:
    MultigoalLazyPrmStar(double prmBuildTime);

public:
    PlanResult plan(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *start,
                    const std::vector<ompl::base::GoalPtr> &goals) override;

    Json::Value parameters() const override;

    std::string name() const override;

};


#endif //NEW_PLANNERS_MULTIGOALLAZYPRMSTAR_H
