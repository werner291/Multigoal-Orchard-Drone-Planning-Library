
#ifndef NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H
#define NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Goal.h>

class SingleGoalPlannerMethods {

    const double timePerAppleSeconds;

    ompl::base::SpaceInformationPtr si;

    ompl::base::OptimizationObjectivePtr optimization_objective;
public:
    const ompl::base::OptimizationObjectivePtr &getOptimizationObjective() const;

public:

    SingleGoalPlannerMethods(const double planTimePerAppleSeconds, ompl::base::SpaceInformationPtr si,
                             ompl::base::OptimizationObjectivePtr optimizationObjective)
            : timePerAppleSeconds(planTimePerAppleSeconds), si(std::move(si)),
              optimization_objective(optimizationObjective) {}

    std::optional<ompl::geometric::PathGeometric> state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b);

    std::optional<ompl::geometric::PathGeometric> state_to_state(const ompl::base::State *a, const ompl::base::State *b);

};

#endif //NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H
