
#ifndef NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H
#define NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Goal.h>
#include <ompl/base/Planner.h>

class SingleGoalPlannerMethods {

    const double timePerAppleSeconds;

    ompl::base::SpaceInformationPtr si;

    ompl::base::OptimizationObjectivePtr optimization_objective;

    ompl::base::PlannerAllocator alloc;

public:
    const ompl::base::OptimizationObjectivePtr &getOptimizationObjective() const;

    SingleGoalPlannerMethods(const double planTimePerAppleSeconds,
                             ompl::base::SpaceInformationPtr si,
                             ompl::base::OptimizationObjectivePtr optimizationObjective,
                             ompl::base::PlannerAllocator alloc)
            : timePerAppleSeconds(planTimePerAppleSeconds),
              si(std::move(si)),
              optimization_objective(optimizationObjective),
              alloc(alloc) {}

    std::optional<ompl::geometric::PathGeometric> state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b);

    std::optional<ompl::geometric::PathGeometric> state_to_state(const ompl::base::State *a, const ompl::base::State *b);

};

typedef std::function<std::shared_ptr<SingleGoalPlannerMethods>(ompl::base::SpaceInformationPtr,
                                                                ompl::base::OptimizationObjectivePtr,
                                                                ompl::base::PlannerAllocator)>
        SingleGoalPlannerMethodsAllocator;

#endif //NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H
