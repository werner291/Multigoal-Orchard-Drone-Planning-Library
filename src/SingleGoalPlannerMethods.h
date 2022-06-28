
#ifndef NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H
#define NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Goal.h>
#include <ompl/base/Planner.h>
#include <jsoncpp/json/value.h>
#include <optional>
#include "InformedRobotStateSampler.h"

#include <ompl/geometric/planners/prm/PRM.h>

class AccessiblePRM : public ompl::geometric::PRM {

public:

    AccessiblePRM(const ompl::base::SpaceInformationPtr &si, bool starStrategy=false) : PRM(si, starStrategy) {}

    bool has_solution();

    double best_cost() const;
};

class SingleGoalPlannerMethods {

    const double timePerAppleSeconds;

    ompl::base::SpaceInformationPtr si;

    ompl::base::OptimizationObjectivePtr optimization_objective;

    ompl::base::PlannerAllocator alloc;

    bool useImprovisedSampler;
    bool tryLuckyShots;
    bool useCostConvergence;

public:
    [[nodiscard]] const ompl::base::OptimizationObjectivePtr &getOptimizationObjective() const;

    SingleGoalPlannerMethods(const double planTimePerAppleSeconds, ompl::base::SpaceInformationPtr si,
                             ompl::base::OptimizationObjectivePtr optimizationObjective,
                             ompl::base::PlannerAllocator alloc, bool useImprovisedSampler, bool tryLuckyShots,
                             bool useCostConvergence)
            : timePerAppleSeconds(planTimePerAppleSeconds),
              si(std::move(si)),
              optimization_objective(optimizationObjective),
              alloc(alloc),
              useImprovisedSampler(useImprovisedSampler), tryLuckyShots(tryLuckyShots),
              useCostConvergence(useCostConvergence) {}

    std::optional<ompl::geometric::PathGeometric> state_to_goal(const ompl::base::State *a, const ompl::base::GoalPtr b);

    std::optional<ompl::geometric::PathGeometric> state_to_state(const ompl::base::State *a, const ompl::base::State *b);

    [[nodiscard]] Json::Value parameters() const;

    std::optional<ompl::geometric::PathGeometric>
    attempt_lucky_shot(const ompl::base::State *a, const ompl::base::GoalPtr& b);
};

typedef std::function<std::shared_ptr<SingleGoalPlannerMethods>(ompl::base::SpaceInformationPtr,
                                                                ompl::base::OptimizationObjectivePtr,
                                                                ompl::base::PlannerAllocator)>
        SingleGoalPlannerMethodsAllocator;

#endif //NEW_PLANNERS_SINGLEGOALPLANNERMETHODS_H
