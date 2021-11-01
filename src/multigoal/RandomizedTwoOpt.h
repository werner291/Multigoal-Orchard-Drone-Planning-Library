
#ifndef NEW_PLANNERS_RANDOMIZEDTWOOPT_H
#define NEW_PLANNERS_RANDOMIZEDTWOOPT_H

#include <functional>
#include "multi_goal_planners.h"
#include "uknn.h"


class RandomizedTwoOpt : public MultiGoalPlanner {

    typedef std::function<double(const ompl::base::Goal *, const ompl::base::Goal *)> BetweenGoalFn;

    typedef std::function<double(const ompl::base::State *, const ompl::base::Goal *)> CostToGoFn;

    std::shared_ptr<MultiGoalPlanner> initialAttemptPlanner_;

    /// TODO: I'll want to package those three up into a struct at tome point.
    BetweenGoalFn betweenGoalDistanceHeuristic_;
    CostToGoFn betweenStateAndGoalHeuristic_;
    bool useCostRejectionHeuristic;

public:
    RandomizedTwoOpt(std::shared_ptr<MultiGoalPlanner> initialAttemptPlanner,
                     std::function<double(const ompl::base::Goal *,
                                          const ompl::base::Goal *)> betweenGoalDistanceHeuristic,
                     std::function<double(const ompl::base::State *,
                                          const ompl::base::Goal *)> betweenStateAndGoalHeuristic,
                     bool useCostRejectionHeuristic);

    MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals,
                             const ompl::base::State *start_state,
                             PointToPointPlanner &point_to_point_planner,
                             std::chrono::milliseconds time_budget) override;

    std::string getName() override;

    void trySwap(const std::vector<GoalSamplerPtr> &goals, MultiGoalPlanResult &result,
                 PointToPointPlanner &planner, size_t i, size_t j, const ompl::base::State *start_state,
                 double maxTimePerSegment);

    double
    computeNewPathLengthLowerbound(const ompl::base::State *start_state, PointToPointPlanner &point_to_point_planner,
                                   const GoalSet &goals, const MultiGoalPlanResult &solution,
                                   const std::vector<MultiGoalPlanResult::ReplacementSpec> &replacements);
};


#endif //NEW_PLANNERS_RANDOMIZEDTWOOPT_H
