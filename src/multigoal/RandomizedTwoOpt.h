
#ifndef NEW_PLANNERS_RANDOMIZEDTWOOPT_H
#define NEW_PLANNERS_RANDOMIZEDTWOOPT_H

#include "multi_goal_planners.h"
#include "uknn.h"

class RandomizedTwoOpt : public MultiGoalPlanner {

    std::shared_ptr<MultiGoalPlanner> initialAttemptPlanner_;
public:
    RandomizedTwoOpt(std::shared_ptr<MultiGoalPlanner> initialAttemptPlanner);

    MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals,
                             const ompl::base::State *start_state,
                             PointToPointPlanner &point_to_point_planner,
                             std::chrono::milliseconds time_budget) override;

    std::string getName() override;

    static void trySwap(const std::vector<GoalSamplerPtr> &goals, MultiGoalPlanResult &result,
                        PointToPointPlanner &planner, size_t i, size_t j,
                        const ompl::base::State *start_state);
};


#endif //NEW_PLANNERS_RANDOMIZEDTWOOPT_H
