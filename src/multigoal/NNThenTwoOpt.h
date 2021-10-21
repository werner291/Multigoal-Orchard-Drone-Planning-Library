
#ifndef NEW_PLANNERS_NNTHENTWOOPT_H
#define NEW_PLANNERS_NNTHENTWOOPT_H

#include "multi_goal_planners.h"
#include "uknn.h"

class NNThenTwoOpt : public MultiGoalPlanner {

    GoalProjectionFn goalProjection_;
    StateProjectionFn stateProjection_;

public:
    NNThenTwoOpt(GoalProjectionFn goalProjection, StateProjectionFn stateProjection);

    MultiGoalPlanResult plan(const std::vector<GoalSamplerPtr> &goals, const ompl::base::State *start_state,
                             PointToPointPlanner &point_to_point_planner) override;

    std::string getName() override;

    static void trySwap(const std::vector<GoalSamplerPtr> &goals, MultiGoalPlanResult &result,
                        PointToPointPlanner &planner, size_t i, size_t j,
                        const ompl::base::State *start_state);
};


#endif //NEW_PLANNERS_NNTHENTWOOPT_H
