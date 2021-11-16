//
// Created by werner on 08-11-21.
//

#ifndef NEW_PLANNERS_METRICTWOOPT_H
#define NEW_PLANNERS_METRICTWOOPT_H

#include "multi_goal_planners.h"

class MetricTwoOpt : public MultiGoalPlanner {

    GoalProjectionFn goalProjection_;
    StateProjectionFn stateProjection_;
    double swapping_budget_portion;

public:
    MetricTwoOpt(GoalProjectionFn goalProjection,
                 std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection,
                 double swappingBudgetPortion);

    MultiGoalPlanResult
    plan(GoalSet &goals, const ompl::base::State *start_state, PointToPointPlanner &point_to_point_planner,
         std::chrono::milliseconds time_budget) override;

    std::string getName() override;


};


#endif //NEW_PLANNERS_METRICTWOOPT_H
