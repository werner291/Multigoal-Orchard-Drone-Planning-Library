//
// Created by werner on 08-11-21.
//

#ifndef NEW_PLANNERS_METRICTWOOPT_H
#define NEW_PLANNERS_METRICTWOOPT_H

#include "multi_goal_planners.h"

class MetricTwoOpt : public MultiGoalPlanner {

    std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection_;
    std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection_;

public:
    MetricTwoOpt(std::function<Eigen::Vector3d(const ompl::base::Goal *)> goalProjection,
                 std::function<Eigen::Vector3d(const ompl::base::State *)> stateProjection);

    MultiGoalPlanResult
    plan(GoalSet &goals, const ompl::base::State *start_state, PointToPointPlanner &point_to_point_planner,
         std::chrono::milliseconds time_budget) override;

    std::string getName() override;


};


#endif //NEW_PLANNERS_METRICTWOOPT_H
