//
// Created by werner on 05-10-21.
//

#ifndef NEW_PLANNERS_TWOOPT_H
#define NEW_PLANNERS_TWOOPT_H


#include "multi_goal_planners.h"

class TwoOpt : public MultiGoalPlanner {
public:
    MultiGoalPlanResult
    plan(const TreeScene &apples, const moveit::core::RobotState &start_state, const robowflex::SceneConstPtr &scene,
         const robowflex::RobotConstPtr &robot, PointToPointPlanner &point_to_point_planner) override;

    std::string getName() override;

private:

    std::chrono::duration<double> max_time;
public:
    TwoOpt(const std::chrono::duration<double> &maxTime);

private:

    static MultiGoalPlanResult
    tryOrder(const moveit::core::RobotState &start_state, const robowflex::RobotConstPtr &robot,
             PointToPointPlanner &point_to_point_planner, std::vector<Eigen::Vector3d> &targets);
};


#endif //NEW_PLANNERS_TWOOPT_H
