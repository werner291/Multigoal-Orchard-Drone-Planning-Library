
#ifndef MULTI_GOAL_PLANNERS_H
#define MULTI_GOAL_PLANNERS_H

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/trajectory.h>
#include <json/value.h>
#include "../procedural_tree_generation.h"
#include "../ompl_custom.h"

const double MAX_TIME_PER_TARGET_SECONDS = 0.1;

void extendTrajectory(robowflex::Trajectory &full_trajectory, robowflex::Trajectory &extension);

struct MultiGoalPlanResult {
    robowflex::Trajectory trajectory;
    Json::Value stats;
};

struct PointToPointPlanResult {
    double solution_length{};
    robowflex::Trajectory point_to_point_trajectory;
};

class MultiGoalPlanner {

public:

    virtual MultiGoalPlanResult plan(const std::vector<Apple> &apples,
                                     const moveit::core::RobotState &start_state,
                                     const robowflex::SceneConstPtr &scene,
                                     const robowflex::RobotConstPtr &robot,
                                     ompl::base::Planner &point_to_point_planner) = 0;


    virtual std::string getName() = 0;
};

std::optional<PointToPointPlanResult>
planPointToPoint(const robowflex::RobotConstPtr &robot, ompl::base::Planner &planner, const ompl::base::GoalPtr &goal,
                 const moveit::core::RobotState &from_state, double max_time);

#endif //MULTI_GOAL_PLANNERS_H
