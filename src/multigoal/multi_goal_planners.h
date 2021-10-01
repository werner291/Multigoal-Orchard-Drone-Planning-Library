
#ifndef MULTI_GOAL_PLANNERS_H
#define MULTI_GOAL_PLANNERS_H

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/trajectory.h>
#include <json/value.h>
#include "../procedural_tree_generation.h"
#include "../ompl_custom.h"
#include "../LeavesCollisionChecker.h"
#include "PointToPointPlanner.h"

const double MAX_TIME_PER_TARGET_SECONDS = 0.1;

void extendTrajectory(robowflex::Trajectory &full_trajectory, robowflex::Trajectory &extension);

struct MultiGoalPlanResult {
    robowflex::Trajectory trajectory;
    Json::Value stats;
};

class MultiGoalPlanner {

public:

    virtual MultiGoalPlanResult plan(const TreeScene &apples,
                                     const moveit::core::RobotState &start_state,
                                     const robowflex::SceneConstPtr &scene,
                                     const robowflex::RobotConstPtr &robot,
                                     PointToPointPlanner &point_to_point_planner) = 0;

    virtual std::string getName() = 0;
};


#endif //MULTI_GOAL_PLANNERS_H
