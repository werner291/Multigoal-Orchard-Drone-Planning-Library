//
// Created by werner on 28-09-21.
//

#ifndef NEW_PLANNERS_JSON_UTILS_H
#define NEW_PLANNERS_JSON_UTILS_H

#include "procedural_tree_generation.h"
#include "LeavesCollisionChecker.h"
#include "ompl_custom.h"
#include "multigoal/multi_goal_planners.h"
#include "multigoal/PointToPointPlanner.h"
#include <random>
#include <robowflex_library/trajectory.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>

void mergeIntoLeft(Json::Value &receiver, const Json::Value &donor);

Json::Value eigenToJson(const Eigen::Vector3d &vec);

//Json::Value makePointToPointJson(const std::optional<PointToPointPlanResult> &pointToPointPlanResult);

Json::Value getStateStatisticsPoint(const moveit::core::RobotState &st);

#endif //NEW_PLANNERS_JSON_UTILS_H
