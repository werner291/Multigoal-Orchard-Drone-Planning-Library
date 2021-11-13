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
#include "experiment_utils.h"

std::vector<LeafCollisions> collectLeafCollisionStats(const LeavesCollisionChecker &leavesCollisionChecker,
                                                      const robot_trajectory::RobotTrajectory &trajectory);

Json::Value toJSON(const LeafCollisions &leaf_collisions);

Json::Value buildRunStatistics(const std::shared_ptr<LeavesCollisionChecker> &leavesCollisionChecker,
                               const Experiment &experiment,
                               const MultiGoalPlanResult &result,
                               const std::chrono::milliseconds elapsed,
                               const moveit::core::RobotModelPtr &robot);

#endif //NEW_PLANNERS_JSON_UTILS_H
