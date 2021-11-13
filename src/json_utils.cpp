
#include <json/json.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "LeavesCollisionChecker.h"
#include "ompl_custom.h"
#include "json_utils.h"
#include "multigoal/multi_goal_planners.h"

Json::Value toJSON(const LeafCollisions &leaf_collisions) {
    Json::Value leaf_collisions_json;
    leaf_collisions_json["t"] = leaf_collisions.t;
    leaf_collisions_json["contacts_ended"] = (int) leaf_collisions.new_contacts;
    leaf_collisions_json["new_leaves_in_contact"] = (int) leaf_collisions.removed_contacts;
    return leaf_collisions_json;
}

double trajLen(robot_trajectory::RobotTrajectory &traj) {
    double length = 0.0;

    for (std::size_t k = 1; k < traj.getWayPointCount(); ++k) {
        const auto &s1 = traj.getWayPoint(k - 1);
        const auto &s2 = traj.getWayPoint(k);

        length += s1.distance(s2);
    }

    return length;
}

Json::Value
buildRunStatistics(const std::shared_ptr<LeavesCollisionChecker> &leavesCollisionChecker,
                   const Experiment &experiment,
                   const MultiGoalPlanResult &result,
                   const std::chrono::milliseconds elapsed,
                   const moveit::core::RobotModelPtr &robot) {

//    robowflex::Trajectory full_trajectory(robot, "whole_body");
//
//    for (const auto &item: result.segments) {
//        extendTrajectory(full_trajectory, convertTrajectory(item.path, robot));
//    }

    robot_trajectory::RobotTrajectory full_trajectory(robot, "whole_body");

    for (const auto &item: result.segments) {

        moveit::core::RobotState st(robot);

        auto state_space = item.path.getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

        for (size_t i = 0; i < item.path.getStateCount(); ++i) {
            state_space->copyToRobotState(st, item.path.getState(i));
            full_trajectory.addSuffixWayPoint(st, 1.0);
        }
    }


    Json::Value run_stats;

    for (const auto &item: collectLeafCollisionStats(*leavesCollisionChecker, full_trajectory)) {
        run_stats["leaf_collisions"].append(toJSON(item));
    }

    run_stats["targets_visited"] = (int) result.segments.size();

    Json::Value segment_stats;

    for (const auto &segment: result.segments) {
        Json::Value seg_json;
        seg_json["goal_idx"] = (int) segment.to_goal;
        seg_json["length"] = segment.path.length();
        segment_stats.append(seg_json);
    }

    run_stats["segment_stats"] = segment_stats;

    run_stats["order_planning"] = experiment.meta_planner->getName();
    run_stats["intermediate_planner"] = experiment.ptp_planner->getName();
    run_stats["sampler"] = experiment.sampler->getName();
    run_stats["optimization_objective"] = experiment.optimization_objective->getDescription();
    run_stats["total_path_length"] = trajLen(full_trajectory);
    run_stats["total_runtime"] = (double) std::chrono::duration_cast<std::chrono::milliseconds>(
            elapsed).count();
    run_stats["runtime_budget"] = (double) std::chrono::duration_cast<std::chrono::milliseconds>(
            experiment.time_budget).count();
    return run_stats;
}