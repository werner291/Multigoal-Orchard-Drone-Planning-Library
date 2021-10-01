//
// Created by werner on 30-09-21.
//

#include "uknn.h"
#include "../UnionGoalSampleableRegion.h"
#include "../json_utils.h"

UnionKNNPlanner::UnionKNNPlanner(size_t k) : k(k) {}

MultiGoalPlanResult UnionKNNPlanner::plan(const TreeScene &apples, const moveit::core::RobotState &start_state,
                                          const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                                          ompl::base::Planner &point_to_point_planner) {
    ompl::NearestNeighborsGNAT<Eigen::Vector3d> unvisited_nn;
    unvisited_nn.setDistanceFunction([](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        return (a - b).norm();
    });

    for (const Apple &apple: apples.apples) {
        unvisited_nn.add(apple.center);
    }

    robowflex::Trajectory full_trajectory(robot, "whole_body");
    full_trajectory.addSuffixWaypoint(start_state);

    Json::Value root;

    while (unvisited_nn.size() > 0) {

        const Eigen::Vector3d start_eepos = full_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                "end_effector").translation();

        std::vector<Eigen::Vector3d> knn;
        unvisited_nn.nearestK(start_eepos, k, knn);

        std::vector<std::shared_ptr<const ompl::base::GoalSampleableRegion>> subgoals;

        for (const auto &target: knn) {
            subgoals.push_back(std::make_shared<DroneEndEffectorNearTarget>(
                    point_to_point_planner.getSpaceInformation(), 0.2,
                    target));
        }

        auto pointToPointResult = planPointToPoint(robot,
                                                   point_to_point_planner,
                                                   std::make_shared<UnionGoalSampleableRegion>(
                                                           point_to_point_planner.getSpaceInformation(), subgoals),
                                                   full_trajectory.getTrajectory()->getLastWayPoint(),
                                                   MAX_TIME_PER_TARGET_SECONDS);

        if (pointToPointResult.has_value()) {

            auto traj = pointToPointResult.value().point_to_point_trajectory.getTrajectory();

            const Eigen::Vector3d end_eepos = pointToPointResult.value().point_to_point_trajectory.getTrajectory()->getLastWayPoint().getGlobalLinkTransform(
                    "end_effector").translation();

            bool which_target = false;
            for (auto &tgt: knn) {
                if ((tgt - end_eepos).norm() < 0.2) {
                    unvisited_nn.remove(tgt);
                    extendTrajectory(full_trajectory, pointToPointResult.value().point_to_point_trajectory);
                    root["segments"].append(makePointToPointJson(tgt, pointToPointResult));
                    which_target = true;
                    break;
                }
            }
            assert(which_target);
        } else {
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }


    }

    root["ordering"] = this->getName();

    return {full_trajectory, root};
}