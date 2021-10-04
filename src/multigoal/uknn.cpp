//
// Created by werner on 30-09-21.
//

#include "uknn.h"
#include "../UnionGoalSampleableRegion.h"
#include "../json_utils.h"

UnionKNNPlanner::UnionKNNPlanner(size_t k) : k(k) {}

MultiGoalPlanResult UnionKNNPlanner::plan(const TreeScene &apples,
                                          const moveit::core::RobotState &start_state,
                                          const robowflex::SceneConstPtr &scene,
                                          const robowflex::RobotConstPtr &robot,
                                          PointToPointPlanner &point_to_point_planner) {

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

        auto pointToPointResult = point_to_point_planner.planPointToPoint(
                full_trajectory.getTrajectory()->getLastWayPoint(), knn, 0);

        if (pointToPointResult.has_value()) {
            unvisited_nn.remove(pointToPointResult.value().endEffectorTarget);
            extendTrajectory(full_trajectory, pointToPointResult.value().point_to_point_trajectory);
            root["segments"].append(makePointToPointJson(pointToPointResult));
        } else {
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }


    }

    root["ordering"] = this->getName();

    return {full_trajectory, root};
}