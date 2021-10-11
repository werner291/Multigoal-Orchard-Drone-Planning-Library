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

    MultiGoalPlanResult result;

    while (unvisited_nn.size() > 0) {

        // Use forward kinematics to compute the end-effector position.
        const auto segment_start_state = result.segments.empty() ? start_state
                                                                 : result.segments.back().point_to_point_trajectory.getTrajectory()->getLastWayPoint();
        const Eigen::Vector3d start_eepos =
                segment_start_state
                        .getGlobalLinkTransform("end_effector").translation();

        std::vector<Eigen::Vector3d> knn;
        unvisited_nn.nearestK(start_eepos, k, knn);

        auto pointToPointResult = point_to_point_planner.planToEndEffectorTarget(segment_start_state, knn,
                                                                                 MAX_TIME_PER_TARGET_SECONDS);

        if (pointToPointResult.has_value()) {
            unvisited_nn.remove(pointToPointResult.value().endEffectorTarget);
            result.segments.push_back(
                    pointToPointResult.value()
            );
        } else {
            unvisited_nn.remove(knn[0]); // Better picks here? Maybe delete all?
        }
    }

    return result;
}