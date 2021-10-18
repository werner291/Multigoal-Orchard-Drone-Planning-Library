#include <robowflex_library/builder.h>
#include <robowflex_library/util.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/trajectory.h>
#include "msgs_utilities.h"
#include "json_utils.h"
#include "build_planning_scene.h"
#include "make_robot.h"
#include "InverseClearanceIntegralObjective.h"
#include "BulletContinuousMotionValidator.h"
#include "multigoal/multi_goal_planners.h"
#include "multigoal/knn.h"
#include "multigoal/uknn.h"
#include "multigoal/PointToPointPlanner.h"
#include "ompl_custom.h"
#include "LeavesCollisionChecker.h"
#include "multigoal/AT2Opt.h"
//#include "multigoal/TwoOpt.h"
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <json/json.h>

using namespace robowflex;

Json::Value collectLeafCollisionStats(const LeavesCollisionChecker &leavesCollisionChecker,
                                      const robot_trajectory::RobotTrajectory &trajectory) {

    Json::Value leaf_collision_stats;
    size_t unique_collisions = 0;

    auto scratchState = std::make_shared<moveit::core::RobotState>(trajectory.getWayPoint(0).getRobotModel());

    std::set<size_t> leaves;
    for (size_t ti = 0; ti < 10000; ti++) {

        double t = (double) ti * trajectory.getDuration() / 10000.0;

        trajectory.getStateAtDurationFromStart(t, scratchState);

        std::set<size_t> new_leaves = leavesCollisionChecker.checkLeafCollisions(*scratchState);

        std::set<size_t> added_leaves;
        std::set_difference(new_leaves.begin(), new_leaves.end(), leaves.begin(), leaves.end(),
                            std::inserter(added_leaves, added_leaves.end()));

        std::set<size_t> removed_leaves;
        std::set_difference(leaves.begin(), leaves.end(), new_leaves.begin(), new_leaves.end(),
                            std::inserter(removed_leaves, removed_leaves.end()));

        unique_collisions += added_leaves.size();

        if (!added_leaves.empty() || !removed_leaves.empty()) {
            Json::Value leaf_collisions;
            leaf_collisions["t"] = t;
            leaf_collisions["contacts_ended"] = (int) removed_leaves.size();
            leaf_collisions["new_leaves_in_contact"] = (int) added_leaves.size();
            leaf_collision_stats["leaf_collisions_over_time"].append(leaf_collisions);
        }

        leaves = new_leaves;

    }

    leaf_collision_stats["unique_leaves_collided"] = (int) unique_collisions;

    return leaf_collision_stats;
}

int main(int argc, char **argv) {

    // Startup ROS
    ROS ros(argc, argv);

    std::shared_ptr<Robot> drone = make_robot();

    const int RUNS = 3; // 100 Is the value reported in the paper.
    Json::Value benchmark_results;

    std::random_device rd;
    std::mt19937 gen(rd());

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone->getModelConst(), "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    for (int i = 0; i < RUNS; i++) {

        Json::Value benchmark_stats;

        auto scene = std::make_shared<Scene>(drone);

        double apple_t = std::uniform_real_distribution(0.0,1.0)(gen);

        int numberOfApples = 5 + (apple_t * apple_t) * 150;

        std::cout << "Run " << (i+1) << " out of " << RUNS << " with " << numberOfApples << " apples." << std::endl;

        benchmark_stats["number_of_apples"] = numberOfApples;

        auto tree_scene = establishPlanningScene(10, numberOfApples);
        scene->getScene()->setPlanningSceneDiffMsg(tree_scene.moveit_diff);
        // Diff message apparently can't handle this?
        scene->getScene()->getAllowedCollisionMatrixNonConst().setDefaultEntry("leaves", true);
        scene->getScene()->getAllowedCollisionMatrixNonConst().setDefaultEntry("apples", true);
        scene->getScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

        auto leavesCollisionChecker = std::make_shared<LeavesCollisionChecker>(tree_scene.leaf_vertices);

        const std::shared_ptr<ompl::base::SpaceInformation> si = initSpaceInformation(scene, drone, state_space);
        const robot_state::RobotState start_state = genStartState(drone);

        struct Experiment {
            std::shared_ptr<MultiGoalPlanner> meta_planner;
            std::shared_ptr<ompl::base::Planner> ptp_planner;
            std::shared_ptr<ompl::base::OptimizationObjective> optimization_objective;
        };

        auto leafCountObjective = std::make_shared<LeavesCollisionCountObjective>(si, drone->getModelConst(),
                                                                                  leavesCollisionChecker);

        auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);

        auto multiObjective50_50 = std::make_shared<ompl::base::MultiOptimizationObjective>(si);
        multiObjective50_50->addObjective(leafCountObjective, 0.5);
        multiObjective50_50->addObjective(pathLengthObjective, 0.5);

        auto stateProjection = [&](const ompl::base::State *state) {
            moveit::core::RobotState st(drone->getModelConst());
            state_space->copyToRobotState(st, state);
            st.update(true);
            return st.getGlobalLinkTransform("end_effector").translation();
        };

        auto goalProjection = [&](const ompl::base::Goal *goal) {
            return goal->as<DroneEndEffectorNearTarget>()->getTarget();
        };

        std::vector<Experiment> experiments{
//                {std::make_shared<KNNPlanner>(1),      std::make_shared<ompl::geometric::PRMstar>(
//                        si),                                        pathLengthObjective},
//                {std::make_shared<KNNPlanner>(2),      std::make_shared<ompl::geometric::PRMstar>(
//                        si),                                        pathLengthObjective},
//                {std::make_shared<KNNPlanner>(3),      std::make_shared<ompl::geometric::PRMstar>(
//                        si),                                        pathLengthObjective},
                {std::make_shared<UnionKNNPlanner>(1, goalProjection, stateProjection),
                 std::make_shared<ompl::geometric::PRMstar>(si), pathLengthObjective},
//                {std::make_shared<UnionKNNPlanner>(2, goalProjection, stateProjection),
//                                                       std::make_shared<ompl::geometric::PRMstar>(
//                                                               si), pathLengthObjective},
//                {std::make_shared<UnionKNNPlanner>(3, goalProjection, stateProjection),
//                                                       std::make_shared<ompl::geometric::PRMstar>(
//                                                               si), pathLengthObjective},
//                {std::make_shared<UnionKNNPlanner>(3), std::make_shared<ompl::geometric::PRMstar>(
//                        si),                                        leafCountObjective},
//                {std::make_shared<UnionKNNPlanner>(3), std::make_shared<ompl::geometric::PRMstar>(si), multiObjective50_50 },
                {std::make_shared<AT2Opt>(), std::make_shared<ompl::geometric::PRMstar>(si), pathLengthObjective},
        };

        for (const auto &experiment: experiments) {

            // Nesting order is important here because the sub-planners are re-created every run.
            std::cout << "Attempting " << experiment.meta_planner->getName()
                      << " with sub-planner " << experiment.ptp_planner->getName()
                      << " and objective " << experiment.optimization_objective->getDescription()
                      << std::endl;

            PointToPointPlanner ptp(experiment.ptp_planner, experiment.optimization_objective);

            std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> goals;
            for (const auto &apple: tree_scene.apples)
                goals.push_back(
                        std::make_shared<DroneEndEffectorNearTarget>(si, GOAL_END_EFFECTOR_RADIUS, apple.center));

            ompl::base::ScopedState start_state_ompl(si);
            state_space->copyToOMPLState(start_state_ompl.get(), start_state);

            auto start = std::chrono::steady_clock::now();
            MultiGoalPlanResult result = experiment.meta_planner->plan(goals, start_state_ompl.get(), ptp);
            auto end = std::chrono::steady_clock::now();

            robowflex::Trajectory full_trajectory(drone, "whole_body");
            for (const auto &item: result.segments) {
                extendTrajectory(full_trajectory, convertTrajectory(item.path, drone));
            }

            Json::Value run_stats;

            mergeIntoLeft(run_stats,
                          collectLeafCollisionStats(*leavesCollisionChecker, *full_trajectory.getTrajectory()));

            run_stats["intermediate_planner"] = experiment.ptp_planner->getName();
            run_stats["optimization_objective"] = experiment.optimization_objective->getDescription();
            run_stats["total_path_length"] = full_trajectory.getLength();
            run_stats["total_runtime"] = (double) std::chrono::duration_cast<std::chrono::milliseconds>(
                    end - start).count();

            benchmark_stats["planner_runs"].append(run_stats);

        }

        benchmark_results.append(benchmark_stats);
    }

    std::ofstream results(RUNS < 50 ? "analysis/results_test.json" : "analysis/results.json");
    results << benchmark_results;
    results.close();

    std::cout << "Done!" << std::endl;

    return 0;
}