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
#include "multigoal/ATNN.h"
#include "multigoal/ATRandom.h"
#include "multigoal/RandomizedTwoOpt.h"
#include "SamplerWrapper.h"
#include "multigoal/MetricTwoOpt.h"
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <json/json.h>

using namespace robowflex;

Json::Value collectLeafCollisionStats(const LeavesCollisionChecker &leavesCollisionChecker,
                                      const robot_trajectory::RobotTrajectory &trajectory) {

    assert(!trajectory.empty());

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

    const int RUNS = 50; // 100 Is the value reported in the paper.
    Json::Value benchmark_results;

    std::random_device rd;
    std::mt19937 gen(rd());

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone->getModelConst(), "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    for (int i = 0; i < RUNS; i++) {

        Json::Value benchmark_stats;

        auto scene = std::make_shared<Scene>(drone);

        double apple_t = std::uniform_real_distribution(0.0, 1.0)(gen);

        int numberOfApples = std::uniform_int_distribution(5, 50)(
                gen);//5 + (apple_t * apple_t) * 50;//150 TODO change this back;

        std::cout << "Run " << (i + 1) << " out of " << RUNS << " with " << numberOfApples << " apples." << std::endl;

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
            std::shared_ptr<SamplerWrapper> sampler;
            std::chrono::milliseconds time_budget;
        };

        // Problem: Leaf count has no known admissible distance heuristic, A* doesn't like it very much!
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

        auto stateToGoal = [&](const ompl::base::State *st, const ompl::base::Goal *gl) {
            // TODO: weigh in the fact that the joint at the start of the arm can actually have an outsized effect on end effector movement.
            // Easiest would be to simply weigh that joint's contribution to cost by the length of the arm.
            // That's probably not even too outrageous from a scientific point of view, since moving the whole
            // arm is more expensive than moving part of the arm.
            return (stateProjection(st) - goalProjection(gl)).norm();
        };

        auto goalToGoal = [&](const ompl::base::Goal *a, const ompl::base::Goal *b) {
            // TODO: weigh in the fact that the joint at the start of the arm can actually have an outsized effect on end effector movement.
            // Easiest would be to simply weigh that joint's contribution to cost by the length of the arm.
            return (goalProjection(a) - goalProjection(b)).norm();
        };

        auto mknn = [&]() { return std::make_shared<KNNPlanner>(1, goalProjection, stateProjection, 1.0); };
        auto mkunn = [&]() { return std::make_shared<UnionKNNPlanner>(1, goalProjection, stateProjection); };
        auto mkprms = [&]() { return std::make_shared<ompl::geometric::PRMstar>(si); };

        std::vector<Experiment> experiments;

//        for (auto budget : { 100, 500, 1000, 2000, 5000, 7500, /*10000, 15000, 20000*/ }) {
//
//            std::vector<std::function<std::shared_ptr<SamplerWrapper>()>> samplers {
//                    [&](){return std::make_shared<UniformSampler>(state_space.get());},
//                    [&](){return std::make_shared<InformedGaussian>(state_space.get(), 2.5);}
//            };
//
//            for (auto mksampler : samplers) {
//                experiments.push_back({
//                                              std::make_shared<KNNPlanner>(1, goalProjection, stateProjection, 1.0),
//                                              mkprms(),
//                                              pathLengthObjective,
//                                              mksampler(),
//                                              std::chrono::milliseconds(budget)
//                                      });
//                experiments.push_back({
//                                              std::make_shared<KNNPlanner>(1, goalProjection, stateProjection, 2.0),
//                                              mkprms(),
//                                              pathLengthObjective,
//                                              mksampler(),
//                                              std::chrono::milliseconds(budget)
//                                      });
//            }
//        }

        std::chrono::milliseconds ptp_budget1(100);
        std::chrono::milliseconds ptp_budget2(200);

        for (auto budget: {500, 1000/*, 2000, 5000, 7500, 10000, 15000, 20000*/ }) {

            std::vector<std::function<std::shared_ptr<SamplerWrapper>()>> samplers{
//                    [&](){return std::make_shared<UniformSampler>(state_space.get());},
                    [&]() { return std::make_shared<InformedGaussian>(state_space.get(), 2.5); }
            };

            for (auto mksampler: samplers) {
                experiments.push_back({
                                              std::make_shared<KNNPlanner>(1, goalProjection, stateProjection, 1.0),
                                              mkprms(),
                                              pathLengthObjective,
                                              mksampler(),
                                              std::chrono::milliseconds(budget)
                                      });
//                experiments.push_back({
//                                              std::make_shared<RandomizedTwoOpt>(mknn(), goalToGoal, stateToGoal, false, false, 0.5),
//                                              mkprms(),
//                                              pathLengthObjective,
//                                              mksampler(),
//                                              std::chrono::milliseconds(budget)
//                                      });

                experiments.push_back({
                                              std::make_shared<RandomizedTwoOpt>(mknn(), goalToGoal, stateToGoal, false,
                                                                                 false, 0.25, ptp_budget1),
                                              mkprms(),
                                              pathLengthObjective,
                                              mksampler(),
                                              std::chrono::milliseconds(budget)
                                      });
                experiments.push_back({
                                              std::make_shared<RandomizedTwoOpt>(mknn(), goalToGoal, stateToGoal, false,
                                                                                 false, 0.25, ptp_budget2),
                                              mkprms(),
                                              pathLengthObjective,
                                              mksampler(),
                                              std::chrono::milliseconds(budget)
                                      });

                experiments.push_back({
                                              std::make_shared<MetricTwoOpt>(goalProjection, stateProjection),
                                              mkprms(),
                                              pathLengthObjective,
                                              mksampler(),
                                              std::chrono::milliseconds(budget)
                                      });
            }
        }

//        std::chrono::milliseconds time_budget(10000 + 100 * numberOfApples);
//
//        std::vector<Experiment> experiments{
//
//                {std::make_shared<RandomizedTwoOpt>(mknn(), goalToGoal, stateToGoal, false, false),
//                        mkprms(),
//                        pathLengthObjective,
//                        std::make_shared<UniformSampler>(state_space.get()),
//                        time_budget},
//
//                {std::make_shared<RandomizedTwoOpt>(mknn(), goalToGoal, stateToGoal, false, false),
//                        mkprms(),
//                        pathLengthObjective,
//                        std::make_shared<InformedGaussian>(state_space.get(), 5.0),
//                        time_budget},
//
//                {mknn(),
//                 mkprms(),
//                        pathLengthObjective,
//                        std::make_shared<UniformSampler>(state_space.get()),
//                        time_budget},
//
//                {mknn(),
//                        mkprms(),
//                        pathLengthObjective,
//                        std::make_shared<InformedGaussian>(state_space.get(), 5.0),
//                        time_budget},
//
//                {mknn(),
//                        mkprms(),
//                        pathLengthObjective,
//                        std::make_shared<InformedGaussian>(state_space.get(), 2.5),
//                        time_budget},

//                {std::make_shared<UnionKNNPlanner>(1, goalProjection, stateProjection),
//                        std::make_shared<ompl::geometric::PRMstar>(si),
//                        leafCountObjective,  std::make_shared<UniformSampler>(state_space.get())},
//
//                {std::make_shared<UnionKNNPlanner>(1, goalProjection, stateProjection),
//                        std::make_shared<ompl::geometric::PRMstar>(si),
//                        leafCountObjective,  std::make_shared<InformedGaussian>(state_space.get())},
//
//                {std::make_shared<RandomizedTwoOpt>(mknn(), goalToGoal, stateToGoal, true, true),
//                        std::make_shared<ompl::geometric::PRMstar>(si),
//                        leafCountObjective,  std::make_shared<UniformSampler>(state_space.get())},
//
//                {std::make_shared<RandomizedTwoOpt>(mknn(), goalToGoal, stateToGoal, true, true),
//                        std::make_shared<ompl::geometric::PRMstar>(si),
//                        leafCountObjective,  std::make_shared<UniformSampler>(state_space.get())},

//        };

        // TODO: Throw in something about sampling strategies and different planners.
        // TODO try out the experience-based planners.


        for (const auto &experiment: experiments) {

            // Nesting order is important here because the sub-planners are re-created every run.
            std::cout << "Attempting " << experiment.meta_planner->getName()
                      << " with sub-planner " << experiment.ptp_planner->getName()
                      << ", objective " << experiment.optimization_objective->getDescription()
                      << ", " << experiment.sampler->getName() << " sampling"
                      << ", and " << experiment.time_budget.count() << "ms time."
                      << std::endl;

            PointToPointPlanner ptp(experiment.ptp_planner, experiment.optimization_objective,
                                    experiment.sampler);

            std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> goals;
            for (const auto &apple: tree_scene.apples)
                goals.push_back(
                        std::make_shared<DroneEndEffectorNearTarget>(si, GOAL_END_EFFECTOR_RADIUS, apple.center));

            ompl::base::ScopedState start_state_ompl(si);
            state_space->copyToOMPLState(start_state_ompl.get(), start_state);

            auto start = std::chrono::steady_clock::now();
            MultiGoalPlanResult result = experiment.meta_planner->plan(goals, start_state_ompl.get(), ptp,
                                                                       experiment.time_budget);
            auto end = std::chrono::steady_clock::now();

            // Let's be honest, the result is simply invalid if the time budget is not respected.
            auto elapsed = end - start;

            std::cout << "Elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << "ms"
                      << std::endl;

            if (abs(experiment.time_budget - elapsed) > std::chrono::milliseconds(500)) {
                std::cerr << "Warning: this is significantly over the allotted time budget!" << std::endl;
            }

            result.check_valid(goals, *si);

            robowflex::Trajectory full_trajectory(drone, "whole_body");
            for (const auto &item: result.segments) {
                extendTrajectory(full_trajectory, convertTrajectory(item.path, drone));
            }

            Json::Value run_stats;

            mergeIntoLeft(run_stats,
                          collectLeafCollisionStats(*leavesCollisionChecker, *full_trajectory.getTrajectory()));

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
            run_stats["total_path_length"] = full_trajectory.getLength();
            run_stats["total_runtime"] = (double) std::chrono::duration_cast<std::chrono::milliseconds>(
                    elapsed).count();
            run_stats["runtime_budget"] = (double) std::chrono::duration_cast<std::chrono::milliseconds>(
                    experiment.time_budget).count();

            benchmark_stats["planner_runs"].append(run_stats);

        }

        benchmark_results.append(benchmark_stats);
    }

    std::ofstream results("analysis/results.json");
    results << benchmark_results;
    results.close();

    std::cout << "Done!" << std::endl;

    return 0;
}