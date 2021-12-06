#include "json_utils.h"
#include "multigoal/multi_goal_planners.h"
#include "multigoal/knn.h"
#include "multigoal/uknn.h"
#include "multigoal/PointToPointPlanner.h"
#include "ompl_custom.h"
#include "LeavesCollisionChecker.h"
#include "SamplerWrapper.h"
#include "multigoal/MetricTwoOpt.h"
#include "experiment_utils.h"
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <json/json.h>

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    const int RUNS = 50; // 100 Is the value reported in the paper.
    Json::Value benchmark_results;

    std::random_device rd;
    std::mt19937 gen(rd());

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    for (int i = 0; i < RUNS; i++) {

        Json::Value benchmark_stats;

        int numberOfApples = std::uniform_int_distribution(5, 50)(gen);

        std::cout << "Run " << (i + 1) << " out of " << RUNS << " with " << numberOfApples << " apples." << std::endl;

        auto tree_scene = buildPlanningScene(numberOfApples, drone);
        const std::shared_ptr<ompl::base::SpaceInformation> si = initSpaceInformation(tree_scene.scene, drone,
                                                                                      state_space);

        auto leavesCollisionChecker = std::make_shared<LeavesCollisionChecker>(tree_scene.leaf_vertices);

        const robot_state::RobotState start_state = genStartState(drone);

        // Problem: Leaf count has no known admissible distance heuristic, A* doesn't like it very much!
        auto leafCountObjective = std::make_shared<LeavesCollisionCountObjective>(si, drone,
                                                                                  leavesCollisionChecker);

        auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);

        auto multiObjective50_50 = std::make_shared<ompl::base::MultiOptimizationObjective>(si);
        multiObjective50_50->addObjective(leafCountObjective, 0.5);
        multiObjective50_50->addObjective(pathLengthObjective, 0.5);

        StateProjection stateProjection{state_space.get()};

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

        std::chrono::milliseconds ptp_budget1(100);
        std::chrono::milliseconds ptp_budget2(200);


        for (auto budget: {500, 1000, 2000, 5000/*, 7500, 10000, 15000, 20000*/ }) {

            std::vector<std::function<std::shared_ptr<SamplerWrapper>()>> samplers{
//                    [&](){return std::make_shared<UniformSampler>(state_space.get());},
                    [&]() { return std::make_shared<InformedGaussian>(state_space.get(), 2.5); }
            };

            for (const auto &mksampler: samplers) {
                experiments.push_back({
                                              std::make_shared<KNNPlanner>(1, goalProjection, stateProjection, 1.0),
                                              mkprms(),
                                              pathLengthObjective,
                                              mksampler(),
                                              std::chrono::milliseconds(budget)
                                      });

                experiments.push_back({
                                              std::make_shared<MetricTwoOpt>(goalProjection, stateProjection, 0.2),
                                              mkprms(),
                                              pathLengthObjective,
                                              mksampler(),
                                              std::chrono::milliseconds(budget)
                                      });

                experiments.push_back({
                                              std::make_shared<MetricTwoOpt>(goalProjection, stateProjection, 0.0),
                                              mkprms(),
                                              pathLengthObjective,
                                              mksampler(),
                                              std::chrono::milliseconds(budget)
                                      });
            }

            This
            is
            your
            reminder
            to
            use
            KNN
            with
            higher
            K and more
            time
            per
            apple.
        }

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

            std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> goals = constructAppleGoals(si,
                                                                                                       tree_scene.apples);

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

            std::cout << "Goals: ";
            for (const auto &item: result.segments) std::cout << item.to_goal << ",";
            std::cout << std::endl;

            if (abs(experiment.time_budget - elapsed) > std::chrono::milliseconds(500)) {
                std::cerr << "Warning: this is significantly over the allotted time budget!" << std::endl;
            }

            result.check_valid(goals, *si);

            Json::Value run_stats = buildRunStatistics(leavesCollisionChecker, experiment, result,
                                                       std::chrono::duration_cast<std::chrono::milliseconds>(elapsed),
                                                       drone);

            benchmark_stats["planner_runs"].append(run_stats);

        }

        benchmark_stats["number_of_apples"] = numberOfApples;

        benchmark_results.append(benchmark_stats);
    }

    std::ofstream results("analysis/results.json");
    results << benchmark_results;
    results.close();

    std::cout << "Done!" << std::endl;

    return 0;
}

