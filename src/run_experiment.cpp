
#include "run_experiment.h"
#include "experiment_utils.h"
#include "probe_retreat_move.h"
#include "NewMultiGoalPlanner.h"
#include "DistanceHeuristics.h"
#include "ShellPathPlanner.h"
#include "MultigoalPrmStar.h"
#include <range/v3/all.hpp>
#include <fstream>

using namespace std;

std::shared_ptr<DroneStateSpace> loadStateSpace(const moveit::core::RobotModelPtr &model) {

    // initialize the state space and such
    auto stateSpace = make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(model, "whole_body"), TRANSLATION_BOUND);

    return stateSpace;
}

ompl::base::SpaceInformationPtr loadSpaceInformation(
        const std::shared_ptr<DroneStateSpace> &stateSpace,
        const AppleTreePlanningScene &scene_info) {
    auto scene = setupPlanningScene(scene_info.scene_msg, stateSpace->getRobotModel());
    auto si = initSpaceInformation(scene, scene->getRobotModel(), stateSpace);
    return si;
}

Json::Value toJson(const NewMultiGoalPlanner::PlanResult &result) {
    Json::Value run_stats;
    run_stats["final_path_length"] = result.length();
    run_stats["goals_visited"] = (int) result.segments.size();
    return run_stats;
}

ompl::base::ScopedState<> genStartState(const shared_ptr<DroneStateSpace> &stateSpace) {
    auto start_state_moveit = stateOutsideTree(stateSpace->getRobotModel());
    ompl::base::ScopedState<> start_state(stateSpace);
    stateSpace->copyToOMPLState(start_state.get(), start_state_moveit);
    return start_state;
}

void
run_planner_experiment(const std::vector<NewMultiGoalPlannerAllocatorFn> &allocators,
                       const std::string &results_path,
                       const int num_runs,
                       const unsigned int nworkers) {

    auto drone = loadRobotModel();
    auto stateSpace = loadStateSpace(drone);

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    std::vector<std::thread> threads;
    std::mutex mut;

    Json::Value statistics;

    auto start_states =
            ranges::views::iota(0, num_runs)
            | ranges::views::transform([&](const auto &i) { return std::make_pair(i, genStartState(stateSpace)); });

    auto tasks = ranges::views::cartesian_product(
            allocators,
            start_states
    ) | ranges::to_vector;

    std::shuffle(tasks.begin(), tasks.end(), std::mt19937(std::random_device()()));

    size_t num_tasks = tasks.size();
    size_t current_task = 0;

    for (size_t thread_id = 0; thread_id < nworkers; ++thread_id) {
        threads.emplace_back([&]() {

            // Keeping this local.
            auto threadLocalStateSpace = loadStateSpace(drone);

            auto si = loadSpaceInformation(threadLocalStateSpace, scene_info);

            while (true) {

                size_t thread_current_task;

                {
                    std::lock_guard<std::mutex> lock(mut);
                    thread_current_task = current_task++;
                    if (thread_current_task >= num_tasks) {
                        return;
                    }
                }

                std::cout << "Starting task " << thread_current_task << " of " << num_tasks << std::endl;

                const auto &[planner_allocator, start_state_pair] = tasks[thread_current_task];
                const auto &[run_i, start_state] = start_state_pair;

                auto planner = planner_allocator(scene_info, si);
                auto goals = constructNewAppleGoals(si, scene_info.apples);

                auto objective = make_shared<DronePathLengthObjective>(si);

                auto start_time = ompl::time::now();
                auto result = planner->plan(si, start_state.get(), goals);
                auto run_time = ompl::time::seconds(ompl::time::now() - start_time);

                auto plan_result = toJson(result);
                plan_result["run_time"] = run_time;
                plan_result["start_state"] = (int) run_i;
                plan_result["planner_params"] = planner->parameters();
                plan_result["planner_name"] = planner->name();

                {
                    std::lock_guard<std::mutex> lock(mut);
                    statistics.append(plan_result);
                    std::cout << "Completed run " << statistics.size() << " out of " << tasks.size() << std::endl;
                }
            }
        });
    }

    for (auto &thread: threads) {
        thread.join();
    }

    std::ofstream ofs;
    ofs.open(results_path);
    ofs << statistics;
    ofs.close();
}

std::vector<NewMultiGoalPlannerAllocatorFn> make_shellpath_allocators() {

    bool applyShellstateOptimization[] = {true, false};
    bool useImprovisedInformedSampler[] = {false, true};
    bool tryLuckyShots[] = {false, true};
    bool useCostConvergence[] = {false, true};
    double ptp_time_seconds[] = {0.4, 0.5, 1.0};

    ompl::base::PlannerAllocator planner_allocators[] = {
            [](const ompl::base::SpaceInformationPtr &si) {
                return std::make_shared<ompl::geometric::PRM>(si);
            },
    };

    return ranges::views::cartesian_product(applyShellstateOptimization,
                                            ptp_time_seconds,
                                            planner_allocators,
                                            useImprovisedInformedSampler,
                                            tryLuckyShots,
                                            useCostConvergence)
           | ranges::views::transform([](const auto tuple) -> NewMultiGoalPlannerAllocatorFn {

        auto [shellOptimize, ptp_budget, allocator, improvised_sampler, tryLucky, costConvergence] = tuple;

        // I... believe this copies them?
        return [shellOptimize=shellOptimize,
                ptp_budget=ptp_budget,
                allocator=allocator,
                improvised_sampler=improvised_sampler,
                tryLucky=tryLucky,
                costConvergence=costConvergence](
                        const AppleTreePlanningScene &scene_info,
                        const ompl::base::SpaceInformationPtr &si) {

            auto shell = std::make_shared<SphereShell>(
                    scene_info.sphere_center,
                    scene_info.sphere_radius
            );

            auto heuristics = std::make_shared<GreatCircleOmplDistanceHeuristics>(
                    GreatCircleMetric(scene_info.sphere_center),
                    std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace())
            );

            auto ptp = std::make_shared<SingleGoalPlannerMethods>(
                    ptp_budget,
                    si,
                    std::make_shared<DronePathLengthObjective>(si),
                    allocator,
                    improvised_sampler,
                    tryLucky,
                    costConvergence
            );

            return std::make_shared<ShellPathPlanner>(shell, shellOptimize, heuristics, ptp);
        };
    }) | ranges::to_vector;
}

std::vector<NewMultiGoalPlannerAllocatorFn> make_tsp_over_prm_allocators() {

    const auto samples_per_goal = ranges::views::iota(2, 10);
    const double plan_times_seconds[] = {1.0, 2.0, 5.0, 10.0, 15.0, 20.0};//, 30.0, 60.0};
    const bool optimize_segments_options[] = {false,true};

    return ranges::views::cartesian_product(plan_times_seconds, samples_per_goal, optimize_segments_options)
           | ranges::views::filter([](const auto tuple) {
        const auto &[plan_time, samples, optimize_segments] = tuple;

        double expected_time = plan_time * (double) (samples * samples);

        if (expected_time >= 600.0) {
            std::cout << "Dropping task with plan_time: " << plan_time << " samples: " << samples << " expected_time: "
                      << expected_time << std::endl;
            return false;
        } else {
            return true;
        }

    }) | ranges::views::transform([&](const auto tuple) -> NewMultiGoalPlannerAllocatorFn {

        auto [plan_time, samples, optimize_segments] = tuple;

        return [plan_time=plan_time,
                samples=samples,
                optimize_segments=optimize_segments](
                        const AppleTreePlanningScene &scene_info,
                        const ompl::base::SpaceInformationPtr &si) {
            return std::make_shared<MultigoalPrmStar>(plan_time, samples, optimize_segments);
        };
    }) | ranges::to_vector;
}


