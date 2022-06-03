
#include "run_experiment.h"
#include "experiment_utils.h"
#include <range/v3/all.hpp>
#include <fstream>

using namespace std;

std::shared_ptr<DroneStateSpace> loadStateSpace() {
    // Load the drone model
    auto drone = loadRobotModel();

    // initialize the state space and such
    auto stateSpace = make_shared<DroneStateSpace>(ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), TRANSLATION_BOUND);

    return stateSpace;
}

ompl::base::SpaceInformationPtr loadSpaceInformation(
        const std::shared_ptr<DroneStateSpace> &stateSpace,
        const AppleTreePlanningScene& scene_info) {
    auto scene = setupPlanningScene(scene_info.scene_msg, stateSpace->getRobotModel());
    auto si = initSpaceInformation(scene, scene->getRobotModel(), stateSpace);
    return si;
}

Json::Value toJson(const NewMultiGoalPlanner::PlanResult& result) {
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

void run_planner_experiment(const std::vector<NewMultiGoalPlannerAllocatorFn> &allocators,
                            const std::string &results_path,
                            const int num_runs) {

    auto stateSpace = loadStateSpace();

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    std::vector<std::thread> threads;
    std::mutex mut;

    Json::Value statistics;

    unsigned int concurrency = 1;//std::thread::hardware_concurrency();

    auto start_states =
            ranges::views::iota(0, num_runs)
            | ranges::views::transform([&](const auto &i) { return std::make_pair(i,genStartState(stateSpace)); });

    auto tasks = ranges::views::cartesian_product(
            allocators,
            start_states
            ) | ranges::to_vector;

    std::shuffle(tasks.begin(), tasks.end(), std::mt19937(std::random_device()()));

    size_t num_tasks = tasks.size();
    size_t current_task = 0;

    for (size_t thread_id = 0; thread_id < concurrency; ++thread_id) {
        threads.emplace_back([&]() {

            auto si = loadSpaceInformation(stateSpace, scene_info);

            while (true) {

                size_t thread_current_task;

                {
                    std::lock_guard<std::mutex> lock(mut);
                    if (current_task >= num_tasks) {
                        return;
                    }
                    thread_current_task = current_task++;
                }

                std::cout << "Starting task " << thread_current_task << " of " << num_tasks << std::endl;

                const auto& [planner_allocator, start_state_pair] = tasks[current_task - 1];
                const auto& [run_i, start_state] = start_state_pair;

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

    for (auto& thread : threads) {
        thread.join();
    }

    std::ofstream ofs;
    ofs.open(results_path);
    ofs << statistics;
    ofs.close();
}


