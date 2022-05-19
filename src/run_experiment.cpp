
#include "run_experiment.h"
#include "experiment_utils.h"
#include "thread_pool.hpp"
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

void run_planner_experiment(const vector<NewMultiGoalPlannerAllocatorFn>& allocators,
                            const std::string &results_path,
                            const int num_runs) {

    auto stateSpace = loadStateSpace();

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    std::vector<std::thread> threads;
    std::mutex mut;

    Json::Value statistics;

    unsigned int concurrency = 1;//std::thread::hardware_concurrency();

    for (size_t thread_id = 0; thread_id < concurrency; ++thread_id) {
        threads.emplace_back([&, thread_id=thread_id, allocators=allocators]() {

            auto si = loadSpaceInformation(stateSpace, scene_info);

            for (size_t run_i = thread_id; run_i < num_runs; run_i += concurrency) {

                auto start_state = genStartState(stateSpace);

                for (const auto& planner_allocator : allocators) {
                    auto planner = planner_allocator(scene_info, stateSpace);
                    auto goals = constructNewAppleGoals(si, scene_info.apples);
                    auto objective = make_shared<DronePathLengthObjective>(si);

                    SingleGoalPlannerMethods ptp(5.0, si, objective);

                    auto start_time = ompl::time::now();
                    auto result = planner->plan(si, start_state.get(), goals, ptp);
                    auto run_time = ompl::time::seconds(ompl::time::now() - start_time);

                    auto plan_result = toJson(result);
                    plan_result["run_time"] = run_time;
                    plan_result["run_index"] = (int) run_i;
                    plan_result["planner_params"] = planner->parameters();
                    plan_result["planner_name"] = planner->name();

                    {
                        std::lock_guard<std::mutex> lock(mut);
                        statistics.append(plan_result);

                    }
                }

                std::cout << "Completed run " << run_i << " out of " << num_runs << std::endl;

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


