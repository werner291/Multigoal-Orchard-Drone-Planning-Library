
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

NewMultiGoalPlanner::PlanResult runPlanner(
        const shared_ptr<DroneStateSpace> &stateSpace,
        const AppleTreePlanningScene &scene_info,
        ompl::base::ScopedState<> &start_state,
        const NewMultiGoalPlannerAllocatorFn &planner_allocator) {

    auto si = loadSpaceInformation(stateSpace, scene_info);
    auto goals = constructNewAppleGoals(si, scene_info.apples);
    auto objective = std::make_shared<DronePathLengthObjective>(si);

    // TODO: Double-check if stuff is being optimized here...
    SingleGoalPlannerMethods ptp(5.0, si, objective);

    return planner_allocator(scene_info, stateSpace)->plan(si, start_state.get(), goals, ptp);
}

ompl::base::ScopedState<> genStartState(const shared_ptr<DroneStateSpace> &stateSpace) {
    auto start_state_moveit = stateOutsideTree(stateSpace->getRobotModel());
    ompl::base::ScopedState<> start_state(stateSpace);
    stateSpace->copyToOMPLState(start_state.get(), start_state_moveit);
    return start_state;
}

void run_planner_experiment(const vector<NewMultiGoalPlannerAllocatorFn> &allocators, const std::string& results_path) {

    auto stateSpace = loadStateSpace();

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    const size_t NUM_RUNS = 100;

    const auto run_indices = ranges::views::iota(0, (int) NUM_RUNS);

    thread_pool pool(8);
    std::vector<std::future<Json::Value>> future_results;

    for (int run_i : run_indices) {
        auto start_state = genStartState(stateSpace);

        for (const auto& planner_allocator: allocators) {
            future_results.push_back(pool.submit([&, planner_allocator=planner_allocator]() -> Json::Value {
                return toJson(runPlanner(stateSpace, scene_info, start_state,planner_allocator));
            }));
        }
    }

    Json::Value statistics;
    for (auto &result: future_results) {
        result.wait();
        statistics.append(result.get());
        std::cout << "Completed run " << statistics.size() << " out of " << future_results.size() << std::endl;
    }

    std::ofstream ofs;
    ofs.open(results_path);
    ofs << statistics;
    ofs.close();
}


