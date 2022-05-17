
#include "../src/experiment_utils.h"
#include "../src/thread_pool.hpp"
#include "../src/greatcircle.h"
#include "../src/NewKnnPlanner.h"
#include <range/v3/all.hpp>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <fstream>

using namespace ranges;
using namespace std;
namespace og = ompl::geometric;

typedef std::function<std::shared_ptr<NewMultiGoalPlanner>(
        const AppleTreePlanningScene& scene_info,
        const std::shared_ptr<DroneStateSpace>& stateSpace)>
        NewMultiGoalPlannerAllocatorFn;

std::vector<NewMultiGoalPlannerAllocatorFn> mkPlannerAllocators() {

    std::vector<NewMultiGoalPlannerAllocatorFn> allocators;

    for (size_t k = 1; k <= 3; ++k) {

        allocators.emplace_back([k=k] (const AppleTreePlanningScene& scene_info, const std::shared_ptr<DroneStateSpace>& stateSpace) {
            const GreatCircleMetric gcm(scene_info.sphere_center);
            auto heuristic = std::make_shared<GreatCircleOmplDistanceHeuristics>(gcm, stateSpace);
            auto planner = std::make_shared<NewKnnPlanner>(heuristic, k);
            return std::static_pointer_cast<NewMultiGoalPlanner>(planner);
        });

        allocators.emplace_back([k=k] (const AppleTreePlanningScene& scene_info,const std::shared_ptr<DroneStateSpace>& stateSpace) {
            auto heuristic = std::make_shared<EuclideanOmplDistanceHeuristics>(stateSpace);
            auto planner = std::make_shared<NewKnnPlanner>(heuristic, k);
            return std::static_pointer_cast<NewMultiGoalPlanner>(planner);
        });

    }

    return allocators;
}

void run_planner_experiment(const vector<NewMultiGoalPlannerAllocatorFn> &allocators) {
    // Load the drone model
    auto drone = loadRobotModel();

    // initialize the state space and such
    auto stateSpace = make_shared<DroneStateSpace>(ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), TRANSLATION_BOUND);

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    const size_t NUM_RUNS = 1;

    const auto run_indices = ranges::views::iota(0, (int) NUM_RUNS);

    thread_pool pool(8);

    // Mutex
    mutex result_mutex;
    Json::Value statistics;

    for (int run_i : run_indices) {
        auto start_state_moveit = stateOutsideTree(drone);
        for (const auto& planner_allocator: allocators) {
            pool.push_task([&,run_i = run_i,planner_allocator=planner_allocator]() {

                // initialize the state space and such
                ExperimentPlanningContext context;

                auto scene = setupPlanningScene(scene_info.scene_msg, drone);
                auto si = initSpaceInformation(scene, scene->getRobotModel(), stateSpace);

                ompl::base::ScopedState<> start_state(si);
                stateSpace->copyToOMPLState(start_state.get(), start_state_moveit);

                auto goals = constructNewAppleGoals(si, scene_info.apples);

                SingleGoalPlannerMethods ptp(5.0, si, ompl::base::OptimizationObjectivePtr());

                auto planner = planner_allocator(scene_info, stateSpace);

                // TODO: Double-check if stuff is being optimized here...
                NewMultiGoalPlanner::PlanResult result = planner->plan(
                        si, start_state.get(), goals,ptp
                );

                size_t goals_visited = result.segments_.size();

                ompl::geometric::PathGeometric full_path(si);
                for (const auto &segment: result.segments_) {
                    full_path.append(segment.path_);
                }

                RobotPath full_path_moveit = omplPathToRobotPath(full_path);

                {
                    scoped_lock lock(result_mutex);

                    Json::Value run_stats;

                    run_stats["run_i"] = run_i;
                    run_stats["final_path_length"] = full_path_moveit.length();
                    run_stats["goals_visited"] = (int) goals_visited;
                    statistics["path_lengths"].append(run_stats);

                    cout << "Done " << statistics["path_lengths"].size() << " out of " << (run_indices.size() * allocators.size()) << endl;
                }
            });
        }
    }

    pool.wait_for_tasks();

    std::ofstream ofs;
    ofs.open("analysis/knn_results.json");
    ofs << statistics;
    ofs.close();
}

int main(int argc, char **argv) {

    run_planner_experiment(mkPlannerAllocators());

    return 0;

}