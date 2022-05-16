
#include "../src/experiment_utils.h"
#include "../src/thread_pool.hpp"
#include "../src/greatcircle.h"
#include "../src/probe_retreat_move.h"
#include "../src/NewKnnPlanner.h"
#include <range/v3/all.hpp>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <fstream>

using namespace ranges;
using namespace std;
namespace og = ompl::geometric;

int main(int argc, char **argv) {

    // Load the drone model
    auto drone = loadRobotModel();

    // initialize the state space and such
    auto stateSpace = make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), TRANSLATION_BOUND);

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    const size_t NUM_RUNS = 1;

    thread_pool pool(8);

    const auto run_indices = ranges::views::iota(0, (int) NUM_RUNS);

    const auto ks = ranges::views::iota(1,3);

    const double PLAN_TIME_PER_APPLE_SECONDS = 5.0;

    GreatCircleMetric gc_metric(scene_info.sphere_center);

    const std::shared_ptr<const OmplDistanceHeuristics> distance_fns[2] = {
        std::make_shared<GreatCircleOmplDistanceHeuristics>(gc_metric, stateSpace),
        std::make_shared<EuclideanOmplDistanceHeuristics>(stateSpace),
    };

    // Mutex
    mutex result_mutex;
    Json::Value statistics;

    auto planner_allocators =
            ranges::views::cartesian_product(distance_fns, ks) |
            views::transform([&](auto tuple) {
                return std::make_shared<NewKnnPlanner>(get<0>(tuple), get<1>(tuple));;
            }) | to_vector;

    for (int run_i : run_indices) {
        auto start_state_moveit = stateOutsideTree(drone);

        for (const auto& planner: planner_allocators) {
            pool.push_task([&,run_i = run_i,planner=planner]() {

                // initialize the state space and such
                ExperimentPlanningContext context;

                auto scene = setupPlanningScene(scene_info.scene_msg, drone);
                auto si = initSpaceInformation(scene, scene->getRobotModel(), stateSpace);
                auto objective = make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

                ompl::base::ScopedState<> start_state(si);

                stateSpace->copyToOMPLState(start_state.get(), start_state_moveit);

                auto goals = scene_info.apples | views::transform([&,si=si](auto&& apple) {

                    auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center);

                    return std::static_pointer_cast<ompl::base::Goal>(goal);
                }) | to_vector;

                auto state_to_goal = [&,objective=objective](const ompl::base::State *a, const ompl::base::GoalPtr b) -> std::optional<og::PathGeometric> {
                    auto prm = std::make_shared<ompl::geometric::PRMstar>(si);
                    return planToGoal(*prm, objective, a, PLAN_TIME_PER_APPLE_SECONDS, true, b);
                };

                auto state_to_state = [&,objective=objective](const ompl::base::State *a, const ompl::base::State *b) -> std::optional<og::PathGeometric> {
                    auto prm = std::make_shared<ompl::geometric::PRMstar>(si);
                    return planFromStateToState(*prm, objective, a, b, PLAN_TIME_PER_APPLE_SECONDS);
                };

                // TODO: Double-check if stuff is being optimized here...
                NewMultiGoalPlanner::PlanResult result = planner->plan(
                        si, start_state.get(), goals,state_to_goal,state_to_state
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

                    cout << "Done " << statistics["path_lengths"].size() << " out of " << (run_indices.size() * planner_allocators.size()) << endl;
                }
            });
        }
    }

    pool.wait_for_tasks();

    std::ofstream ofs;
    ofs.open("analysis/knn_results.json");
    ofs << statistics;
    ofs.close();

    return 0;

}