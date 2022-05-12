
#include "../src/experiment_utils.h"
#include "../src/thread_pool.hpp"
#include "../src/greatcircle.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include <range/v3/all.hpp>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <fstream>

int main(int argc, char **argv) {

    // Load the drone model
    auto drone = loadRobotModel();

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    const size_t NUM_RUNS = 10;

    thread_pool pool(8);

    const auto run_indices = ranges::views::iota(0, (int) NUM_RUNS);

    const auto ks = ranges::views::iota(1,5);

    const double PLAN_TIME_PER_APPLE_SECONDS = 10.0;

    using namespace ranges;
    using namespace std;
    namespace og = ompl::geometric;

    GreatCircleMetric gc_metric(scene_info.sphere_center);

    std::function<double(Eigen::Vector3d ,Eigen::Vector3d )> distance_fns[2] = {
        [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return gc_metric.measure(a, b); },
        [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return (a - b).norm(); },
    };

    // Mutex
    mutex result_mutex;
    Json::Value statistics;

    auto carthesian = ranges::views::cartesian_product(run_indices, ks, distance_fns) | to_vector;

    for (auto [run_i,k,distance_fn]: carthesian) {

        pool.push_task([&,run_i = run_i,k=k,distance_fn=distance_fn]() {

            // Send all the parameters to cout
            cout << "run_i: " << run_i << " k: " << k << endl;

            // initialize the state space and such
            auto state_space = std::make_shared<DroneStateSpace>(
                    ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), TRANSLATION_BOUND);

            auto scene = setupPlanningScene(scene_info.scene_msg, drone);
            auto si = initSpaceInformation(scene, scene->getRobotModel(), state_space);
            auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

            auto start_state_moveit = stateOutsideTree(drone);

            ompl::base::ScopedState<> start_state(si);

            state_space->copyToOMPLState(start_state.get(), start_state_moveit);

            ompl::NearestNeighborsGNAT<Eigen::Vector3d> gnat;
            gnat.add(scene_info.apples | views::transform([&](const Apple& a) { return a.center; }) | to_vector);
            gnat.setDistanceFunction(distance_fn);

            og::PathGeometric full_path(si, start_state.get());

            size_t goals_visited = 0;

            while (gnat.size() > 0) {

                auto last_state = full_path.getState(full_path.getStateCount() - 1);
                moveit::core::RobotState last_state_moveit(drone);
                state_space->copyToRobotState(last_state_moveit, last_state);
                last_state_moveit.update(true);

                std::vector<Eigen::Vector3d> nearest_k;

                // Find the k nearest apples.
                gnat.nearestK(last_state_moveit.getGlobalLinkTransform("end_effector").translation(), k, nearest_k);

                std::vector<pair<Eigen::Vector3d,og::PathGeometric>> paths;

                // For all, plan a path
                for (auto &apple: nearest_k) {

                    og::PRMstar planner(si);

                    auto result = planFromStateToApple(planner, objective, last_state, Apple{apple,{0.0,0.0,0.0}}, PLAN_TIME_PER_APPLE_SECONDS, true);

                    if (result) {
                        paths.emplace_back(apple,result.value());
                    }
                }

                if (paths.empty()) {
                    gnat.remove(nearest_k[0]);
                } else {
                    // Pick the shortest path
                    auto shortest = paths[0];
                    for (auto &path: paths) {
                        if (path.second.length() < shortest.second.length()) {
                            shortest = path;
                        }
                    }
                    gnat.remove(shortest.first);
                    full_path.append(shortest.second);

                    goals_visited += 1;
                }
            }

            RobotPath full_path_moveit = omplPathToRobotPath(full_path);

            {
                scoped_lock lock(result_mutex);


                Json::Value run_stats;

                run_stats["run_i"] = run_i;
                run_stats["final_path_length"] = full_path_moveit.length();
                run_stats["goals_visited"] = (int) goals_visited;

                statistics["path_lengths"].append(run_stats);

                cout << "Done " << statistics["path_lengths"].size() << " out of " << carthesian.size() << endl;
            }
        });
    }

    pool.wait_for_tasks();

    std::ofstream ofs;
    ofs.open("analysis/knn_results.json");
    ofs << statistics;
    ofs.close();

    return 0;

}