

#include "../src/planning_scene_diff_message.h"
#include "../src/experiment_utils.h"
#include "../src/prm_multigoal.h"
#include "../src/thread_pool.hpp"

#include <range/v3/all.hpp>
#include <fstream>

using namespace std;
using namespace ranges;



int main(int argc, char **argv) {

    // Load the drone model
    auto drone = loadRobotModel();

    // Load the apple tree model with some metadata.
    auto scene_info = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    std::shuffle(scene_info.apples.begin(), scene_info.apples.end(), std::mt19937{std::random_device{}()});

    const size_t NUM_RUNS = 10;

    Json::Value statistics;

    mutex result_mutex;

    thread_pool pool(8);

    const auto run_indices = ranges::views::iota(0, (int) NUM_RUNS);

    const auto samples_per_goal = ranges::views::iota(1,3);

    const double plan_times_seconds[] = {
            1.0, 2.0, 5.0, 10.0, 15.0//, 20.0
    };

    const size_t num_apples[] = { 10, 20, 50, 100, scene_info.apples.size() };

    const auto apples = num_apples
            | views::stride(20)
            | views::transform([&](size_t num_apples) {
                return scene_info.apples | views::take(num_apples) | to_vector;
            });

    const bool optimize_segments_options[] = {false,true};

    auto cartesian = ranges::views::cartesian_product(
            run_indices,
            samples_per_goal,
            plan_times_seconds,
            apples,
            optimize_segments_options
            ) | to_vector;

    std::shuffle(cartesian.begin(), cartesian.end(), std::mt19937{std::random_device{}()});

    std::cout << "Will perform " << cartesian.size()
              << " runs ( run indices: " << run_indices.size()
              << " * samples per goal: " << samples_per_goal.size()
              << " * plan times: " << std::size(plan_times_seconds)
              << " * apples: " << apples.size()
              << " * optimize segments: " << std::size(optimize_segments_options)
              << " )" << std::endl;

    for (auto [run_i,
               num_goal_samples,
               planning_time,
               selected_apples,
               optimize_segments] : cartesian) {

        pool.push_task(
                [&,
                 run_i = run_i,
                 planning_time = planning_time,
                 optimize_segments = optimize_segments,
                 num_goal_samples = num_goal_samples,
                 selected_apples = selected_apples
                         ]() {

                    // Send all the parameters to cout
                    cout << "run_i: " << run_i
                         << " planning_time: " << planning_time
                         << " optimize_segments: " << optimize_segments
                         << " num_goal_samples: " << num_goal_samples
                         << " apples:" << selected_apples.size() << endl;

                    auto plan_result = planByApples(
                            stateOutsideTree(drone),
                            setupPlanningScene(scene_info.scene_msg, drone),
                            selected_apples,
                            planning_time, optimize_segments, num_goal_samples);

                    {
                        scoped_lock lock(result_mutex);


                        Json::Value run_stats;

                        run_stats["run_i"] = run_i;
                        run_stats["final_path_length"] = plan_result.path.length();
                        run_stats["goals_visited"] = (int) plan_result.apple_visits.size();
                        run_stats["planning_time"] = planning_time;
                        run_stats["optimize_segments"] = optimize_segments;
                        run_stats["num_apples"] = (int) selected_apples.size();

                        cout << "Run done " << run_stats << std::endl;

                        statistics["path_lengths"].append(run_stats);

                        cout << "Done " << statistics["path_lengths"].size() << " out of " << cartesian.size() << endl;
                    }
                });
    }

    pool.wait_for_tasks();

    std::ofstream ofs;
    ofs.open("analysis/prm_multigoal_results.json");
    ofs << statistics;
    ofs.close();

    return 0;
}