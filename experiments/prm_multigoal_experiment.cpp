

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

    const size_t NUM_RUNS = 10;

    Json::Value statistics;

    mutex result_mutex;

    thread_pool pool(4);

    for (size_t run_i = 0; run_i < NUM_RUNS; ++run_i) {
        pool.push_task([&, run_i=run_i]() {

            auto final_path = planByApples(
                    stateOutsideTree(drone),
                    setupPlanningScene(scene_info.scene_msg, drone),
                    scene_info.apples
                    );

            {
                scoped_lock lock(result_mutex);
                cout << "Run done " << run_i << endl;
                statistics["path_lengths"].append(final_path.length());
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