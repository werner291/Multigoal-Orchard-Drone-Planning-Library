

#include "../src/planning_scene_diff_message.h"
#include "../src/experiment_utils.h"
#include "../src/MultigoalPrmStar.h"
#include "../src/run_experiment.h"

#include <range/v3/all.hpp>

using namespace std;
using namespace ranges;

int main(int argc, char **argv) {


    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    run_planner_experiment(
            make_tsp_over_prm_allocators(),
            "analysis/prm_multigoal_experiment.json",
            50,
            std::thread::hardware_concurrency()
            );

    return 0;
}