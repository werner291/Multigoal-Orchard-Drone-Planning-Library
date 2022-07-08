#include "../src/run_experiment.h"

int main(int argc, char **argv) {


//    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    run_planner_experiment(
            make_robo_tsp_allocators(),
            "analysis/RoboTSP.json",
            2,
            {SIZE_MAX},
            std::thread::hardware_concurrency()
    );

    return 0;
}