#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/NewMultiGoalPlanner.h"
#include "../src/DistanceHeuristics.h"
#include "../src/run_experiment.h"

#include <ompl/geometric/planners/prm/PRM.h>

int main(int argc, char **argv) {

    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    run_planner_experiment(
            make_shellpath_allocators(),
            "analysis/shellpath.json",
            10,
            std::thread::hardware_concurrency()
    );

}





