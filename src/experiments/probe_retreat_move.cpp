#include "../experiment_utils.h"
#include "../probe_retreat_move.h"
#include "../src/MultiGoalPlanner.h"
#include "../DistanceHeuristics.h"
#include "../run_experiment.h"

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





