#include "../experiment_utils.h"
#include "../probe_retreat_move.h"
#include "../planners/MultiGoalPlanner.h"
#include "../DistanceHeuristics.h"
#include "../run_experiment.h"

#include <ompl/geometric/planners/prm/PRM.h>

int main(int argc, char **argv) {

	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

	auto allocators = make_shellpath_allocators(
			{true, false},
			{false, true},
			{false, true},
			{false, true},
			{0.4, 0.5, 1.0}
			);

	run_planner_experiment(allocators,
						   "analysis/shellpath.json",
						   10,
						   {10, 50, 100, 150},
						   std::thread::hardware_concurrency());

}





