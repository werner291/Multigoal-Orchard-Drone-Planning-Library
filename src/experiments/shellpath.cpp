#include "../experiment_utils.h"
#include "../probe_retreat_move.h"
#include "../planners/MultiGoalPlanner.h"
#include "../DistanceHeuristics.h"
#include "../run_experiment.h"

#include <ompl/geometric/planners/prm/PRM.h>

int main(int argc, char **argv) {

	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

	const std::vector<bool> &applyShellstateOptimization = {true, false};
	const std::vector<bool> &useImprovisedInformedSampler = {false, true};
	const std::vector<bool> &tryLuckyShots = {false, true};
	const std::vector<bool> &useCostConvergence = {false, true};
	const std::vector<double> &ptpTimeSeconds = {0.4, 0.5, 1.0};
	const std::vector<ShellBuilderAllocatorFn> &shellBuilders = {
			[]() { return std::make_shared<PaddedSphereShellAroundLeavesBuilder>(0.1); }
	};
	std::vector<NewMultiGoalPlannerAllocatorFn> result;
	const std::vector<bool> &applyShellstateOptimization1 = {true, false};
	const std::vector<bool> &useImprovisedInformedSampler1 = {false, true};
	const std::vector<bool> &tryLuckyShots1 = {false, true};
	const std::vector<bool> &useCostConvergence1 = {false, true};
	const std::vector<double> &ptpTimeSeconds1 = {0.4, 0.5, 1.0};
	const std::vector<ShellBuilderAllocatorFn> &shellBuilders1 = {
			[]() { return std::make_shared<PaddedSphereShellAroundLeavesBuilder>(0.1); }};
	std::vector<NewMultiGoalPlannerAllocatorFn> result1;
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
						   std::thread::hardware_concurrency(),
						   0,
						   true);

}





