/*
 * This is the experiment entry code for the ICRA 2023 paper.
 *
 * It is known to work correctly on rev 2ef8176 (tagged fullrun-icra2023)
 */

#include <range/v3/range/conversion.hpp>
#include "../run_experiment.h"

int main(int argc, char **argv) {

    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    std::vector<NewMultiGoalPlannerAllocatorFn> planners;

	// Use known-good parameters here. We don't necessarily need huge diversity since this is our planner.
    auto shellpath_allocators = make_shellpath_allocators(
			{true},
			{true},
			{true},
			{true},
			{0.4, 0.5, 1.0}
			);

    planners.insert(planners.end(), shellpath_allocators.begin(), shellpath_allocators.end());

	// To prove conclusively that our planner is better than this one, we'll want to test a large number of parameters.
    auto tsp_over_prm_allocators = make_tsp_over_prm_allocators(
			{2,3,4,5,6,7,8,9},
			{1.0, 2.0, 5.0, 10.0, 15.0, 20.0},
			{true}
			);

    planners.insert(planners.end(), tsp_over_prm_allocators.begin(), tsp_over_prm_allocators.end());
    
    run_planner_experiment(
            planners,
            "analysis/full_experiment.json",
            10,
            {10, 50, 100, 150},
            std::thread::hardware_concurrency()
    );

    return 0;
}