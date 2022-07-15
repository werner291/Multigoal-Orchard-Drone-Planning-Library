#include <range/v3/view/concat.hpp>
#include <range/v3/range/conversion.hpp>
#include "../src/run_experiment.h"

int main(int argc, char **argv) {

    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    std::vector<NewMultiGoalPlannerAllocatorFn> planners;

    auto shellpath_allocators = make_shellpath_allocators();
    planners.insert(planners.end(), shellpath_allocators.begin(), shellpath_allocators.end());
    
    auto tsp_over_prm_allocators = make_tsp_over_prm_allocators();
    planners.insert(planners.end(), tsp_over_prm_allocators.begin(), tsp_over_prm_allocators.end());
    
    run_planner_experiment(
            planners,
            "analysis/full_experiment.json",
            5,
            {10, 50, SIZE_MAX},
            std::thread::hardware_concurrency()
    );

    return 0;
}