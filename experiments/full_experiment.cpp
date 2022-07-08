#include <range/v3/view/concat.hpp>
#include <range/v3/range/conversion.hpp>
#include "../src/run_experiment.h"

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

void handler(int sig) {
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

int main(int argc, char **argv) {

    signal(SIGSEGV, handler);   // install our handler

    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    std::vector<NewMultiGoalPlannerAllocatorFn> planners;

    auto shellpath_allocators = make_shellpath_allocators();
    planners.insert(planners.end(), shellpath_allocators.begin(), shellpath_allocators.end());

    auto tsp_over_prm_allocators = make_tsp_over_prm_allocators();
    planners.insert(planners.end(), tsp_over_prm_allocators.begin(), tsp_over_prm_allocators.end());

    run_planner_experiment(
            planners,
            "analysis/full_experiment.json",
            10,
            {20, 80, SIZE_MAX},
            std::thread::hardware_concurrency()
    );

    return 0;
}