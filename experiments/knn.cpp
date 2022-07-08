
#include "../src/experiment_utils.h"
#include "../src/thread_pool.hpp"
#include "../src/greatcircle.h"
#include "../src/NewKnnPlanner.h"
#include "../src/run_experiment.h"
#include <range/v3/all.hpp>
#include <fstream>

using namespace ranges;
using namespace std;
namespace og = ompl::geometric;

int main(int argc, char **argv) {

    run_planner_experiment(mkPlannerAllocators(), "analysis/knn_results.json", 100, { 0.1, 0.2, 0.5, 1.0, 2.0 });

    return 0;

}