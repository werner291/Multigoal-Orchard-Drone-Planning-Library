
#include "../utilities/experiment_utils.h"
#include "../src/thread_pool.hpp"
#include "../GreatCircleMetric.h"
#include "../src/NewKnnPlanner.h"
#include "../run_experiment.h"
#include <range/v3/all.hpp>
#include <fstream>

using namespace ranges;
using namespace std;
namespace og = ompl::geometric;

std::vector<NewMultiGoalPlannerAllocatorFn> mkPlannerAllocators() {

    std::vector<NewMultiGoalPlannerAllocatorFn> allocators;

    for (size_t k = 1; k <= 3; ++k) {

        allocators.emplace_back([k=k] (const AppleTreePlanningScene& scene_info, const std::shared_ptr<DroneStateSpace>& stateSpace) {
            const GreatCircleMetric gcm(scene_info.sphere_center);
            auto heuristic = std::make_shared<GreatCircleOmplDistanceHeuristics>(gcm, stateSpace);
            auto planner = std::make_shared<NewKnnPlanner>(heuristic, k);
            return std::static_pointer_cast<NewMultiGoalPlanner>(planner);
        });

        allocators.emplace_back([k=k] (const AppleTreePlanningScene& scene_info,const std::shared_ptr<DroneStateSpace>& stateSpace) {
            auto heuristic = std::make_shared<EuclideanOmplDistanceHeuristics>(stateSpace);
            auto planner = std::make_shared<NewKnnPlanner>(heuristic, k);
            return std::static_pointer_cast<NewMultiGoalPlanner>(planner);
        });

    }

    return allocators;
}


int main(int argc, char **argv) {

    run_planner_experiment(mkPlannerAllocators(), "analysis/knn_results.json", 100, { 0.1, 0.2, 0.5, 1.0, 2.0 });

    return 0;

}