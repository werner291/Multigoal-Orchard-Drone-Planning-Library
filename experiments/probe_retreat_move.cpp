#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/NewMultiGoalPlanner.h"
#include "../src/DistanceHeuristics.h"
#include "../src/ShellPathPlanner.h"
#include "../src/run_experiment.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv) {

    std::vector<NewMultiGoalPlannerAllocatorFn> allocators {
        [](const AppleTreePlanningScene& scene_info, const std::shared_ptr<DroneStateSpace>& stateSpace) {
            auto shell = std::make_shared<SphereShell>(scene_info.sphere_center, scene_info.sphere_radius);
            auto heuristics = std::make_shared<GreatCircleOmplDistanceHeuristics>(GreatCircleMetric(scene_info.sphere_center), stateSpace);
            return std::make_shared<ShellPathPlanner>(shell, true,heuristics);
        }
    };

    run_planner_experiment(allocators,"analysis/shellpath.json");

}





