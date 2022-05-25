#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/NewMultiGoalPlanner.h"
#include "../src/DistanceHeuristics.h"
#include "../src/ShellPathPlanner.h"
#include "../src/run_experiment.h"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/transform.hpp>

int main(int argc, char **argv) {

    bool applyShellstateOptimization[] = {false,true};
    double ptp_time_seconds[] = {0.1, 0.2, 0.5, 1.0, 2.0, 5.0};

    auto allocators = ranges::views::cartesian_product(applyShellstateOptimization, ptp_time_seconds)
        | ranges::views::transform([](const auto tuple) -> NewMultiGoalPlannerAllocatorFn {
        return [tuple = tuple](const AppleTreePlanningScene &scene_info,
                               const ompl::base::SpaceInformationPtr &si) {

            auto shell = std::make_shared<SphereShell>(
                    scene_info.sphere_center,
                    scene_info.sphere_radius
                    );

            auto heuristics = std::make_shared<GreatCircleOmplDistanceHeuristics>(
                    GreatCircleMetric(scene_info.sphere_center),
                    std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace())
                    );

            auto ptp = std::make_shared<SingleGoalPlannerMethods>(
                std::get<1>(tuple), si,
                std::make_shared<DronePathLengthObjective>(si),
                [](ompl::base::SpaceInformationPtr si) {
                    return std::make_shared<ompl::geometric::PRM>(si);
                });

            return std::make_shared<ShellPathPlanner>(
                    shell,
                    std::get<0>(tuple),
                    heuristics,
                    ptp
                    );
        };
    }) | ranges::to_vector;

    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    run_planner_experiment(allocators, "analysis/shellpath.json", 50);

}





