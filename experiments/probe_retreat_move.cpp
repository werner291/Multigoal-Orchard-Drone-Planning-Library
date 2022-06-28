#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/NewMultiGoalPlanner.h"
#include "../src/DistanceHeuristics.h"
#include "../src/ShellPathPlanner.h"
#include "../src/run_experiment.h"

//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/transform.hpp>
#include "../src/MyBitStar.h"

#include <ompl/geometric/planners/prm/PRMstar.h>



int main(int argc, char **argv) {

    bool applyShellstateOptimization[] = {true, false};
    bool useImprovisedInformedSampler[] = {false, true};
    bool tryLuckyShots[] = {false, true};
    bool useCostConvergence[] = {false, true};
    double ptp_time_seconds[] = {0.4, 0.5, 1.0};
//
//    bool applyShellstateOptimization[] = {true, };
//    bool useImprovisedInformedSampler[] = {true};
//    bool tryLuckyShots[] = { true};
//    double ptp_time_seconds[] = {1.0 };

    ompl::base::PlannerAllocator planner_allocators[] = {
            [](const ompl::base::SpaceInformationPtr& si) {
                return std::make_shared<AccessiblePRM>(si);
            },
//            [](const ompl::base::SpaceInformationPtr& si) {
//                return std::make_shared<ompl::geometric::PRMstar>(si);
//            },
    };

    auto allocators =
            ranges::views::cartesian_product(applyShellstateOptimization,
                                             ptp_time_seconds,
                                             planner_allocators,
                                             useImprovisedInformedSampler,
                                             tryLuckyShots,
                                             useCostConvergence)
            | ranges::views::transform([](const auto tuple) -> NewMultiGoalPlannerAllocatorFn {
        return [tuple = tuple](const AppleTreePlanningScene &scene_info,
                               const ompl::base::SpaceInformationPtr &si) {

            auto [shellOptimize, ptp_budget, allocator, improvised_sampler, tryLucky, useCostConvergence] = tuple;

            auto shell = std::make_shared<SphereShell>(
                    scene_info.sphere_center,
                    scene_info.sphere_radius
                    );

            auto heuristics = std::make_shared<GreatCircleOmplDistanceHeuristics>(
                    GreatCircleMetric(scene_info.sphere_center),
                    std::dynamic_pointer_cast<DroneStateSpace>(si->getStateSpace())
                    );

            auto ptp = std::make_shared<SingleGoalPlannerMethods>(
                    ptp_budget,
                    si,
                    std::make_shared<DronePathLengthObjective>(si),
                    allocator,
                    improvised_sampler,
                    tryLucky,
                    useCostConvergence
                    );

            return std::make_shared<ShellPathPlanner>(shell, shellOptimize, heuristics, ptp);
        };
    }) | ranges::to_vector;

    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    run_planner_experiment(
            allocators,
            "analysis/shellpath_adaptive.json",
            10,
            std::thread::hardware_concurrency()
            );

}





