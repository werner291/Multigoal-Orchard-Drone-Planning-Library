#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/general_utilities.h"
#include "../src/msgs_utilities.h"
#include "../src/thread_pool.hpp"
#include "../src/ExperimentVisualTools.h"
#include "../src/functional_optional.h"

#include <execution>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <range/v3/all.hpp>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#define VISUALIZE 1

using namespace ranges;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

shared_ptr<ompl::base::ProblemDefinition>
mkProblemDefinitionForApproach(const ob::SpaceInformationPtr &si,
                               const ob::OptimizationObjectivePtr &objective,
                               const Apple &apple,
                               const OMPLSphereShellWrapper &shell) {
    auto pdef = make_shared<ompl::base::ProblemDefinition>(si);
    pdef->setOptimizationObjective(objective);
    const auto stateOutside = shell.state_on_shell(apple);
    pdef->addStartState(*stateOutside);
    pdef->setGoal(make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center));
    return pdef;
}

std::optional<std::array<og::PathGeometric, 3>>
planApproachesForApple(const std::shared_ptr<ompl::base::SpaceInformation> &si,
                       const std::shared_ptr<ManipulatorDroneMoveitPathLengthObjective> &objective, const Apple &apple,
                       OMPLSphereShellWrapper &shell,
                       const std::function<std::shared_ptr<ompl::geometric::RRTstar>(const std::shared_ptr<ompl::base::SpaceInformation> &)> &allocPlanner) {

    auto pdef = mkProblemDefinitionForApproach(si, objective, apple, shell);

    auto planner = allocPlanner(si);
//    auto planner = make_shared<ompl::geometric::PRMstar>(si);
    planner->setProblemDefinition(pdef);

    std::cout << "Ready for planning" << std::endl;

    if (auto naive = planExactForPdef(*planner, 1.0, false, pdef)) {

        assert(naive->check());

        std::cout << "Hello" << std::endl;

        auto simplified = optimize(*naive, objective, si);
        auto exit_optimized = optimizeExit(apple, simplified, objective, shell, si);
        return {{*naive, simplified, exit_optimized}};
    } else {
        return {};
    }
}

vector<pair<Apple, og::PathGeometric>>
appleApproachPairs(const vector<Apple> &apples, const vector<std::optional<array<og::PathGeometric, 3>>> &approaches,
                   size_t approach_index) {
    return views::zip(apples, approaches)
           | views::filter([](auto it) { return it.second.has_value(); })
           | views::transform([&](auto it) { return std::make_pair(it.first, (*it.second)[approach_index]); })
           | to_vector;
}

int main(int argc, char **argv) {

    // Load the drone model
    auto drone = loadRobotModel();

    // Load the apple tree model with some metadata.
    auto[scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] =
    createMeshBasedAppleTreePlanningSceneMessage("appletree");

//    apples = vectorByOrdering(apples, DIFFICULT_APPLES);

#if VISUALIZE
    ExperimentVisualTools evt;
    evt.publishPlanningScene(scene_msg);
#endif

    thread_pool pool(8);

    const int NUM_RUNS = 10;

    const std::string approach_names[] {
            "naive", "optimized", "exit_optimized"
    };

    struct RunResult {
        std::vector<std::optional<std::array<og::PathGeometric, 3>>> apple_approaches;
        std::vector<og::PathGeometric> full_paths;
    };

    auto ints = boost::irange(0,NUM_RUNS);

    std::mutex result_mutex;
    std::vector<RunResult> results;

    std::for_each(std::execution::par, ints.begin(), ints.end(),
            [&, apples = apples, scene_msg = scene_msg, SPHERE_CENTER = SPHERE_CENTER, SPHERE_RADIUS = SPHERE_RADIUS](
                    int run_i) {

                    auto state_space = std::make_shared<DroneStateSpace>(ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), 10.0);
                    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
                    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

                    auto allocPlanner = [](const shared_ptr<ompl::base::SpaceInformation> &si) {
                        return make_shared<ompl::geometric::RRTstar>(si);
                    };

                    OMPLSphereShellWrapper shell(SphereShell(SPHERE_CENTER, SPHERE_RADIUS), si);

                    auto approaches =
                            apples |
                            views::transform([&](auto apple) {
                                return planApproachesForApple(si, objective, apple, shell, allocPlanner);
                            }) | to_vector;

                    moveit::core::RobotState start_state = stateOutsideTree(drone);

                    GreatcircleDistanceHeuristics gdh(start_state.getGlobalLinkTransform("end_effector").translation(),
                                                      GreatCircleMetric(SPHERE_CENTER));

                    ompl::base::ScopedState start(si);
                    state_space->copyToOMPLState(start.get(), start_state);

                    auto full_paths = views::iota(0, 3)
                                      | views::transform([&](auto nm_i) {
                        auto approach_pairs = appleApproachPairs(apples, approaches, nm_i);
                        auto optimized_pairs = optimizeApproachOrder(gdh, *state_space, approach_pairs);
                        return planFullPath(si, start.get(), shell, optimized_pairs);
                    }) | to_vector;

                    std::cout << "Run done" << std::endl;

                    std::scoped_lock lock(result_mutex);
                    results.push_back({
                            approaches, full_paths
                    });
            });

    Json::Value statistics;

    for (auto pair: views::enumerate(results)) {

        size_t run_i = pair.first;
        auto& result = pair.second;

        Json::Value run_stats;

        for (size_t approach_index: boost::irange(0, 3)) {

            auto approach_pairs = appleApproachPairs(apples, result.apple_approaches, approach_index);
            auto approach_type = approach_names[approach_index];

            auto state_space = std::make_shared<DroneStateSpace>(
                    ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), 5.0);
            auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);

            run_stats[approach_type]["final_path_length"] = result.full_paths[approach_index].length();

            for (const auto &item : result.apple_approaches) {
                if (item) {
                    run_stats[approach_type]["approach_lengths"].append((*item)[approach_index].length());
                } else {
                    run_stats[approach_type]["approach_lengths"].append(NAN);
                }
            }

#if VISUALIZE
            evt.dumpProjections(approach_pairs, "/apple_to_sphere_projections_" + approach_type + "_" + to_string(run_i));
            evt.dumpApproaches(si, approach_pairs, "/approaches_" + approach_type + "_" + to_string(run_i));
            evt.publishPath(si, "/trajectory_" + approach_type + "_" + to_string(run_i), result.full_paths[approach_index]);
            ros::spinOnce();
#endif // VISUALIZE

        }

        statistics.append(run_stats);

    }

    ofstream out;
    out.open("analysis/approach_statistics.json");
    out << statistics;
    out.close();

    std::cout << "done" << std::endl;

#if VISUALIZE
    ros::spin();
#endif

}





