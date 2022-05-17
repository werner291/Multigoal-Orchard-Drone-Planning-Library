#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/general_utilities.h"
#include "../src/thread_pool.hpp"
#include "../src/functional_optional.h"
#include "../src/robot_path.h"
#include "../src/NewMultiGoalPlanner.h"
#include "../src/DistanceHeuristics.h"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <range/v3/all.hpp>
#include <utility>
#include <utility>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#define VISUALIZE 0

using namespace ranges;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef std::array<og::PathGeometric, 3> ApproachesToApple;

ApproachesToApple planForOptimizations(const shared_ptr<ompl::base::SpaceInformation> &si,
                                       const shared_ptr<ManipulatorDroneMoveitPathLengthObjective> &objective,
                                       const Apple &apple, const OMPLSphereShellWrapper &shell,
                                       std::optional<ompl::geometric::PathGeometric> naive) {

    auto simplified = optimize(*naive, objective, si);

    auto exit_optimized = optimizeExit(apple, simplified, objective, shell, si);

    array<og::PathGeometric, 3> optimizationLevels{
            *naive, simplified, exit_optimized
    };

    return optimizationLevels;
}

ob::ProblemDefinitionPtr mkProblemDefinitionForApproach(
        const ob::SpaceInformationPtr &si,
        const ob::OptimizationObjectivePtr &objective,
        const Apple &apple,
        const OMPLSphereShellWrapper &shell) {

    auto pdef = make_shared<ompl::base::ProblemDefinition>(si);
    pdef->setOptimizationObjective(objective);
    ob::ScopedState stateOutside(si);
    shell.state_on_shell(apple, stateOutside.get());
    pdef->addStartState(stateOutside);
    pdef->setGoal(make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center));
    return pdef;

}

std::optional<ApproachesToApple>
planApproachesForApple(const std::shared_ptr<ompl::base::SpaceInformation> &si,
                       const std::shared_ptr<ManipulatorDroneMoveitPathLengthObjective> &objective, const Apple &apple,
                       OMPLSphereShellWrapper &shell,
                       const ompl::base::PlannerAllocator &allocPlanner) {

    auto planner = allocPlanner(si);

    auto pdef = mkProblemDefinitionForApproach(si, objective, apple, shell);
    planner->setProblemDefinition(pdef);

    if (auto naive = planExactForPdef(*planner, 1.0, false, pdef)) {
        return {planForOptimizations(si, objective, apple, shell, *naive)};
    } else {
        return {};
    }
}


vector<pair<Apple, og::PathGeometric>>
appleApproachPairs(const vector<Apple> &apples,
                   const vector<std::optional<array<og::PathGeometric, 3>>> &approaches,
                   size_t approach_index) {

    return views::zip(apples, approaches)
           | views::filter([](auto it) { return it.second.has_value(); })
           | views::transform([&](auto it) { return std::make_pair(it.first, (*it.second)[approach_index]); })
           | to_vector;
}

struct RunResult {
    std::vector<std::optional<std::array<RobotPath, 3>>> apple_approaches;
    std::vector<RobotPath> full_paths;
};


Json::Value collectRunStats(const string approach_names[3], const RunResult &result) {
    Json::Value run_stats;

    for (size_t approach_index: boost::irange(0, 3)) {

        auto approach_type = approach_names[approach_index];

        run_stats[approach_type]["final_path_length"] = result.full_paths[approach_index].length();

        for (const auto &item: result.apple_approaches) {
            if (item) {
                run_stats[approach_type]["approach_lengths"].append((*item)[approach_index].length());
            } else {
                run_stats[approach_type]["approach_lengths"].append(NAN);
            }
        }
    }
    return run_stats;
}

int main(int argc, char **argv) {

    // Load the drone model
    auto drone = loadRobotModel();

    // Load the apple tree model with some metadata.
    auto [scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] =
            createMeshBasedAppleTreePlanningSceneMessage("appletree");

    apples = vectorByOrdering(apples, DIFFICULT_APPLES);

#if VISUALIZE
    ExperimentVisualTools evt;
    evt.publishPlanningScene(scene_msg);
#endif

    const int NUM_RUNS = 40;

    const std::string approach_names[]{
            "naive", "optimized", "exit_optimized"
    };

    auto ints = boost::irange(0, NUM_RUNS);

    Json::Value statistics;

    for (auto pair: views::enumerate(results)) {

        size_t run_i = pair.first;
        auto &result = pair.second;

#if VISUALIZE
        for (size_t approach_index: boost::irange(0, 3)) {
            auto approach_pairs = appleApproachPairs(apples, result.apple_approaches, approach_index);
            evt.dumpProjections(approach_pairs, "/apple_to_sphere_projections_" + approach_type + "_" + to_string(run_i));
            evt.dumpApproaches(si, approach_pairs, "/approaches_" + approach_type + "_" + to_string(run_i));
            evt.publishPath(si, "/trajectory_" + approach_type + "_" + to_string(run_i), result.full_paths[approach_index]);
            ros::spinOnce();
        }
#endif // VISUALIZE

        statistics.append(collectRunStats(approach_names, result));

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





