
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>

#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/traveling_salesman.h"
#include "../src/general_utilities.h"
#include "../src/SphereShell.h"

#include <range/v3/all.hpp>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <execution>

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    auto [scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] = createMeshBasedAppleTreePlanningSceneMessage("appletree");

    moveit::core::RobotState start_state = stateOutsideTree(drone);

    apples = vectorByOrdering(apples, DIFFICULT_APPLES);

    std::mutex statistics_mtx;
    Json::Value statistics;

    auto ints = boost::irange(0,20);

    std::for_each(std::execution::par, ints.begin(), ints.end(),[
            SPHERE_CENTER=SPHERE_CENTER,
            scene_msg=scene_msg,
            apples=apples,
            drone, start_state,
            &statistics_mtx,
            &statistics
            ](int i){

        GreatcircleDistanceHeuristics gdh(start_state.getGlobalLinkTransform("end_effector").translation(),
                                          GreatCircleMetric(SPHERE_CENTER));

        auto state_space = std::make_shared<DroneStateSpace>(
                ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), 10.0);
        auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);

        auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

        const SphereShell sphereShell(SPHERE_CENTER, 1.8);

        OMPLSphereShellWrapper shell(sphereShell, si);

        std::cout << "Round: " << i << std::endl;

        Json::Value run_stats;

        std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches_naive;
        std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches_optimized;
        std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches_exit_optimized;

        for (const Apple &apple: apples) {

            Json::Value apple_stats;

            auto stateOutside = shell.state_on_shell(apple);

            auto planner = std::make_shared<ompl::geometric::PRMstar>(si);

            auto initial_attempt = planFromStateToApple(*planner, objective, stateOutside->get(), apple, 1.0, false);

            if (initial_attempt) {

                apple_stats["initial_attempt"] = initial_attempt->length();
                approaches_naive.emplace_back(apple, *initial_attempt);

                ompl::geometric::PathGeometric backup = *initial_attempt;
                ompl::geometric::PathSimplifier(si).simplifyMax(*initial_attempt);
                // Simplifying sometimes produces *wild* path lengths...
                // We're resetting the path if the simplifier actually makes the path worse.
                if (backup.length() < initial_attempt->length()) initial_attempt = backup;

                apple_stats["simplified"] = initial_attempt->length();
                approaches_optimized.emplace_back(apple, *initial_attempt);

                optimizeExit(apple, *initial_attempt, objective, shell, si);

                apple_stats["exit_optimized"] = initial_attempt->length();
                approaches_exit_optimized.emplace_back(apple, *initial_attempt);

            } else {
                apple_stats = Json::nullValue;
            }

            run_stats["approach_stats"].append(apple_stats);

            std::cout << "On approach done..." << std::endl;

        }

        std::cout << "Run approaches done." << std::endl;

        ompl::base::ScopedState start(si);
        state_space->copyToOMPLState(start.get(), start_state);

        std::tuple<std::string, std::vector<std::pair<Apple, ompl::geometric::PathGeometric>>> orderings[] = {
                {"naive",          approaches_naive},
                {"optimized",      approaches_optimized},
                {"exit_optimized", approaches_exit_optimized}
        };

        for (const auto &[approach_type, approaches]: orderings) {
            auto fullPath = planFullPath(si, start.get(), shell, optimizeApproachOrder(gdh, *state_space, approaches));
            run_stats["full_paths"][approach_type] = fullPath.length();
        }

        run_stats["apples_visited"] = (int) approaches_naive.size();

        std::scoped_lock lck(statistics_mtx);
        statistics["runs"].append(run_stats);

        std::cout << "Run " << i << " done." << std::endl;
    });

    statistics["apples_total"] = (int) apples.size();

    std::ofstream of;
    of.open("analysis/approach_stats_difficult.json");
    of << statistics;
    of.close();

    std::cout << "Done." << std::endl;

}