#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/traveling_salesman.h"
#include "../src/general_utilities.h"
#include "../src/msgs_utilities.h"
#include "../src/ros_utilities.h"
#include "../src/ExperimentVisualTools.h"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <range/v3/all.hpp>
#include <boost/format.hpp>

int main(int argc, char **argv) {

    // Load the drone model
    auto drone = loadRobotModel();

    // Load the apple tree model with some metadata.
    auto[scene_msg, apples, SPHERE_CENTER, SPHERE_RADIUS] =
            createMeshBasedAppleTreePlanningSceneMessage("appletree");

    ExperimentVisualTools evt;

    apples = vectorByOrdering(apples, DIFFICULT_APPLES);

    auto state_space = std::make_shared<DroneStateSpace>(
            ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"), 10.0);
    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

    OMPLSphereShellWrapper shell(SphereShell(SPHERE_CENTER, 1.8), si);

    const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches_naive = planApproaches(apples, objective, shell, si);

    std::cout << "Approaches planned..." << std::endl;

    const std::vector<std::pair<Apple, ompl::geometric::PathGeometric>> approaches_optimized = approaches_naive | ranges::views::transform([&](const auto pair) {
        ompl::geometric::PathGeometric path_copy = pair.second;
        optimizeExit(pair.first, path_copy, objective, shell, si);
        return std::make_pair(pair.first, path_copy);
    }) | ranges::to_vector;

    std::tuple<std::string, std::vector<std::pair<Apple, ompl::geometric::PathGeometric>>> orderings[] = {
            {"naive",     approaches_naive },
            {"optimized", approaches_optimized}
    };

    moveit::core::RobotState start_state = stateOutsideTree(drone);

    GreatcircleDistanceHeuristics gdh(start_state.getGlobalLinkTransform("end_effector").translation(),
                                      GreatCircleMetric(SPHERE_CENTER));

    ompl::base::ScopedState start(si);
    state_space->copyToOMPLState(start.get(), start_state);

    for (const auto &[approach_type, approaches] : orderings) {

        auto fullPath = planFullPath(si, start.get(), shell, optimizeApproachOrder(gdh, *state_space, approaches));

        std::cout << "Full path (" << approach_type << ") length: " << fullPath.length() << std::endl;
        evt.publishPath(si, "/trajectory_"+approach_type, fullPath);
        evt.dumpProjections(approaches, "/apple_to_sphere_projections_" + approach_type);
        evt.dumpApproaches(si, approaches, "/approaches_"+approach_type);
    }

    std::cout << "ready" << std::endl;

    ros::spin();

}



