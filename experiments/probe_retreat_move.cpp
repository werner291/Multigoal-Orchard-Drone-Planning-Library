
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>
#include <ompl/geometric/PathSimplifier.h>
#include <utility>

#include "../src/experiment_utils.h"
#include "../src/probe_retreat_move.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"
#include "../src/traveling_salesman.h"
#include "../src/SphereShell.h"
#include "../src/general_utilities.h"


int main(int argc, char** argv) {

    auto drone = loadRobotModel();

    auto[scene_msg, apples] = createMeshBasedAppleTreePlanningSceneMessage();

    moveit::core::RobotState start_state = stateOutsideTree(drone);

    std::shuffle(apples.begin(), apples.end(), std::mt19937(std::random_device()()));
    apples.resize(10);

    const Eigen::Vector3d sphere_center(0.0,0.0,2.2);

    std::vector<Apple> apples_in_order = vectorByOrdering(apples, ORToolsOrderingStrategy().apple_ordering(apples, GreatcircleDistanceHeuristics(
            start_state.getGlobalLinkTransform("end_effector").translation(),
            GreatCircleMetric(sphere_center)
    )));

    auto state_space = std::make_shared<DroneStateSpace>(ompl_interface::ModelBasedStateSpaceSpecification(drone, "whole_body"));
    auto si = initSpaceInformation(setupPlanningScene(scene_msg, drone), drone, state_space);
    auto objective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);

    ompl::base::ScopedState start(si);
    state_space->copyToOMPLState(start.get(), start_state);

    OMPLSphereShellWrapper shell(SphereShell(sphere_center, 4), ompl::base::SpaceInformationPtr());
    const auto approaches = planApproaches(apples_in_order, objective, shell, si);
    auto fullPath = planFullPath(si, start.get(), shell, approaches);
    auto moveit_trajectory = omplPathToRobotTrajectory(drone, state_space, fullPath);

    dumpToROS(scene_msg, moveit_trajectory);

}

