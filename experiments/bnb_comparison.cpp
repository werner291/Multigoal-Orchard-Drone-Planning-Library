
#include <cstddef>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include "../src/experiment_utils.h"
#include "../src/json_utils.h"
#include <fstream>
#include <filesystem>
#include <condition_variable>
#include <random>

int main(int argc, char **argv) {

    // Get the drone model from URDF and SRDF
    auto drone = loadRobotModel();

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");

    // Load the scene data
    const Json::Value trees_data = jsonFromGzipFile("test_robots/trees/trees_2021-11-20T12:21:28Z.json.gzip");

    // Limit ourselves to a subset of all trees.
    const size_t MAX_TREES = 5;

    // Create a state space for the drone.
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    auto tree_data = *treeSceneFromJson(trees_data[0]);

    // Build the space information, planning scene, etc...
    auto planning_scene = constructPlanningScene(tree_data, drone);
    auto si = initSpaceInformation(planning_scene, drone, state_space);
    auto pathLengthObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);

    std::shared_ptr<SamplerWrapper> sampler;
    auto prms = std::make_shared<ompl::geometric::PRMstar>(si);

    // Convert the apples into sampleable goal regons.
    auto goals = constructAppleGoals(si, tree_data.apples);

    // Destructor for State* pointers that uses the state space.
    auto destroyState = [&](ompl::base::State *st) {
        state_space->freeState(st);
    };

    // Similar to ScopedState, but I trust this implementation more in regard to move semantics
    typedef std::unique_ptr<ompl::base::State, decltype(destroyState)> StateUptr;

    // Initialize the point-to-point planner.
    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);


    return 0;
}
