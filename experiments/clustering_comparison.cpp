
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include "../src/experiment_utils.h"
#include "../test/test_utils.h"
#include "../src/ManipulatorDroneMoveitPathLengthObjective.h"

int main(int argc, char **argv) {

    auto drone = loadRobotModel();

    auto[scene,apples] = createWallApplePlanningScene(drone);

    ompl_interface::ModelBasedStateSpaceSpecification spec(drone, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);

    auto si = initSpaceInformation(scene, drone, state_space);

    static const int SAMPLES_PER_GOAL = 20;

    auto goals = constructAppleGoals(si, apples);

    auto goal_samples = clustering::takeInitialSamples(goals, si, SAMPLES_PER_GOAL);

    auto prms = std::make_shared<ompl::geometric::AITstar>(si);
    auto pathLengthObjective = std::make_shared<ManipulatorDroneMoveitPathLengthObjective>(si);
    auto sampler = std::make_shared<InformedGaussian>(state_space.get(), 2.5);

    PointToPointPlanner ptp(prms, pathLengthObjective, sampler);

    clustering::NearestKPreselection preselection;
    clustering::SelectByExponentialRadius postselection({0.1, 1.5});

    auto clusters = clustering::buildClusters(ptp, goal_samples, preselection, postselection);



}