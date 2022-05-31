
#include <gtest/gtest.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "test_utils.h"
#include "../src/experiment_utils.h"

TEST(MoveitPathLengthObjectiveTest, TestInformedSampler) {

    auto robot = loadRobotModel();
    ompl_interface::ModelBasedStateSpaceSpecification spec(robot, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);
    auto space_information = std::make_shared<ompl::base::SpaceInformation>(state_space);

    space_information->setStateValidityChecker([&](const ompl::base::State *state) {
        return true; // Always valid
    });

    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(space_information);

    ompl::base::ScopedState start(space_information);
    start.random();

    ompl::RNG rng;

    auto goal = std::make_shared<DroneEndEffectorNearTarget>(space_information, 0.0,
                                                             Eigen::Vector3d(rng.uniformReal(-1.0, 1.0),
                                                                             rng.uniformReal(-1.0, 1.0),
                                                                             rng.uniformReal(-1.0, 1.0)));

    ompl::base::ScopedState goal_sample(space_information);
    double costLowerBoundEstimate = INFINITY;
    for (size_t i = 0; i < 10000; ++i) {
        goal->sampleGoal(goal_sample.get());
        costLowerBoundEstimate = std::min(costLowerBoundEstimate, state_space->distance(start.get(), goal_sample.get()));
    }

    pdef->addStartState(start);
    auto objective = std::make_shared<DronePathLengthObjective>(space_information);
    pdef->setOptimizationObjective(objective);
    pdef->setGoal(goal);

    auto informed_sampler = pdef->getOptimizationObjective()->allocInformedStateSampler(pdef, 1);

    for (size_t i = 0; i < 100; ++i) {

        ompl::base::ScopedState sample(space_information);
        const ompl::base::Cost &maxCost = ompl::base::Cost(costLowerBoundEstimate * rng.uniformReal(1.0, 5.0));
        informed_sampler->sampleUniform(sample.get(), maxCost);

        double pathLowerBound = objective->combineCosts(
                objective->motionCost(start.get(), sample.get()),
                objective->costToGo(sample.get(), goal.get())
        ).value();

        ASSERT_LE(pathLowerBound,maxCost.value());

    }


}