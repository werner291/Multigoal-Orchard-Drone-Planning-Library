
#include <gtest/gtest.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "test_utils.h"
#include "../src/experiment_utils.h"
#include "../src/MoveitPathLengthObjective.h"

TEST(MoveitPathLengthObjectiveTest, TestInformedSampler) {

    auto robot = loadRobotModel();
    ompl_interface::ModelBasedStateSpaceSpecification spec(robot, "whole_body");
    auto state_space = std::make_shared<DroneStateSpace>(spec);
    auto space_information = std::make_shared<ompl::base::SpaceInformation>(state_space);

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
//        costLowerBoundEstimate = std::min(costLowerBoundEstimate, state_space->distance(start, goal));
    }

    pdef->addStartState(start);
    auto objective = std::make_shared<MoveitPathLengthObjective>(space_information);
    pdef->setOptimizationObjective(objective);
    pdef->setGoal(goal);

    auto informed_sampler = pdef->getOptimizationObjective()->allocInformedStateSampler(pdef, 1);

    assert(informed_sampler->hasInformedMeasure());

    informed_sampler->getInformedMeasure(ompl::base::Cost(rng.uniformReal(0.0, 100.0)));
    informed_sampler->getInformedMeasure(objective->identityCost());
    informed_sampler->getInformedMeasure(objective->infiniteCost());

    for (size_t i = 0; i < 100; ++i) {

        ompl::base::ScopedState sample(space_information);
        informed_sampler->sampleUniform(sample.get(),)

//        ompl::base::Cost(rng.uniformReal(0.0, 100.0))

    }


}