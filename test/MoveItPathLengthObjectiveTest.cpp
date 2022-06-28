
#include <gtest/gtest.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

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

    Eigen::Vector3d tgt = Eigen::Vector3d(rng.uniformReal(-1.0, 1.0),
                                          rng.uniformReal(-1.0, 1.0),
                                          rng.uniformReal(-1.0, 1.0));

    std::cout << "Target: " << tgt.x() << ", " << tgt.y() << ", " << tgt.z() << std::endl;

    auto goal = std::make_shared<DroneEndEffectorNearTarget>(space_information, 0.0,
                                                             tgt);

    ompl::base::ScopedState goal_sample(space_information);

    pdef->addStartState(start);
    auto objective = std::make_shared<DronePathLengthObjective>(space_information);
    pdef->setOptimizationObjective(objective);
    pdef->setGoal(goal);

    auto informed_sampler = pdef->getOptimizationObjective()->allocInformedStateSampler(pdef, 1);

    for (size_t i = 0; i < 1000; ++i) {

        ompl::base::ScopedState sample(space_information);
        const ompl::base::Cost &maxCost = ompl::base::Cost(objective->costToGo(start.get(), goal.get()).value() + 10.0);//* rng.uniformReal(1.0, 5.0));

        bool success = informed_sampler->sampleUniform(sample.get(), maxCost);

        auto c2go = objective->costToGo(sample.get(), goal.get());
        double pathLowerBound = objective->combineCosts(
                objective->motionCost(start.get(), sample.get()),
                c2go
        ).value();

        ASSERT_TRUE(success);

        ASSERT_LE(pathLowerBound, maxCost.value());

        moveit::core::RobotState robot_state(state_space->getRobotModel());
        state_space->copyToRobotState(robot_state, sample.get());

        Eigen::Vector3d trs = robot_state.getGlobalLinkTransform("base_link").translation();
        std::cout << "Sample: " << trs.x() << ", " << trs.y() << ", " << trs.z() << std::endl;



    }


}