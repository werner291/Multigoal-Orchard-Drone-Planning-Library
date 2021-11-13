//
// Created by werner on 12-10-21.
//

#include "../src/BulletContinuousMotionValidator.h"
#include <gtest/gtest.h>
#include "test_utils.h"
#include "../src/experiment_utils.h"

std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<moveit::core::RobotModel> &drone) {
    auto st1 = std::make_shared<moveit::core::RobotState>(drone);
    st1->setToRandomPositions();
    double *st1_values = st1->getVariablePositions();
    st1_values[0] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    st1_values[1] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    st1_values[2] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    return st1;
}

std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> genGoals(const ompl::base::SpaceInformationPtr &si) {
    std::vector<std::shared_ptr<ompl::base::GoalSampleableRegion>> goals;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-50.0, 50.0);

    for (int i = 0; i < 100; i++) {
        goals.push_back(std::make_shared<DroneEndEffectorNearTarget>(si, 0.1, Eigen::Vector3d(
                distribution(generator),
                distribution(generator),
                distribution(generator)
        )));
    }
    return goals;
}
