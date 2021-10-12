//
// Created by werner on 12-10-21.
//

#include "../src/BulletContinuousMotionValidator.h"
#include <gtest/gtest.h>
#include "test_utils.h"

moveit::core::RobotModelPtr loadRobotModel() {
    auto urdf = std::make_shared<urdf::Model>();
    assert(urdf->initFile("test_robots/urdf/bot.urdf"));

    auto srdf = std::make_shared<srdf::Model>();
    assert(srdf->initFile(*urdf, "test_robots/config/aerial_manipulator_drone.srdf"));

    return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}

std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<moveit::core::RobotModel> &drone) {
    auto st1 = std::make_shared<moveit::core::RobotState>(drone);
    st1->setToRandomPositions();
    double *st1_values = st1->getVariablePositions();
    st1_values[0] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    st1_values[1] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    st1_values[2] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    return st1;
}