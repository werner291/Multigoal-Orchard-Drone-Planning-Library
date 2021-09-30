
#include <gtest/gtest.h>
#include "../src/BulletContinuousMotionValidator.h"

using namespace robowflex;

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

TEST(BulletContinuousMotionValidatorTest, max_angle) {

    auto robot = loadRobotModel();

    for (size_t i = 0; i < 1000; i++) {

        auto st1 = genRandomState(robot);
        auto st2 = genRandomState(robot);

        double max_angle = BulletContinuousMotionValidator::estimateMaximumRotation(st1, st2);

        for (const auto &lm: robot->getLinkModels()) {
            Eigen::Quaterniond r1(st1->getGlobalLinkTransform(lm).rotation());
            Eigen::Quaterniond r2(st2->getGlobalLinkTransform(lm).rotation());

            double angle = r1.angularDistance(r2);

            ASSERT_LE(angle, max_angle);
        }
    }

}