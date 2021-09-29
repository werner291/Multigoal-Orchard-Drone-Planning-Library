
#include <gtest/gtest.h>
#include "../src/make_robot.h"
#include "../src/BulletContinuousMotionValidator.h"

using namespace robowflex;

std::shared_ptr<moveit::core::RobotState> genRandomState(const std::shared_ptr<Robot> &drone) {
    auto st1 = drone->allocState();
    st1->setToRandomPositions();
    double *st1_values = st1->getVariablePositions();
    st1_values[0] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    st1_values[1] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    st1_values[2] = st1->getRandomNumberGenerator().uniformReal(-100.0, 100.0);
    return st1;
}

TEST(BulletContinuousMotionValidatorTest, max_angle) {

    std::shared_ptr<Robot> drone = make_robot();

    for (size_t i = 0; i < 1000; i++) {

        auto st1 = genRandomState(drone);
        auto st2 = genRandomState(drone);

        double max_angle = BulletContinuousMotionValidator::estimateMaximumRotation(st1, st2);

        for (const auto &lm: drone->getModelConst()->getLinkModels()) {
            Eigen::Quaterniond r1(st1->getGlobalLinkTransform(lm).rotation());
            Eigen::Quaterniond r2(st2->getGlobalLinkTransform(lm).rotation());

            double angle = r1.angularDistance(r2);

            ASSERT_LE(angle, max_angle);
        }
    }

}