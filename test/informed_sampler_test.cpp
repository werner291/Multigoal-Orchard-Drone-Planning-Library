#include <gtest/gtest.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <random_numbers/random_numbers.h>
#include <boost/range/combine.hpp>
#include "../src/experiment_utils.h"
#include "../src/general_utilities.h"
#include "../src/BetweenMoveItStatesInformedSampler.h"
#include "../src/InformedManipulatorDroneSampler.h"
#include "../src/DroneStateConstraintSampler.h"

TEST(InformedSamplerTest, perpendicular_of_two) {

    ompl::RNG rng;

    Eigen::Vector4d
                a(rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01()),
                b(rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01());

    a.normalize();
    b.normalize();

    auto c = any_perpendicular_of_two(a,b);

    EXPECT_NEAR(a.dot(c), 0.0, 1.0e-10);
    EXPECT_NEAR(b.dot(c), 0.0, 1.0e-10);
    EXPECT_GT(c.norm(), 1.0e-10);

    auto d = cross_three(a,b,c);

    EXPECT_NEAR(d.dot(a), 0.0, 1.0e-10);
    EXPECT_NEAR(d.dot(b), 0.0, 1.0e-10);
    EXPECT_NEAR(d.dot(c), 0.0, 1.0e-10);
}

TEST(InformedSamplerTest, informed_point_on_sphere) {

    ompl::RNG rng;

    // Test with 100 pairs of random quaternions
    for (size_t i : boost::irange(0,10000)) {

        // Generate two random quaternions
        auto ra = Eigen::Quaterniond::UnitRandom();
        auto rb = Eigen::Quaterniond::UnitRandom();

        // Compute the distance.
        double qd = quat_dist(ra,rb);
        EXPECT_GE(M_PI/2.0,qd); // Sanity check: should never be greater than PI/2

        // Pick a maximum distance of at least the given distance.
        // No point in picking anything reater than M_PI since the sum
        // of quaternion distances can never exceed it anyway.
        double m = std::min(qd * rng.uniformReal(1.0,1.5),M_PI);

        // Pick a hundred samples
        for (size_t _i : boost::irange(0,10000)) {
            auto sample = sampleInformedQuaternion(ra, rb, m);

            // Ensure the quaternions are unit norm.
            EXPECT_NEAR(1.0, sample.norm(), 1.0e-10);

            // Make sure the sum of distances doesn't exceed the maximum.
            // TODO: I'm still plenty happy if it exceeds by a tiny amount,
            // since that's a very decent approximation, but I'd like to know
            // why we can't seem to be able to use a margin of 0.001.
            // Just some big numerical errors?
            if (m + 0.01 <= quat_dist(ra,sample)+quat_dist(sample,rb))
                std::cout << quat_dist(ra,sample) << " + " << quat_dist(sample,rb) << std::endl;
            EXPECT_GE(
                    m + 0.01, quat_dist(ra,sample)+quat_dist(sample,rb)
            );
        }

    }

}

TEST(InformedSamplerTest, test_random_state_pairs) {

    auto drone = loadRobotModel();

    moveit::core::RobotState st1(drone),st2(drone),sample(drone);

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution range_distr(1.0,10.0);

    for (size_t i : boost::irange(0,1000)) {

        st1.setToRandomPositions();
        st2.setToRandomPositions();

        double distance = st1.distance(st2);

        double maxDist = distance * range_distr(rng);

        sampleBetween(st1,st2,sample,maxDist);

        EXPECT_LT(st1.distance(sample)+sample.distance(st2),maxDist+0.1);

    }

}

TEST(InformedManipulatorDroneSampler, test_random_state_pairs_upright) {

    auto drone = loadRobotModel();

    moveit::core::RobotState st1(drone),st2(drone),sample(drone);

    ompl::RNG rng;

    for (size_t i : boost::irange(0,1000)) {

        DroneStateConstraintSampler::randomizeUprightWithBase(st1, 20.0);
        DroneStateConstraintSampler::randomizeUprightWithBase(st2, 20.0);

        double distance = st1.distance(st2);

        double maxDist = distance * rng.uniformReal(1.0,10.0);

        sampleBetweenUpright(st1,st2,sample,maxDist);

        EXPECT_LT(st1.distance(sample)+sample.distance(st2),maxDist+0.01);

       EXPECT_EQ((sample.getGlobalLinkTransform("base_link").rotation() *
                  Eigen::Vector3d::UnitZ()).dot(Eigen::Vector3d::UnitZ()), 1.0);

    }

}
