#include <gtest/gtest.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include "../src/experiment_utils.h"
#include <random_numbers/random_numbers.h>
#include "../src/general_utilities.h"
#include <boost/range/combine.hpp>

double quat_dist(const Eigen::Quaterniond& qs1, const Eigen::Quaterniond& qs2) {

    const double MAX_QUATERNION_NORM_ERROR = 1.0e-10;

    double dq = fabs(qs1.x() * qs2.x() + qs1.y() * qs2.y() + qs1.z() * qs2.z() + qs1.w() * qs2.w());

    if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
        return 0.0;
    return acos(dq);
}

void sampleBetween(const moveit::core::RobotState& a,
                   const moveit::core::RobotState& b,
                   moveit::core::RobotState& result,
                   double maxDist) {

    result = a;

    // Get an RNG for sampling
    ompl::RNG rng;

    std::vector<double> weights(a.getRobotModel()->getActiveJointModels().size());

    // Compute a random weight for each joint, wuch that all weights sum to 1.
    double total = 0.0;
    for (double& w : weights) {
        w = rng.uniform01(); // TODO deal with weighted joints
        total += w;
    }
    for (double& w : weights) {
        w /= total;
    }

    // The distance between a and b limits how much sampling freedom we have.
    double wiggle_room = maxDist - a.distance(b);

    // If this is negative, the user requested a path length lower than the lower bound.
    assert(wiggle_room >= 0.0);

    // Loop through all the joint models with the weight previously-assigned to them.
    for (const auto& w_jm: boost::combine(weights, a.getRobotModel()->getActiveJointModels())) {

        // Destructure the pair
        double w;
        const moveit::core::JointModel *jm;
        boost::tie(w, jm) = w_jm;

        // Get a pointer to the variables.
        const double* pos_a = a.getJointPositions(jm);
        const double* pos_b = b.getJointPositions(jm);

        switch (jm->getType()) {

            case moveit::core::JointModel::JointType::REVOLUTE:
            case moveit::core::JointModel::JointType::PRISMATIC: {

                double middle;
                jm->interpolate(pos_a,pos_b,0.5,&middle);

                result.setJointPositions(jm, {
                    rng.uniformReal(middle - wiggle_room*w/2.0,
                                    middle + wiggle_room*w/2.0)
                });
            }
            break;

            case moveit::core::JointModel::JointType::PLANAR:

                ROS_ERROR("Not implemented.");
                break;

            case moveit::core::JointModel::JointType::FLOATING: {

                size_t dim = jm->getType() == moveit::core::JointModel::JointType::PLANAR ? 2 : 3;

                double linear_weight = rng.uniform01();

                ompl::ProlateHyperspheroid phs(dim, pos_a, pos_b);

                double d = 0.0;
                for (auto d : boost::irange<size_t>(0,dim)) d += std::pow(pos_a[d]-pos_b[d],2);
                phs.setTransverseDiameter(std::sqrt(d) + wiggle_room * w * linear_weight);

                double phs_sample[3];

                // See https://stackoverflow.com/a/28523089 for the shared_ptr hackery.
                rng.uniformProlateHyperspheroid(std::shared_ptr<ompl::ProlateHyperspheroid>(
                        std::shared_ptr<ompl::ProlateHyperspheroid>(), &phs), phs_sample);

                    Eigen::Quaterniond qa(Eigen::Quaterniond(pos_a[6], pos_a[3], pos_a[4], pos_a[5]));
                    Eigen::Quaterniond qb(Eigen::Quaterniond(pos_b[6], pos_b[3], pos_b[4], pos_b[5]));

                    auto sample = sampleInformedQuaternion(qa,qb,quat_dist(qa,qb) + (1.0-w) * wiggle_room);

                    result.setJointPositions(jm,
                                             {
                                             phs_sample[0],phs_sample[1],phs_sample[2],
                                             sample.x(), sample.y(), sample.z(), sample.w()
                                             });

            }

            case moveit::core::JointModel::JointType::FIXED:
                // Do nothing.
                break;

            default:
                ROS_ERROR("Unknown joint type not supported.");
        }
    }

    result.update(true);

}


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
