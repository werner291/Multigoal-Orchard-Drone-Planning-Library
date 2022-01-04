#include <gtest/gtest.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include "../src/experiment_utils.h"
#include <random_numbers/random_numbers.h>
#include "../src/general_utilities.h"

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

    ompl::RNG rng;

    for (const auto jm: a.getRobotModel()->getActiveJointModels()) {

        const double* pos_a = a.getJointPositions(jm);
        const double* pos_b = b.getJointPositions(jm);

        switch (jm->getType()) {

            case moveit::core::JointModel::JointType::REVOLUTE:
            case moveit::core::JointModel::JointType::PRISMATIC: {
                double middle = (pos_a[0] + pos_b[0]) / 2.0;

                result.setJointPositions(jm, {rng.uniformReal(middle - maxDist / 2.0, middle + maxDist / 2.0)});
            }
                break;

            case moveit::core::JointModel::JointType::PLANAR: {
                ompl::ProlateHyperspheroid phs(2, pos_a, pos_b);
                phs.setTransverseDiameter(maxDist);

                double phs_result[2];

                // See https://stackoverflow.com/a/28523089 for the shared_ptr hackery.
                rng.uniformProlateHyperspheroid(std::shared_ptr<ompl::ProlateHyperspheroid>(
                        std::shared_ptr<ompl::ProlateHyperspheroid>(), &phs),
                                                phs_result);

                double theta_middle = (pos_a[2] + pos_b[2]) / 2.0;

                result.setJointPositions(jm, {
                        phs_result[0], phs_result[1],
                    rng.uniformReal(theta_middle - maxDist / 2.0, theta_middle + maxDist / 2.0)
                });


            }

break;
            case moveit::core::JointModel::JointType::FLOATING: {
                ompl::ProlateHyperspheroid phs(3, pos_a, pos_b);
                phs.setTransverseDiameter(maxDist);

                double phs_result[3];

                // See https://stackoverflow.com/a/28523089 for the shared_ptr hackery.
                rng.uniformProlateHyperspheroid(std::shared_ptr<ompl::ProlateHyperspheroid>(
                                                        std::shared_ptr<ompl::ProlateHyperspheroid>(), &phs),
                                                phs_result);



                double quat_middle[4];

//                result.setJointPositions(jm, {
//                        phs_result[0], phs_result[1],
//                        rng.uniformReal(theta_middle - maxDist / 2.0, theta_middle + maxDist / 2.0)
//                });
            }
break;
            case moveit::core::JointModel::JointType::FIXED:
            {
                // Fixed joint, not sampling.
            }
break;
            default:
                ROS_ERROR("Unknown joint type not supported.");

        }
    }

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

    for (size_t i : boost::irange(0,100)) {

        auto ra = Eigen::Quaterniond::UnitRandom();
        auto rb = Eigen::Quaterniond::UnitRandom();

        double qd = quat_dist(ra,rb);
        EXPECT_GE(M_PI/2.0,qd);
        double m = std::min(qd * rng.uniformReal(1.0,1.5),M_PI);

        for (size_t i : boost::irange(0,100)) {
            auto sample = sampleInformedQuaternion(ra, rb, m);

            EXPECT_NEAR(1.0, sample.norm(), 1.0e-10);

            EXPECT_GE(
                    m + 0.05, quat_dist(ra,sample)+quat_dist(sample,rb)
            );
        }

    }

}

TEST(InformedSamplerTest, informed_so3_sampling_test) {

    /*
     * acos(qa ⋅ qs) + acos(qb ⋅ qs) = m
     *
     * cos(acos(qa ⋅ qs) + acos(qb ⋅ qs)) = cos(m)
     *
     * cos(acos(qa ⋅ qs)) * cos(acos(qb ⋅ qs)) - sin(acos(qa ⋅ qs)) * sin(acos(qb ⋅ qs)) = cos(m)
     *
     * (qa ⋅ qs) * (qb ⋅ qs) - sqrt(1-(qa ⋅ qs)^2) * sqrt(1-(qa ⋅ qs)^2) = cos(m)
     *
     * A * B - sqrt(1-A^2) * sqrt(1-B^2) = cos(m)
     *      with A = qa ⋅ qs, B = qb ⋅ qs
     *      so, qs = A ⋅/ qa, qs = B ⋅/ qa
     *
     */
//
//    Eigen::Vector3d ra(1.0,1.0,1.0), rb(-2.0,0.0,0.0);
//
//    Eigen::Quaterniond
//        ra(Eigen::AngleAxisd(ra.norm(), ra.normalized())),
//        rb(Eigen::AngleAxisd(rb.norm(), rb.normalized()));
//
//    double max_rot = ra.
//
//    ompl::RNG rng;
//
//    for (size_t i : boost::irange(0,10000)) {
//        std::vector<double> sample_aa(3);
//        rng.uniformInBall(M_PI, sample_aa);
//        Eigen::AngleAxisd sample_rot(sample_aa);
//
//
//    }

}

TEST(InformedSamplerTest, test_random_state_pairs) {

    auto drone = loadRobotModel();

    moveit::core::RobotState st1(drone),st2(drone),sample(drone);

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution range_distr(1.0,10.0);

    for (size_t i : boost::irange(0,100)) {

        st1.setToRandomPositions();
        st2.setToRandomPositions();

        double distance = st1.distance(st2);

        double maxDist = distance * range_distr(rng);

        sampleBetween(st1,st2,sample,maxDist);

        EXPECT_LT(st1.distance(sample)+sample.distance(st2),maxDist);

    }

}
