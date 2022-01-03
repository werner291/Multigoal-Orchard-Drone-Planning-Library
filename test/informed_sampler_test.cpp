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

    for (size_t i : boost::irange(0,1000)) {

        Eigen::Vector4d
                ra(rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01()),
                rb(rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01());

        ra.normalize();
        rb.normalize();

        std::cout << ra << std::endl;
        std::cout << rb << std::endl;

        auto up = (0.5 * ra + 0.5 * rb).normalized();
        auto fw = (ra - rb).normalized();

        Eigen::Vector4d p1 = any_perpendicular_of_two(up,fw).normalized();
        Eigen::Vector4d p2 = cross_three(up,fw,p1);

        double c = std::acos(ra.dot(rb));
        double m = c * 1.5;

        std::cout << "c: " << c << " m: " << m << std::endl;

        Eigen::Vector4d xfed;

        if (m+c >= 2.0*M_PI) {
            xfed = Eigen::Vector4d(rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01(),
                   rng.gaussian01());
            xfed.normalize();
        } else {

            std::vector<double> sample_xy(3);
            rng.uniformInBall(1.0, sample_xy);

            double sma = std::tan(m / 4.0);
            double smi = std::tan(std::acos(std::cos(m / 2.0) / std::cos(c / 2.0))/2.0);

            std::cout
                << std::cos(c / 2.0) << ", "
                << std::cos(m / 2.0) << ", "
                << (std::cos(m / 2.0) / std::cos(c / 2.0)) << ", "
                << std::acos(std::cos(m / 2.0) / std::cos(c / 2.0)) << std::endl;

            Eigen::Vector3d sample(sample_xy[0] * sma, sample_xy[1] * smi, sample_xy[2] * smi);

            std::cout << "sma: " << sma << " smi: " << smi << std::endl;
            std::cout << "sample: " << sample << std::endl;

            Eigen::Vector4d on_sphere(
                    2.0 * sample.x() / (1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z()),
                    2.0 * sample.y() / (1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z()),
                    2.0 * sample.z() / (1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z()),
                    (1.0 - sample.x() * sample.x() - sample.y() * sample.y() - sample.z() * sample.z()) /
                    (1.0 + sample.x() * sample.x() + sample.y() * sample.y() + sample.z() * sample.z())
            );

            EXPECT_NEAR(1.0, on_sphere.norm(), 1.0e-10);

            Eigen::Matrix4d xf {
                    {fw.x(),p1.x(),p2.x(),up.x()},
                    {fw.y(),p1.y(),p2.y(),up.y()},
                    {fw.z(),p1.z(),p2.z(),up.z()},
                    {fw.w(),p1.w(),p2.w(),up.w()},
            };

            std::cout << "Xf: " << xf << std::endl;

            xfed = xf * on_sphere;
        }

        std::cout << "Xfed: " << xfed << std::endl;

        EXPECT_NEAR(1.0, xfed.norm(), 1.0e-10);

        EXPECT_GE(
                m + 0.05, std::acos(xfed.dot(ra)) + std::acos(xfed.dot(rb))
        );

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
