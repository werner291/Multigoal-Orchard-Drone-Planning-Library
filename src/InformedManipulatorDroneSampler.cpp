#include "InformedManipulatorDroneSampler.h"
#include "ompl_custom.h"
#include <ompl/base/goals/GoalState.h>
#include <boost/range/combine.hpp>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

double InformedManipulatorDroneSampler::getInformedMeasure(const ompl::base::Cost& currentCost) const{
    ROS_ERROR("Not implemented.");
    return NAN;
}

    bool InformedManipulatorDroneSampler::hasInformedMeasure() const {
        return false;
    }

    bool InformedManipulatorDroneSampler::sampleUniform(ompl::base::State* statePtr,
                               const ompl::base::Cost& minCost,
                               const ompl::base::Cost& maxCost) {
        ROS_ERROR("Not implemented.");
        return false;
    }

    bool InformedManipulatorDroneSampler::sampleUniform(ompl::base::State* statePtr,
                               const ompl::base::Cost& maxCost) {

        auto space = probDefn_->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

        moveit::core::RobotState st(space->getRobotModel());

        bool result = sampleBetweenUpright(st1,st2,st,maxCost.value());

        if (result) space->copyToOMPLState(statePtr, st);
        // else, do nothing but still return false.

        return result;


    }


InformedManipulatorDroneSampler::InformedManipulatorDroneSampler(const ompl::base::ProblemDefinitionPtr& probDefn, unsigned int maxNumberCalls): ompl::base::InformedSampler(probDefn, maxNumberCalls),
st1(probDefn->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>()->getRobotModel()),
st2(probDefn->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>()->getRobotModel())
{
    auto space = probDefn_->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

    space->copyToRobotState(st1, probDefn->getStartState(0));
    space->copyToRobotState(st2, probDefn->getGoal()->as<ompl::base::GoalState>()->getState());
}


bool sampleBetweenUpright(const moveit::core::RobotState& a,
    const moveit::core::RobotState& b,
    moveit::core::RobotState& result,
    double maxDist) {

        assert(isfinite(maxDist));

        // Get an RNG for sampling
        ompl::RNG rng;

        std::vector<double> weights(a.getRobotModel()->getActiveJointModels().size());

        // Compute a random weight for each joint, wuch that all weights sum to 1.
        double total = 0.0;
        for (double& w : weights) {
            w = rng.uniform01();                            // TODO deal with weighted joints
            total += w;
          }
        for (double& w : weights) {
            w /= total;
          }

        // The distance between a and b limits how much sampling freedom we have.
        // Max with 0.0 since AIT* gets a bit over-enthusiastic with the maxCost requests.
        // I'd return false, but then this line https://github.com/ompl/ompl/blob/96eb89e51d84bbc75093409ce186e6826c93ec5a/src/ompl/geometric/planners/informedtrees/aitstar/src/ImplicitGraph.cpp#L339 gets stuck.
        double wiggle_room = std::max(0.0,maxDist - a.distance(b));

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

                    double linear_weight = rng.uniform01();

                    ompl::ProlateHyperspheroid phs(3, pos_a, pos_b);

                    double d = std::sqrt(
                        std::pow(pos_a[0]-pos_b[0],2) +
                        std::pow(pos_a[1]-pos_b[1],2) +
                        std::pow(pos_a[2]-pos_b[2],2)
                      );

                    phs.setTransverseDiameter(d + wiggle_room * w * linear_weight);

                    double phs_sample[3];

                    // See https://stackoverflow.com/a/28523089 for the shared_ptr hackery.
                    rng.uniformProlateHyperspheroid(std::shared_ptr<ompl::ProlateHyperspheroid>(
                        std::shared_ptr<ompl::ProlateHyperspheroid>(), &phs), phs_sample);

                    Eigen::EulerAnglesXYZd ra = Eigen::Quaterniond(pos_a[6], pos_a[3], pos_a[4], pos_a[5]);
                    Eigen::EulerAnglesXYZd rb = Eigen::Quaterniond(pos_b[6], pos_b[3], pos_b[4], pos_b[5]);

                    assert(ra.alpha() == 0.0 && ra.beta() == 0.0);
                    assert(rb.alpha() == 0.0 && rb.beta() == 0.0);

                    double middle = (std::max(ra.gamma(), rb.gamma()) + std::min(ra.gamma(), rb.gamma())) / 2.0;
                    double radius = (std::max(ra.gamma(), rb.gamma()) - std::min(ra.gamma(), rb.gamma())) / 2.0;

                    assert(radius >= 0.0);
                    assert(radius <= M_PI);
                    if (radius > M_PI/2.0) {
                        middle += M_PI;
                        radius = M_PI - radius;
                    }

                    double sample_radius = radius + wiggle_room * w * (1.0-linear_weight)/2.0;
                    assert(sample_radius >= 0.0);

                    Eigen::Quaterniond qs = Eigen::EulerAnglesXYZd(0,0,
                        rng.uniformReal(middle - sample_radius,middle + sample_radius));

                    result.setJointPositions(jm, {
                        phs_sample[0],phs_sample[1],phs_sample[2],
                        qs.x(),qs.y(),qs.z(),qs.w()
                    });

                } break;

                case moveit::core::JointModel::JointType::FIXED:
                    // Do nothing.
                    break;

                default:
                    ROS_ERROR("Unknown joint type not supported.");
              }
          }

        result.update(true);

        return true;

      }
