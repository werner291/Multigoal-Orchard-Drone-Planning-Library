#include "BetweenMoveItStatesInformedSampler.h"
#include <moveit/robot_model/joint_model.h>
#include <rosconsole/macros_generated.h>
#include <moveit/robot_state/robot_state.h>
#include "ompl_custom.h"
#include <ompl/base/goals/GoalState.h>
#include <boost/range/combine.hpp>
#include <boost/range/irange.hpp>
#include "general_utilities.h"

double BetweenMoveItStatesInformedSampler::getInformedMeasure(const ompl::base::Cost& currentCost) const
{

    ROS_ERROR("Not implemented.");
    return NAN;
//     auto robot = probDefn_
//         ->getSpaceInformation()
//         ->getStateSpace()
//         ->as<DroneStateSpace>()
//         ->getRobotModel();
//
//     double measure = 1.0;

//     for (const auto& jm: robot->getActiveJointModels())) {
//
//         // Get a pointer to the variables.
//         const double* pos_a = a.getJointPositions(jm);
//         const double* pos_b = b.getJointPositions(jm);
//
//         switch (jm->getType()) {
//
//             case moveit::core::JointModel::JointType::REVOLUTE:
//             case moveit::core::JointModel::JointType::PRISMATIC: {
//                 double joint_d = std::max(0.0,currentCost - st1.distance(st2));
//                 if (jm->getType() == moveit::core::JointModel::JointType::REVOLUTE &&
//                     jm->as<moveit::core::RevoluteJointModel>()->isContinuous() &&
//                     joint_d > M_PI) {
//                          joint_d = M_PI;
//                     }
//             }
//             break;
//
//             case moveit::core::JointModel::JointType::PLANAR:
//
//                 ROS_ERROR("Not implemented.");
//                 break;
//
//             case moveit::core::JointModel::JointType::FLOATING: {
//
//                 size_t dim = jm->getType() == moveit::core::JointModel::JointType::PLANAR ? 2 : 3;
//
//                 double linear_weight = rng.uniform01();
//
//                 ompl::ProlateHyperspheroid phs(dim, pos_a, pos_b);
//
//                 double d = 0.0;
//                 for (auto d : boost::irange<size_t>(0,dim)) d += std::pow(pos_a[d]-pos_b[d],2);
//                 phs.setTransverseDiameter(std::sqrt(d) + wiggle_room * w * linear_weight);
//
//                 d *= phs.getPhsMeasure();
//
//                 double phs_sample[3];
//
//                 Eigen::Quaterniond qa(Eigen::Quaterniond(pos_a[6], pos_a[3], pos_a[4], pos_a[5]));
//                 Eigen::Quaterniond qb(Eigen::Quaterniond(pos_b[6], pos_b[3], pos_b[4], pos_b[5]));
//
//                 double angle = quat_dist(qa);
//                 double minor_angle =
//
//
//             }
//
//             case moveit::core::JointModel::JointType::FIXED:
//                 // Do nothing.
//                 break;
//
//             default:
//                 ROS_ERROR("Unknown joint type not supported.");
//         }
//
//     }
}

bool BetweenMoveItStatesInformedSampler::hasInformedMeasure() const
{
    return false;
}

bool BetweenMoveItStatesInformedSampler::sampleUniform(ompl::base::State* statePtr, const ompl::base::Cost& minCost, const ompl::base::Cost& maxCost)
{
    ROS_ERROR("Not implemented.");
    return false;
}

bool BetweenMoveItStatesInformedSampler::sampleUniform(ompl::base::State* statePtr, const ompl::base::Cost& maxCost)
{

    double maxDist = maxCost.value();
    double lowerBound = st1.distance(st2) > maxCost.value();

    if (maxDist < lowerBound) return false;

    if (!isfinite(maxDist)) {

        maxDist = 1000.0;

      }

    moveit::core::RobotState
    st_sample(space_->as<DroneStateSpace>()->getRobotModel());

    sampleBetween(st1,st2,st_sample,maxDist);

    space_->as<DroneStateSpace>()->copyToOMPLState(statePtr, st_sample);

    return true;
}

BetweenMoveItStatesInformedSampler::BetweenMoveItStatesInformedSampler(const ompl::base::ProblemDefinitionPtr& probDefn, unsigned int maxNumberCalls) : ompl::base::InformedSampler(probDefn, maxNumberCalls),
st1(probDefn->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>()->getRobotModel()),
st2(probDefn->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>()->getRobotModel())
{

    auto space = probDefn->getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

    space->copyToRobotState(st1, probDefn->getStartState(0));
    space->copyToRobotState(st2, probDefn->getGoal()->as<ompl::base::GoalState>()->getState());
}

void sampleBetween(const moveit::core::RobotState& a,
    const moveit::core::RobotState& b,
    moveit::core::RobotState& result,
    double maxDist) {

        assert(isfinite(maxDist));

        //         std::cout << "maxDist: " << maxDist << std::endl;

        result = a;

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
        double wiggle_room = maxDist - a.distance(b);

                std::cout << "wiggle room: " << wiggle_room << std::endl;


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
