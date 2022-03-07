//
// Created by werner on 24-10-21.
//

#include "InformedRobotStateSampler.h"

#include <utility>
#include "DroneStateConstraintSampler.h"
#include "ompl_custom.h"
//
//void ExpandingHyperspheroidBasedSampler::sampleUniform(ompl::base::State *state) {
//
//    t_ = std::min(t_ * 1.0001, 20.0);
//
//    auto st = takeSample(t_);
//
//    space_->as<DroneStateSpace>()->copyToOMPLState(state, st);
//
//}
//
//void ExpandingHyperspheroidBasedSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) {
//    throw ompl::Exception("Not implemented.");
//}
//
//void ExpandingHyperspheroidBasedSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
//    throw ompl::Exception("Not implemented.");
//}
//
//moveit::core::RobotState ExpandingHyperspheroidBasedSampler::takeSample(double maxDistance) {
//    // This could be cached.
//    double ee_sourced[3] = {ee_source_.x(),ee_source_.y(),ee_source_.z()};
//    double ee_targetd[3] = {ee_target_.x(),ee_target_.y(),ee_target_.z()};
//    auto phs = std::make_shared<ompl::ProlateHyperspheroid>(3, ee_sourced,ee_targetd);
//    phs->setTransverseDiameter((ee_source_-ee_target_).norm() + t_);
//    double ee_interpd[3];
//    rng_.uniformProlateHyperspheroid(phs, ee_interpd);
//    Eigen::Vector3d ee_interp(ee_interpd[0],ee_interpd[1],ee_interpd[2]);
//
//    moveit::core::RobotState onGoal(this->start_state_.getRobotModel());
//    DroneStateConstraintSampler::randomizeUprightWithBase(onGoal);
//    DroneStateConstraintSampler::moveEndEffectorToGoal(onGoal, 0.0, ee_interp);
//
//    return onGoal;
//
//}

//ExpandingHyperspheroidBasedSampler::ExpandingHyperspheroidBasedSampler(const ompl::base::StateSpace *space,
//                                                                       Eigen::Vector3d eeTarget,
//                                                                       const moveit::core::RobotState& startState):
//        StateSampler(space),
//        ee_target_(std::move(eeTarget)),
//        start_state_(startState),
//        ee_source_(startState.getGlobalLinkTransform("base_link").translation()),
//        t_(1.0)
//{}

void MakeshiftExponentialSampler::sample(ompl::base::State *state) {

    ompl::base::ScopedState goal_sample(this->goalRegion->getSpaceInformation());
    goalRegion->sampleGoal(goal_sample.get());

    ompl::base::ScopedState inBetween(this->goalRegion->getSpaceInformation());
    space_->interpolate(start_state, goal_sample.get(), rng_.uniform01(), inBetween.get());

    ompl::base::ScopedState uniform_random_sample(this->goalRegion->getSpaceInformation());
    uniformSampler->sampleUniformNear(state, inBetween.get(), std::abs(rng_.gaussian(0.0, stddev_)));

}

void MakeshiftExponentialSampler::sampleUniform(ompl::base::State *state) {
    sample(state);
}

void MakeshiftExponentialSampler::sampleUniformNear(ompl::base::State *state,
                                                    const ompl::base::State *near,
                                                    double distance) {
    throw ompl::Exception("Not implemented");
}

void MakeshiftExponentialSampler::sampleGaussian(ompl::base::State *state,
                                                 const ompl::base::State *mean,
                                                 double stdDev) {
    throw ompl::Exception("Not implemented");
}

MakeshiftExponentialSampler::MakeshiftExponentialSampler(const ompl::base::StateSpace *space,
                                                         std::shared_ptr<ompl::base::StateSampler> uniformSampler,
                                                         const ompl::base::State *startState,
                                                         std::shared_ptr<const ompl::base::GoalSampleableRegion> goalRegion,
                                                         double stddev)
        : StateSampler(space),
          uniformSampler(std::move(uniformSampler)),
          start_state(startState),
          goalRegion(std::move(goalRegion)),
          stddev_(stddev) {
    std::cout << "New sampler" << std::endl;
}
