//
// Created by werner on 26-10-21.
//

#include "InformedRobotStateSampler.h"
#include <utility>
#include "SamplerWrapper.h"

SamplerWrapper::SamplerWrapper(ompl::base::StateSpace *ss) : ss_(ss) {}

std::shared_ptr<ompl::base::StateSampler> UniformSampler::getSampler() {
    return std::make_shared<DroneStateSampler>(ss_);
}

void UniformSampler::setStartAndGoal(const ompl::base::State *start,
                                     const std::shared_ptr<ompl::base::GoalSampleableRegion> &goal) {
    // The uniform sampler ignores start states and goals.
}

UniformSampler::UniformSampler(ompl::base::StateSpace *ss) : SamplerWrapper(ss) {}

std::string UniformSampler::getName() {
    return "uniform";
}

InformedGaussian::InformedGaussian(ompl::base::StateSpace *ss, double stddev) : SamplerWrapper(ss), stddev_(stddev) {}

std::shared_ptr<ompl::base::StateSampler> InformedGaussian::getSampler() {
    if (!underlying_sampler_) {
        throw ompl::Exception("Start and goal have not yet been configured.");
    }
    return underlying_sampler_;
}

void InformedGaussian::setStartAndGoal(const ompl::base::State *start,
                                       const std::shared_ptr<ompl::base::GoalSampleableRegion> &goal) {
    if (!underlying_sampler_) {
        underlying_sampler_.reset(
                new ExpandingHyperspheroidBasedSampler(ss_, std::make_shared<DroneStateSampler>(ss_), start, goal,
                                                       stddev_));
    } else {
        underlying_sampler_->goalRegion = goal;
        underlying_sampler_->start_state = start;
    }
}

std::string InformedGaussian::getName() {
    std::stringstream ss;

    ss << "InfGauss";
    ss << this->stddev_;

    return ss.str();
}
