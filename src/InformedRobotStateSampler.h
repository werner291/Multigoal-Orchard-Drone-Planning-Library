//
// Created by werner on 24-10-21.
//

#ifndef NEW_PLANNERS_INFORMEDROBOTSTATESAMPLER_H
#define NEW_PLANNERS_INFORMEDROBOTSTATESAMPLER_H

#include <ompl/base/samplers/InformedStateSampler.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/goals/GoalSampleableRegion.h>

class ExpandingHyperspheroidBasedSampler : public ompl::base::StateSampler {
public:

    ExpandingHyperspheroidBasedSampler(const ompl::base::StateSpace *space,
                                       std::shared_ptr<ompl::base::StateSampler> uniformSampler,
                                       const ompl::base::State *startState,
                                       std::shared_ptr<const ompl::base::GoalSampleableRegion> goalRegion,
                                       double stddev);

    void sample(ompl::base::State *state);

    void sampleUniform(ompl::base::State *state) override;

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

    const ompl::base::State *start_state;
    std::shared_ptr<const ompl::base::GoalSampleableRegion> goalRegion;
    double stddev_;

private:
    const std::shared_ptr<ompl::base::StateSampler> uniformSampler;
    ompl::RNG rng_;

};


#endif //NEW_PLANNERS_INFORMEDROBOTSTATESAMPLER_H
