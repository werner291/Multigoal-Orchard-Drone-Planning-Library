#ifndef NEW_PLANNERS_DRONESTATESAMPLER_H
#define NEW_PLANNERS_DRONESTATESAMPLER_H

#include <utility>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/PathGeometric.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

class DroneStateSampler : public ompl::base::StateSampler {

    double translation_bound;

public:
    explicit DroneStateSampler(const ompl::base::StateSpace *space, double translationBound = 10.0);

    void sampleUniform(ompl::base::State *state) override;

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;
};


#endif //NEW_PLANNERS_DRONESTATESAMPLER_H
