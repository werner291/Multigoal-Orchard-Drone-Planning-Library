#ifndef NEW_PLANNERS_DRONESTATESAMPLER_H
#define NEW_PLANNERS_DRONESTATESAMPLER_H

#include <utility>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/PathGeometric.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/trajectory.h>

class DroneStateSampler : public ompl::base::StateSampler {

public:
    explicit DroneStateSampler(const ompl::base::StateSpace *space);

    void sampleUniform(ompl::base::State *state) override;

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;
};


#endif //NEW_PLANNERS_DRONESTATESAMPLER_H
