#ifndef INFORMEDMANIPULATORDRONESAMPLER_H
#define INFORMEDMANIPULATORDRONESAMPLER_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/samplers/InformedStateSampler.h>

/**
 * @todo write docs
 */
class InformedManipulatorDroneSampler : public ompl::base::InformedSampler
{

    /// The RobotStates to sample between.
    moveit::core::RobotState st1, st2;

public:

    virtual double getInformedMeasure(const ompl::base::Cost& currentCost) const;

    virtual bool hasInformedMeasure() const;

    virtual bool sampleUniform(ompl::base::State* statePtr,
                               const ompl::base::Cost& minCost,
                               const ompl::base::Cost& maxCost);

    virtual bool sampleUniform(ompl::base::State* statePtr,
                               const ompl::base::Cost& maxCost);

    InformedManipulatorDroneSampler(const ompl::base::ProblemDefinitionPtr& probDefn,
                                    unsigned int maxNumberCalls);

};

bool sampleBetweenUpright(const moveit::core::RobotState& a,
    const moveit::core::RobotState& b,
    moveit::core::RobotState& result,
    double maxDist);

#endif // INFORMEDMANIPULATORDRONESAMPLER_H
