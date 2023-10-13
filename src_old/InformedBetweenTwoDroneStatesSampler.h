#ifndef INFORMEDMANIPULATORDRONESAMPLER_H
#define INFORMEDMANIPULATORDRONESAMPLER_H

#include <moveit/robot_state/robot_state.h>
#include <ompl/base/samplers/InformedStateSampler.h>

/**
 * @todo write docs
 */
class InformedBetweenTwoDroneStatesSampler : public ompl::base::InformedSampler
{

    /// The RobotStates to sample between.
    moveit::core::RobotState st1, st2;

public:

    double getInformedMeasure(const ompl::base::Cost& currentCost) const override;

    bool hasInformedMeasure() const override;

    bool sampleUniform(ompl::base::State* statePtr,
                               const ompl::base::Cost& minCost,
                               const ompl::base::Cost& maxCost) override;

    bool sampleUniform(ompl::base::State* statePtr,
                               const ompl::base::Cost& maxCost) override;

    InformedBetweenTwoDroneStatesSampler(const ompl::base::ProblemDefinitionPtr& probDefn,
                                         unsigned int maxNumberCalls);

};

class InformedBetweenDroneStateAndTargetSampler : public ompl::base::InformedSampler
{
    moveit::core::RobotState st1;
    Eigen::Vector3d target;

public:

    double getInformedMeasure(const ompl::base::Cost& currentCost) const override;

    bool hasInformedMeasure() const override;

    bool sampleUniform(ompl::base::State* statePtr,const ompl::base::Cost& minCost,const ompl::base::Cost& maxCost) override;

    bool sampleUniform(ompl::base::State* statePtr,const ompl::base::Cost& maxCost) override;

    InformedBetweenDroneStateAndTargetSampler(const ompl::base::ProblemDefinitionPtr& probDefn,
                                             unsigned int maxNumberCalls);

};

bool sampleBetweenUpright(const moveit::core::RobotState& a,
    const moveit::core::RobotState& b,
    moveit::core::RobotState& result,
    double maxDist);

#endif // INFORMEDMANIPULATORDRONESAMPLER_H
