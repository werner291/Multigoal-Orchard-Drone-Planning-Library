#ifndef BETWEENMOVEITSTATESINFORMEDSAMPLER_H
#define BETWEENMOVEITSTATESINFORMEDSAMPLER_H

#include <ompl/base/samplers/InformedStateSampler.h>
#include <moveit/robot_state/robot_state.h>

/**
 * @brief Pick a sample S such that a.distance(s) + s.distance(b) <= maxDist
 *
 * @param a Begin state
 * @param b End state
 * @param result The state in which to store the result.
 * @param maxDist Maximum distance.
 */
void sampleBetween(const moveit::core::RobotState& a,
                   const moveit::core::RobotState& b,
                   moveit::core::RobotState& result,
                   double maxDist);

/**
 * @brief An InformedSampler that samples between two RobotStates.
 */
class BetweenMoveItStatesInformedSampler : public ompl::base::InformedSampler
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

    BetweenMoveItStatesInformedSampler(const ompl::base::ProblemDefinitionPtr& probDefn,
                                       unsigned int maxNumberCalls);

};

#endif // BETWEENMOVEITSTATESINFORMEDSAMPLER_H
