#ifndef NEW_PLANNERS_SAMPLERWRAPPER_H
#define NEW_PLANNERS_SAMPLERWRAPPER_H

#include <ompl/base/goals/GoalState.h>
#include "InformedRobotStateSampler.h"
#include "DroneStateSampler.h"

/**
 * \brief A wrapper around ompl::base::SpaceSampler that can accept information about the current planning problem.
 *
 * The getSampler() method returns a shared pointer to a SpaceSampler that will be updated with `setStartAndGoal`, which
 * effectively allows control over the sampler after it has been handed off to the OMPL planner, since said planner
 * somewhat tightly-encapsulates the sampler under normal circumstances.
 *
 * Be careful with multi-threaded planners!
 */
class SamplerWrapper {

protected:
    ompl::base::StateSpace *ss_;

public:
    SamplerWrapper(ompl::base::StateSpace *ss);

// OMPL clearly doesn't want us to change the sampler after the setup. Let's do it anyway!
    virtual std::shared_ptr<ompl::base::StateSampler> getSampler() = 0;

    virtual void
    setStartAndGoal(const ompl::base::State *start, const std::shared_ptr<ompl::base::GoalSampleableRegion> &goal) = 0;

    virtual std::string getName() = 0;
};

class UniformSampler : public SamplerWrapper {
public:
    explicit UniformSampler(ompl::base::StateSpace *ss);

    std::shared_ptr<ompl::base::StateSampler> getSampler() override;

    void setStartAndGoal(const ompl::base::State *start,
                         const std::shared_ptr<ompl::base::GoalSampleableRegion> &goal) override;

    std::string getName() override;

};

class InformedGaussian : public SamplerWrapper {

    std::shared_ptr<MakeshiftExponentialSampler> underlying_sampler_;
    double stddev_;

public:
    explicit InformedGaussian(ompl::base::StateSpace *ss, double stddev);

    std::shared_ptr<ompl::base::StateSampler> getSampler() override;

    void setStartAndGoal(const ompl::base::State *start,
                         const std::shared_ptr<ompl::base::GoalSampleableRegion> &goal) override;

    std::string getName() override;

};


#endif //NEW_PLANNERS_SAMPLERWRAPPER_H
