

#ifndef NEW_PLANNERS_OMPL_CUSTOM_H
#define NEW_PLANNERS_OMPL_CUSTOM_H

#include <utility>
#include <ompl/base/MotionValidator.h>

class CustomModelBasedStateSpace : public ompl_interface::ModelBasedStateSpace {

    const std::string param_type_ = "custom";
public:
    CustomModelBasedStateSpace(const ompl_interface::ModelBasedStateSpaceSpecification &spec)
            : ModelBasedStateSpace(spec) {}

    unsigned int validSegmentCount(const ompl::base::State *state1, const ompl::base::State *state2) const override {
        return this->distance(state1, state2) / 0.2;
    }

    const std::string &getParameterizationType() const override {
        return param_type_;
    }

};

class StateValidityChecker : public ompl::base::StateValidityChecker {

    robowflex::SceneConstPtr scene_;

public:
    StateValidityChecker(ompl::base::SpaceInformation *si, robowflex::SceneConstPtr scene)
            : ompl::base::StateValidityChecker(si), scene_(std::move(scene)) {
    }

    bool isValid(const ompl::base::State *state) const override;

    double clearance(const ompl::base::State *state) const override;

};

class DroneStateSampler : public ompl::base::StateSampler {

public:
    explicit DroneStateSampler(const ompl::base::StateSpace *space);

    void sampleUniform(ompl::base::State *state) override;

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;
};

class InverseClearanceIntegralObjectiveOMPL : public ompl::base::StateCostIntegralObjective {
public:
    InverseClearanceIntegralObjectiveOMPL(const ompl::base::SpaceInformationPtr &si,
                                          bool enableMotionCostInterpolation);

    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

};

class DroneEndEffectorNearTarget : public ompl::base::GoalSampleableRegion {

    double radius;
    Eigen::Vector3d target;

public:
    DroneEndEffectorNearTarget(const ompl::base::SpaceInformationPtr &si, double radius, const Eigen::Vector3d &target);

    void sampleGoal(ompl::base::State *state) const override;

    [[nodiscard]] unsigned int maxSampleCount() const override;

    double distanceGoal(const ompl::base::State *state) const override;

};

#endif //NEW_PLANNERS_OMPL_CUSTOM_H
