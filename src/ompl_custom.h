

#ifndef NEW_PLANNERS_OMPL_CUSTOM_H
#define NEW_PLANNERS_OMPL_CUSTOM_H

#include <utility>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/PathGeometric.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include "DroneStateSampler.h"


class StateValidityChecker : public ompl::base::StateValidityChecker {

    const planning_scene::PlanningScenePtr scene_;

public:
    StateValidityChecker(ompl::base::SpaceInformation *si, planning_scene::PlanningScenePtr scene)
            : ompl::base::StateValidityChecker(si), scene_(std::move(scene)) {
    }

    bool isValid(const ompl::base::State *state) const override;

    double clearance(const ompl::base::State *state) const override;

};

class DroneStateSpace : public ompl_interface::ModelBasedStateSpace {

    const std::string param_type_ = "custom";
public:
    explicit DroneStateSpace(const ompl_interface::ModelBasedStateSpaceSpecification &spec)
            : ModelBasedStateSpace(spec) {}

    unsigned int validSegmentCount(const ompl::base::State *state1, const ompl::base::State *state2) const override {
        return (int) std::ceil(this->distance(state1, state2) / 0.2);
    }

    [[nodiscard]] const std::string &getParameterizationType() const override {
        return param_type_;
    }

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override {
        return std::make_shared<DroneStateSampler>(this);
    }

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
    double getRadius() const;

    const Eigen::Vector3d &getTarget() const;

private:

    // Just for statistics, doesn't affect functionality, so it's mutable.
    mutable size_t samples_yielded;
    mutable size_t samples_tried;

public:
    size_t getSamplesYielded() const;

    size_t getSamplesTried() const;

    DroneEndEffectorNearTarget(const ompl::base::SpaceInformationPtr &si, double radius, const Eigen::Vector3d &target);

    void sampleGoal(ompl::base::State *state) const override;

    [[nodiscard]] unsigned int maxSampleCount() const override;

    double distanceGoal(const ompl::base::State *state) const override;

};

std::shared_ptr<ompl::base::SpaceInformation>
initSpaceInformation(const planning_scene::PlanningScenePtr &scene,
                     const moveit::core::RobotModelPtr &robot,
                     std::shared_ptr<DroneStateSpace> &state_space);

#endif //NEW_PLANNERS_OMPL_CUSTOM_H
