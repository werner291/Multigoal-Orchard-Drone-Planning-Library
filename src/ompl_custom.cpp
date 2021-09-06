//
// Created by werner on 06-09-21.
//

#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <fcl/fcl.h>
#include "EndEffectorConstraintSampler.h"
#include "InverseClearanceIntegralObjective.h"
#include "init_planner.h"
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/util.h>
#include <robowflex_library/builder.h>
#include "ompl_custom.h"

bool StateValidityChecker::isValid(const ompl::base::State *state) const {

    auto space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();

    moveit::core::RobotState robot_state(space->getRobotModel());
    space->copyToRobotState(robot_state, state);

    // We rely on the sampler producing states that are valid in all other aspects, so here we just check collision.
    return !scene_->checkCollision(robot_state).collision;
}

double StateValidityChecker::clearance(const ompl::base::State *state) const {
    auto space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();

    moveit::core::RobotState robot_state(space->getRobotModel());
    space->copyToRobotState(robot_state, state);

    // We rely on the sampler producing states that are valid in all other aspects, so here we just check collision.
    return scene_->distanceToCollision(robot_state);
}

DroneStateSampler::DroneStateSampler(const ompl::base::StateSpace *space)
        : StateSampler(space) {}

void DroneStateSampler::sampleUniform(ompl::base::State *state) {
    moveit::core::RobotState st(space_->as<CustomModelBasedStateSpace>()->getRobotModel());
    DroneStateConstraintSampler::randomizeUprightWithBase(st);
    space_->as<CustomModelBasedStateSpace>()->copyToOMPLState(state, st);
}

void DroneStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) {
    ROS_ERROR("Not implemented.");
}

void DroneStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
    ROS_ERROR("Not implemented.");
}

InverseClearanceIntegralObjectiveOMPL::InverseClearanceIntegralObjectiveOMPL(const ompl::base::SpaceInformationPtr &si,
                                                                             bool enableMotionCostInterpolation)
        : StateCostIntegralObjective(si, enableMotionCostInterpolation) {}

ompl::base::Cost InverseClearanceIntegralObjectiveOMPL::stateCost(const ompl::base::State *s) const {
    return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
}

DroneEndEffectorNearTarget::DroneEndEffectorNearTarget(const ompl::base::SpaceInformationPtr &si, double radius,
                                                       const Eigen::Vector3d &target)
        : GoalSampleableRegion(si), radius(radius), target(target) {}

void DroneEndEffectorNearTarget::sampleGoal(ompl::base::State *state) const {
    auto *state_space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();

    moveit::core::RobotState st(state_space->getRobotModel());
    DroneStateConstraintSampler::randomizeUprightWithBase(st);
    DroneStateConstraintSampler::moveEndEffectorToGoal(st, radius, target);
    state_space->as<CustomModelBasedStateSpace>()->copyToOMPLState(state, st);
}

double DroneEndEffectorNearTarget::distanceGoal(const ompl::base::State *state) const {
    auto *state_space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();
    moveit::core::RobotState st(state_space->getRobotModel());

    Eigen::Vector3d ee_pos = st.getGlobalLinkTransform("end_effector").translation();

    Eigen::Vector3d delta = target - ee_pos;

    return delta.norm();
}

unsigned int DroneEndEffectorNearTarget::maxSampleCount() const {
    return INT_MAX;
}
