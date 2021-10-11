//
// Created by werner on 06-09-21.
//

#include "BulletContinuousMotionValidator.h"
#include "procedural_tree_generation.h"
#include "LeavesCollisionChecker.h"
#include <robowflex_library/trajectory.h>
#include <json/value.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
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
#include "UnionGoalSampleableRegion.h"

bool StateValidityChecker::isValid(const ompl::base::State *state) const {

    auto space = si_->getStateSpace()->as<DroneStateSpace>();

    assert(space->getRobotModel());
    moveit::core::RobotState robot_state(space->getRobotModel());
    assert(state->as<DroneStateSpace::StateType>());
    space->copyToRobotState(robot_state, state);

    // We rely on the sampler producing states that are  valid in all other aspects, so here we just check collision.
    return !scene_->checkCollision(robot_state).collision;

}

double StateValidityChecker::clearance(const ompl::base::State *state) const {
    auto space = si_->getStateSpace()->as<DroneStateSpace>();

    moveit::core::RobotState robot_state(space->getRobotModel());
    space->copyToRobotState(robot_state, state);

    // We rely on the sampler producing states that are valid in all other aspects, so here we just check collision.
    return scene_->distanceToCollision(robot_state);
}

DroneStateSampler::DroneStateSampler(const ompl::base::StateSpace *space)
        : StateSampler(space) {}

void DroneStateSampler::sampleUniform(ompl::base::State *state) {
    moveit::core::RobotState st(space_->as<DroneStateSpace>()->getRobotModel());
    DroneStateConstraintSampler::randomizeUprightWithBase(st);
    space_->as<DroneStateSpace>()->copyToOMPLState(state, st);
}

void DroneStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) {
    ROS_ERROR("Not implemented DroneStateSampler::sampleUniformNear");
}

void DroneStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
    ROS_ERROR("Not implemented DroneStateSampler::sampleGaussian");
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
    auto *state_space = si_->getStateSpace()->as<DroneStateSpace>();

    moveit::core::RobotState st(state_space->getRobotModel());

    const size_t ATTEMPTS_BEFORE_GIVE_UP = 100;

    size_t attempts_this_time = 0;

    do {
        DroneStateConstraintSampler::randomizeUprightWithBase(st);
        DroneStateConstraintSampler::moveEndEffectorToGoal(st, radius, target);
        state_space->as<DroneStateSpace>()->copyToOMPLState(state, st);
        samples_tried += 1;

        if (attempts_this_time++ > ATTEMPTS_BEFORE_GIVE_UP) {
            OMPL_WARN("Goal sampling failed after %d attempts. Giving up.", ATTEMPTS_BEFORE_GIVE_UP);
            break;
        }

    } while (!si_->isValid(state));

    samples_yielded += 1;
}

double DroneEndEffectorNearTarget::distanceGoal(const ompl::base::State *state) const {
    auto *state_space = si_->getStateSpace()->as<DroneStateSpace>();
    moveit::core::RobotState st(state_space->getRobotModel());

    Eigen::Vector3d ee_pos = st.getGlobalLinkTransform("end_effector").translation();

    Eigen::Vector3d delta = target - ee_pos;

    return delta.norm();
}

unsigned int DroneEndEffectorNearTarget::maxSampleCount() const {
    return INT_MAX;
}

size_t DroneEndEffectorNearTarget::getSamplesYielded() const {
    return samples_yielded;
}

size_t DroneEndEffectorNearTarget::getSamplesTried() const {
    return samples_tried;
}

std::shared_ptr<ompl::base::SpaceInformation>
initSpaceInformation(const robowflex::SceneConstPtr &scene, const robowflex::RobotConstPtr &robot,
                     std::shared_ptr<DroneStateSpace> &state_space) {
    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);
    si->setStateValidityChecker(std::make_shared<StateValidityChecker>(si.get(), scene));
    si->setMotionValidator(std::make_shared<BulletContinuousMotionValidator>(si.get(), robot, scene));
    si->setup();

    return si;
}

robowflex::Trajectory
convertTrajectory(ompl::geometric::PathGeometric &path, const std::shared_ptr<const robowflex::Robot> &ptr) {
    // Initialize an empty trajectory.
    robowflex::Trajectory trajectory(ptr, "whole_body");

    moveit::core::RobotState st(ptr->getModelConst());

    auto state_space = path.getSpaceInformation()->getStateSpace()->as<DroneStateSpace>();

    for (auto state: path.getStates()) {
        state_space->copyToRobotState(st, state);
        trajectory.addSuffixWaypoint(st);
    }

    return trajectory;
}