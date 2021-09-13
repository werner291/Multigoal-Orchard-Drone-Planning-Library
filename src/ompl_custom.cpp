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

    collision_detection::CollisionRequest req;

    req.contacts = true;

    auto res = scene_->checkCollision(robot_state, req);

    if (res.collision) {
        for ( const auto &myPair : res.contacts ) {

            collision_detection::AllowedCollision::Type tp;
            bool allowed = scene_->getACMConst().getAllowedCollision(myPair.first.first, myPair.first.second, tp);

            std::cout << myPair.first.first << " - " << myPair.first.second << " Allowed: " << allowed << " type: " << tp << std::endl;

        }
    }

    // We rely on the sampler producing states that are  valid in all other aspects, so here we just check collision.

    return ! res.collision;

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
    auto *state_space = si_->getStateSpace()->as<CustomModelBasedStateSpace>();

    moveit::core::RobotState st(state_space->getRobotModel());

    const size_t ATTEMPTS_BEFORE_GIVE_UP = 1000;

    size_t attempts_this_time = 0;

    do {
        DroneStateConstraintSampler::randomizeUprightWithBase(st);
        DroneStateConstraintSampler::moveEndEffectorToGoal(st, radius, target);
        state_space->as<CustomModelBasedStateSpace>()->copyToOMPLState(state, st);
        samples_tried += 1;

        if (attempts_this_time++ > ATTEMPTS_BEFORE_GIVE_UP) {
            OMPL_WARN("Goal sampling failed after %d attempts. Giving up.", ATTEMPTS_BEFORE_GIVE_UP);
            break;
        }

    } while (!si_->isValid(state));

    samples_yielded += 1;
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

size_t DroneEndEffectorNearTarget::getSamplesYielded() const {
    return samples_yielded;
}

size_t DroneEndEffectorNearTarget::getSamplesTried() const {
    return samples_tried;
}

