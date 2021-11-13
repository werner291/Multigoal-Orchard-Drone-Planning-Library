
#include "BulletContinuousMotionValidator.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <fcl/fcl.h>
#include "DroneStateConstraintSampler.h"
#include "InverseClearanceIntegralObjective.h"
#include "ompl_custom.h"
#include "UnionGoalSampleableRegion.h"

bool StateValidityChecker::isValid(const ompl::base::State *state) const {

    auto space = si_->getStateSpace()->as<DroneStateSpace>();

    assert(space->getRobotModel());
    moveit::core::RobotState robot_state(space->getRobotModel());
    assert(state->as<DroneStateSpace::StateType>());
    space->copyToRobotState(robot_state, state);

    // We rely on the sampler producing states that are  valid in all other aspects, so here we just check collision.
    collision_detection::CollisionResult result;
    collision_detection::CollisionRequest request;
    scene_->checkCollision(request, result, robot_state);

    return !result.collision;

}

double StateValidityChecker::clearance(const ompl::base::State *state) const {
    auto space = si_->getStateSpace()->as<DroneStateSpace>();

    moveit::core::RobotState robot_state(space->getRobotModel());
    space->copyToRobotState(robot_state, state);

    // We rely on the sampler producing states that are valid in all other aspects, so here we just check collision.
    return scene_->distanceToCollision(robot_state);
}

InverseClearanceIntegralObjectiveOMPL::InverseClearanceIntegralObjectiveOMPL(const ompl::base::SpaceInformationPtr &si,
                                                                             bool enableMotionCostInterpolation)
        : StateCostIntegralObjective(si, enableMotionCostInterpolation) {}

ompl::base::Cost InverseClearanceIntegralObjectiveOMPL::stateCost(const ompl::base::State *s) const {
    return ompl::base::Cost(1.0 / si_->getStateValidityChecker()->clearance(s));
}

DroneEndEffectorNearTarget::DroneEndEffectorNearTarget(const ompl::base::SpaceInformationPtr &si, double radius,
                                                       const Eigen::Vector3d &target)
        : GoalSampleableRegion(si), radius(radius), target(target) {
    this->setThreshold(1e-5);
}

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
            return;
        }

    } while (!si_->isValid(state));

    assert(this->isSatisfied(state));

    samples_yielded += 1;
}

double DroneEndEffectorNearTarget::distanceGoal(const ompl::base::State *state) const {
    auto *state_space = si_->getStateSpace()->as<DroneStateSpace>();
    moveit::core::RobotState st(state_space->getRobotModel());
    state_space->copyToRobotState(st, state);

    Eigen::Vector3d ee_pos = st.getGlobalLinkTransform("end_effector").translation();

    Eigen::Vector3d delta = target - ee_pos;

    return std::max(delta.norm() - radius, 0.0);
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

double DroneEndEffectorNearTarget::getRadius() const {
    return radius;
}

const Eigen::Vector3d &DroneEndEffectorNearTarget::getTarget() const {
    return target;
}

std::shared_ptr<ompl::base::SpaceInformation>
initSpaceInformation(const planning_scene::PlanningScenePtr &scene,
                     const moveit::core::RobotModelPtr &robot,
                     std::shared_ptr<DroneStateSpace> &state_space) {

    auto si = std::make_shared<ompl::base::SpaceInformation>(state_space);
    si->setStateValidityChecker(std::make_shared<StateValidityChecker>(si.get(), scene));
    si->setMotionValidator(std::make_shared<BulletContinuousMotionValidator>(si.get(), robot, scene));
    si->setup();

    return si;
}

