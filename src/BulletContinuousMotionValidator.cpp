#include <robowflex_library/geometry.h>
#include <robowflex_ompl/ompl_interface.h>
#include "BulletContinuousMotionValidator.h"

bool BulletContinuousMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {

    collision_detection::CollisionResult res;
    collision_detection::CollisionRequest req;

    auto st1 = rb_robot_->allocState();
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*st1, s1);

    auto st2 = rb_robot_->allocState();
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*st2, s2);

    rb_scene_->getSceneConst()->getCollisionEnv()->checkRobotCollision(req, res, *st1, *st2, rb_scene_->getACMConst());

    return ! res.collision;
}

bool BulletContinuousMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                                  std::pair<ompl::base::State *, double> &lastValid) const {
    ROS_ERROR("Not implemented BulletContinuousMotionValidator::checkMotion");

    return false;
}
