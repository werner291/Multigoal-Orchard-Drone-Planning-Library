//
// Created by werner on 8/25/21.
//

#include "make_robot.h"
#include "ClearanceDecreaseMinimzationObjective.h"
#include "build_planning_scene.h"
#include "build_request.h"
#include "EndEffectorConstraintSampler.h"
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <random_numbers/random_numbers.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/util.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/geometry.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_library/benchmarking.h>
#include <robowflex_library/builder.h>
#include "BulletContinuousMotionValidator.h"

bool BulletContinuousMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {

    collision_detection::CollisionResult res;
    collision_detection::CollisionRequest req;

    auto st1 = rb_robot_->allocState();
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*st1, s1);

    auto st2 = rb_robot_->allocState();
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*st2, s2);

    rb_scene_->getScene()->getCollisionEnv()->checkRobotCollision(req, res, *st1, *st2, rb_scene_->getACM());

    return ! res.collision;
}

bool BulletContinuousMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                                  std::pair<ompl::base::State *, double> &lastValid) const {
    ROS_ERROR("Not implemented.");

    return false;
}
