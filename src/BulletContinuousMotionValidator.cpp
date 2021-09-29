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

    double max_angle = estimateMaximumRotation(st1, st2);

    // 8 sections for every radian of rotation, upto a max of 16.
    // TODO: Review it with Frank maybe?
    size_t num_sections = (size_t) std::ceil(max_angle * 8.0 / M_PI) + 1;
    size_t num_points = num_sections + 1;

    auto last = st1;

    // TODO: Optimization: collisions are often normally-distributed around the middle of the motion.
    for (size_t i = 0; i < num_sections; i++) {

        double t = (double) i / (double) num_points;
        std::shared_ptr<moveit::core::RobotState> interp;

        if (i+1 == num_sections) {
            interp = rb_robot_->allocState();
            st1->interpolate(*st2, t, *interp);
            interp->update(true);
        } else {
            interp = st2;
        }

        rb_scene_->getSceneConst()->getCollisionEnv()->checkRobotCollision(req, res, *last, *interp, rb_scene_->getACMConst());

        if (res.collision) { return false; }

        last = interp;

    }

    return true;
}

double BulletContinuousMotionValidator::estimateMaximumRotation(const moveit::core::RobotStatePtr &st1,
                                                                const moveit::core::RobotStatePtr &st2) const {// Upper bound on the rotation of any part of the robot.
    double max_angle = 0.0;
    for (const auto &jm: rb_robot_->getModelConst()->getJointModels()) {
        switch (jm->getType()) {
            case moveit::core::JointModel::UNKNOWN:
                ROS_ERROR("Cannot compute maximum rotation with unknown joint type.");
                break;
            case moveit::core::JointModel::PRISMATIC:
            case moveit::core::JointModel::FIXED:
                // These do not affect rotation
                break;

            case moveit::core::JointModel::PLANAR: {
                const double *variables1 = st1->getJointPositions(jm);
                const double *variables2 = st2->getJointPositions(jm);
                max_angle += std::abs(variables1[2] - variables2[2]);
            }
                break;

            case moveit::core::JointModel::FLOATING: {

                const double *variables1 = st1->getJointPositions(jm);
                Eigen::Quaterniond rot1(variables1[6], variables1[3], variables1[4], variables1[5]);
                const double *variables2 = st2->getJointPositions(jm);
                Eigen::Quaterniond rot2(variables2[6], variables2[3], variables2[4], variables2[5]);

                max_angle += rot1.angularDistance(rot2);
            }
            break;

            case moveit::core::JointModel::REVOLUTE: {
                const double *variables1 = st1->getJointPositions(jm);
                const double *variables2 = st2->getJointPositions(jm);
                max_angle += std::abs(variables1[0] - variables2[0]);
            }
                break;
        }
    }
    return max_angle;
}

bool BulletContinuousMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                                  std::pair<ompl::base::State *, double> &lastValid) const {
    ROS_ERROR("Not implemented BulletContinuousMotionValidator::checkMotion");

    return false;
}
