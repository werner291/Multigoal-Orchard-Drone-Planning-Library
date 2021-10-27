#include <robowflex_library/geometry.h>
#include <robowflex_ompl/ompl_interface.h>
#include "BulletContinuousMotionValidator.h"

bool BulletContinuousMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
    auto dummy_pair = std::make_pair((ompl::base::State *) nullptr, 0.0);
    return checkMotion(s1, s2, dummy_pair);
}

double BulletContinuousMotionValidator::estimateMaximumRotation(const moveit::core::RobotStatePtr &st1,
                                                                const moveit::core::RobotStatePtr &st2) {// Upper bound on the rotation of any part of the robot.
    assert(st1->getRobotModel() == st2->getRobotModel());

    double max_angle = 0.0;
    for (const auto &jm: st1->getRobotModel()->getJointModels()) {
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
                max_angle += std::abs(variables1[2] - variables2[2]); //FIXME this looks wrong, what about wrapping?
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
                max_angle += std::abs(variables1[0] - variables2[0]); // FIXME Wrong, but shouldn't cause asymmetry.
            }
                break;
        }
    }
    return max_angle;
}

bool BulletContinuousMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                                  std::pair<ompl::base::State *, double> &lastValid) const {

    // Prepare the input and output structs.
    collision_detection::CollisionResult res;
    collision_detection::CollisionRequest req;

    // Convert the OMPL states to RobotStates
    auto st1 = rb_robot_->allocState();
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*st1, s1);

    auto st2 = rb_robot_->allocState();
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*st2, s2);

    // Compute the largest-possible rotation based on the angle changes in the joints.
    double max_angle = estimateMaximumRotation(st1, st2);

    // Descide on the number of sections to break down into,
    // see issue https://github.com/ros-planning/moveit/issues/2889
    // 8 sections for every 180 degrees of rotation
    // TODO: Review it with Frank maybe?
    size_t num_sections = (size_t) std::ceil(max_angle * 8.0 / M_PI) + 1;

    // Keep track of the begin state of the next section to check.
    auto last = st1;

    // TODO: Possible optimization : collisions are often normally-distributed around the middle of the motion.
    // TODO: the usage of T as the end of the section is weird, here. Check logic.
    for (size_t i = 0; i < num_sections; i++) {


        std::shared_ptr<moveit::core::RobotState> interp;
        if (i + 1 < num_sections) {
            // Allocate a state and interpolate using the coefficient
            interp = rb_robot_->allocState();

            // Interpolation coefficient based on loop index, +1 since it's the end of the motion.
            double t = (double) (i + 1) / (double) num_sections;
            st1->interpolate(*st2, t, *interp);
            interp->update(true);
        } else {
            // Just use the end state if we're in the last section
            interp = st2;
        }

        // Perform the linear CCD check. Note that this simply checks against the convex hull of the shapes of the robot
        // before and after the transformation, which doesn't really work well with rotations and swinging motions,
        // we break up the motion based on how much rotation is involved.
        rb_scene_->getSceneConst()->getCollisionEnv()->checkRobotCollision(req, res, *last, *interp,
                                                                           rb_scene_->getACMConst());

        // Fail the collision check if the section causes a collision.
        if (res.collision) {

            if (lastValid.first != nullptr) {
                si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToOMPLState(lastValid.first,
                                                                                                  *last);
                lastValid.second = (double) i / (double) num_sections;
            }

            return false;
        }

        // End of the current section is beginning of the next.
        last = interp;

    }

    // No collisions found in any of the sections, return valid.
    return true;
}
