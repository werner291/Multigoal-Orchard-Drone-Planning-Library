//
// Created by werner on 8/25/21.
//

#ifndef NEW_PLANNERS_BULLETCONTINUOUSMOTIONVALIDATOR_H
#define NEW_PLANNERS_BULLETCONTINUOUSMOTIONVALIDATOR_H

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

class BulletContinuousMotionValidator : public ompl::base::MotionValidator {

    moveit::core::RobotModelConstPtr rb_robot_;
    planning_scene::PlanningSceneConstPtr rb_scene_;

public:
    BulletContinuousMotionValidator(ompl::base::SpaceInformation *si,
                                    moveit::core::RobotModelConstPtr rbRobot,
                                    planning_scene::PlanningSceneConstPtr rbScene)
            : MotionValidator(si), rb_robot_(std::move(rbRobot)), rb_scene_(std::move(rbScene)) {

    }

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override;

/**
 *
 * Estimate the largest angle that any part of the robot can rotate by, in radians. This is because
 * bullet-based CCD simply takes the convex hull of the robot's link shapes in the first and second
 * state, which doesn't work when rotations are involved. Knowing the amount of rotation in a motion
 * helps with that.
 *
 * See issue: https://github.com/ros-planning/moveit/issues/2889
 *
 * @param st1 Starting state
 * @param st2 Ending state
 * @return Maximum rotation in radians
 */
    static double
    estimateMaximumRotation(const moveit::core::RobotState &st1, const moveit::core::RobotState &st2);
};

#endif //NEW_PLANNERS_BULLETCONTINUOUSMOTIONVALIDATOR_H
