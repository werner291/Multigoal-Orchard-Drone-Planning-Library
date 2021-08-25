//
// Created by werner on 8/25/21.
//

#ifndef NEW_PLANNERS_BULLETCONTINUOUSMOTIONVALIDATOR_H
#define NEW_PLANNERS_BULLETCONTINUOUSMOTIONVALIDATOR_H

#include <memory>
#include <ompl/base/State.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>

class BulletContinuousMotionValidator : public ompl::base::MotionValidator {

    std::shared_ptr<robowflex::Scene> rb_scene_;
    std::shared_ptr<robowflex::Robot> rb_robot_;
public:
    BulletContinuousMotionValidator(ompl::base::SpaceInformation *si,
                                    const std::shared_ptr<robowflex::Robot> &rbRobot,
                                    const std::shared_ptr<robowflex::Scene> &rbScene)
            : MotionValidator(si), rb_robot_(rbRobot), rb_scene_(rbScene) {}

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override;

};

#endif //NEW_PLANNERS_BULLETCONTINUOUSMOTIONVALIDATOR_H
