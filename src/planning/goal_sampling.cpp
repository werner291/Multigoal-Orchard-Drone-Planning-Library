// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 17-1-24.
//

#include "goal_sampling.h"
#include "collision_detection.h"

namespace mgodpl
{

    RobotState genUprightState(random_numbers::RandomNumberGenerator rng)
    {
        RobotState state = {
            .base_tf = math::Transformd{
                .translation = math::Vec3d(0.0, 0.0, 0.0),
                .orientation = math::Quaterniond::fromAxisAngle(math::Vec3d::UnitZ(), rng.uniformReal(-M_PI, M_PI))
            },
            .joint_values = {rng.uniformReal(-M_PI / 2.0, M_PI / 2.0)}
        };

        return state;
    }

    RobotState genGoalStateUniform(
        random_numbers::RandomNumberGenerator rng,
        const math::Vec3d& target,
        const robot_model::RobotModel& robot,
        const robot_model::RobotModel::LinkId& flying_base,
        const robot_model::RobotModel::LinkId& end_effector)
    {
        RobotState state = genUprightState(rng);

        const auto& fk = robot_model::forwardKinematics(
            robot,
            state.joint_values,
            flying_base,
            state.base_tf
        );

        math::Vec3d target_delta = target - fk.forLink(end_effector).translation;

        state.base_tf.translation = state.base_tf.translation + target_delta;

        return state;
    }

    std::optional<RobotState> findGoalStateByUniformSampling(
        const math::Vec3d& target,
        const robot_model::RobotModel& robot,
        const robot_model::RobotModel::LinkId& flying_base,
        const robot_model::RobotModel::LinkId& end_effector,
        const fcl::CollisionObjectd& tree_trunk_object,
        random_numbers::RandomNumberGenerator rng,
        size_t max_attempts)
    {
        for (size_t i = 0; i < max_attempts; ++i)
        {
            RobotState state = genGoalStateUniform(rng, target, robot, flying_base, end_effector);

            if (!check_robot_collision(robot, tree_trunk_object, state))
            {
                return state;
            }
        }

        return std::nullopt;
    }
}