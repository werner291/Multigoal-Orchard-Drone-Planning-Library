// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#include "../planning/RobotModel.h"

#include "load_mesh_ros.h"
#include "procedural_robot_models.h"

namespace mgodpl
{
    using namespace experiments;
    using namespace robot_model;

	static const double ARM_LENGTH = 0.75;

	robot_model::RobotModel mgodpl::experiments::createProceduralRobotModel()
    {
        double stick_length = ARM_LENGTH;

        RobotModel model;

        RobotModel::LinkId flying_base = model.insertLink(
            {
                .name = "flying_base",
                .joints = {},
                .collision_geometry = {
                    {
                        .shape = Box{
                            .size = {
                                0.8, 0.8,
                                0.2
                            }
                        },
                        .transform = math::Transformd::identity()
                    }
                },
                .visual_geometry = {
                    {
                        .shape = loadMesh(
                            "test_robots/meshes/drone.dae"),
                        .transform = math::Transformd::identity()
                    }
                }
            });

        RobotModel::LinkId stick = model.insertLink({
            .name = "stick",
            .joints = {},
            .collision_geometry = {
                {
                    .shape = Box{
                        .size = {
                            0.05, stick_length, 0.05
                        }
                    },
                    .transform = math::Transformd::identity()
                }
            },
            .visual_geometry = {}
        });

        RobotModel::LinkId end_effector = model.insertLink({
            .name = "end_effector",
            .joints = {}
        });

        model.insertJoint({
            .name = "stick_base_attachment",
            .attachmentA = {
                .translation = {0, 0.2, 0},
                .orientation = {0, 0, 0, 1}
            },
            .attachmentB = {
                .translation = {
                    0, -stick_length /
                    2.0,
                    0
                },
                .orientation = {0, 0, 0, 1}
            },
            .linkA = flying_base,
            .linkB = stick,
            .type_specific = RobotModel::RevoluteJoint{
                .axis = {1, 0, 0},
                .min_angle = -M_PI,
                .max_angle = M_PI
            }
        });

        model.insertJoint({
            .name = "stick_end_attachment",
            .attachmentA = {
                .translation = {
                    0,
                    stick_length / 2.0,
                    0
                },
                .orientation = {0, 0, 0, 1}
            },
            .attachmentB = {
                .translation = {0, 0, 0},
                .orientation = {0, 0, 0, 1}
            },
            .linkA = stick,
            .linkB = end_effector,
            .type_specific = RobotModel::FixedJoint{}
        });

        return model;
    }
}
