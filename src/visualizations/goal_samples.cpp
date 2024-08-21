// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/14/24.
//

#include "../planning/RandomNumberGenerator.h"
#include "../math/Vec3.h"
#include "../planning/RobotModel.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RobotState.h"
#include "../planning/goal_sampling.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/robot_state.h"
#include "../visualization/visualization_function_macros.h"


using namespace mgodpl;

REGISTER_VISUALIZATION(goal_samples) {

    // Create a random number generator
    random_numbers::RandomNumberGenerator rng(42);

    // Define the target position
    math::Vec3d target(0, 0, 0);
	viewer.addSphere(0.05, target, {1, 0, 0});

	// Create a robot model
    robot_model::RobotModel robot = experiments::createProceduralRobotModel(experiments::RobotArmParameters {
		.total_arm_length = 1.0,
		.joint_types = {experiments::JointType::HORIZONTAL},//, experiments::JointType::VERTICAL},
		.add_spherical_wrist = false
	});

    // Get the link ID of the flying base and the end effector
    robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");
    robot_model::RobotModel::LinkId end_effector = robot.findLinkByName("end_effector");

	static const int N_SAMPLES = 10;

	// Generate and visualize 100 goal states
    for (int i = 0; i < N_SAMPLES; ++i) {
        // Generate a goal state
        RobotState goal_state = genGoalStateUniform(rng, target, robot, flying_base, end_effector);

        // Compute the forward kinematics
        auto fk = forwardKinematics(robot, goal_state.joint_values, 0, goal_state.base_tf);

        // Visualize the goal state
        vizualisation::vizualize_robot_state(viewer, robot, fk, {0.5, 0.5, 0.5});
    }

	viewer.lockCameraUp();

	viewer.setCameraTransform({5, 5, 1}, target);

    // Start the viewer
    viewer.start();
}