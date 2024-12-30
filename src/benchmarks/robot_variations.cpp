// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/3/24
//

#include <CGAL/Polyhedron_3.h>
#include <vtkRenderer.h>
#include "../visualization/visualization_function_macros.h"

#include "../planning/RobotModel.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/state_tools.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../planning/swept_volume_ccd.h"
#include "../planning/RandomNumberGenerator.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(sensor_cone) {

	VtkTriangleSetVisualization visualization(1.0, 0.5, 0.5, 0.3);

	// Load a robot model.
	auto robot = experiments::createProceduralRobotModel();

	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

	auto state1 = fromEndEffectorAndVector(robot,
										   {rng.uniformReal(-2.0, 0.0), rng.uniformReal(-2.0, 2.0),
											rng.uniformReal(-2.0, 2.0)},
										   {rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0),
											rng.uniformReal(-2.0, 2.0)});
	auto state2 = fromEndEffectorAndVector(robot,
										   {rng.uniformReal(0.0, 2.0), rng.uniformReal(-2.0, 2.0),
											rng.uniformReal(-2.0, 2.0)},
										   {rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0),
											rng.uniformReal(-2.0, 2.0)});

	// compute FK.
	auto fk1 = forwardKinematics(robot, state1.joint_values, 0, state1.base_tf);

	auto robot_viz = visualization::vizualize_robot_state(viewer, robot, fk1);

	size_t max_repeats = 20;

	viewer.addTimerCallback([&]() {
		static double t = 0.0;
		t += 0.02;

		if (t > 1.0) {
			t = 0.0;

			state1 = fromEndEffectorAndVector(robot,
											  {rng.uniformReal(-2.0, 0.0), rng.uniformReal(-2.0, 2.0),
											   rng.uniformReal(-2.0, 2.0)},
											  {rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0),
											   rng.uniformReal(-2.0, 2.0)});
			state2 = fromEndEffectorAndVector(robot,
											  {rng.uniformReal(-0.0, 2.0), rng.uniformReal(-2.0, 2.0),
											   rng.uniformReal(-2.0, 2.0)},
											  {rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0),
											   rng.uniformReal(-2.0, 2.0)});


			max_repeats--;

			if (max_repeats == 0) {
				viewer.stop();
			}

		}

		auto st = interpolate(state1, state2, t);

		auto fk = forwardKinematics(robot, st.joint_values, 0, st.base_tf);

		// update.
		visualization::update_robot_state(robot, fk, robot_viz);

		// Get the end-effector position and orientation.
		auto ee_tf = fk.forLink(robot.findLinkByName("end_effector"));

		math::Vec3d ee_center = ee_tf.translation;

		// Create a cone.
		std::vector<std::array<math::Vec3d, 3>> cone_triangles;

		// The cone's base is a circle with 10 points.
		size_t num_points = 10;

		for (size_t i = 0; i < num_points; ++i) {
			double angle = 2.0 * M_PI * i / num_points;
			double next_angle = 2.0 * M_PI * (i + 1) / num_points;

			math::Vec3d a = ee_center;
			math::Vec3d b = ee_tf.apply(math::Vec3d{cos(angle), sin(angle), 0.0});
			math::Vec3d c = ee_tf.apply(math::Vec3d{cos(next_angle), sin(next_angle), 0.0});

			cone_triangles.push_back({a, b, c});
		}

		// Add the cone to the visualization.
		visualization.updateTriangles(cone_triangles);
	});

	viewer.addActor(visualization.getActor());

	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

	viewer.start();

}

// Add this function to generate random robot arm parameters
mgodpl::experiments::RobotArmParameters generateRandomRobotArmParameters(random_numbers::RandomNumberGenerator &rng) {
    mgodpl::experiments::RobotArmParameters params;
    params.total_arm_length = rng.uniformReal(0.5, 1.5); // Random length between 0.5 and 1.5
    params.add_spherical_wrist = rng.uniform01() > 0.5; // Randomly decide to add a spherical wrist

    // Randomly decide the joint types
    size_t num_joints = rng.uniformInteger(1, 6); // Random number of joints between 2 and 6
    for (size_t i = 0; i < num_joints; ++i) {
        params.joint_types.push_back(rng.uniform01() > 0.5 ? mgodpl::experiments::HORIZONTAL : mgodpl::experiments::VERTICAL);
    }

    return params;
}

mgodpl::RobotState randomState(const mgodpl::robot_model::RobotModel& robot, random_numbers::RandomNumberGenerator& rng) {
	RobotState state {
		.base_tf = math::Transformd::identity(),
		.joint_values = {}
	};

	// For every joint, append a random number between -PI/2 and PI/2
	for (size_t i = 0; i < robot.count_joint_variables(); ++i) {
		state.joint_values.push_back(rng.uniformReal(-M_PI / 2.0, M_PI / 2.0));
	}

	return state;
}

// Modify the REGISTER_VISUALIZATION function
REGISTER_VISUALIZATION(procedural_robot_models) {

    VtkTriangleSetVisualization visualization(1.0, 0.5, 0.5, 0.3);

    // Create a random number generator.
    random_numbers::RandomNumberGenerator rng;

	mgodpl::visualization::RobotActors robot_actors;

	robot_model::RobotModel robot = mgodpl::experiments::createProceduralRobotModel(generateRandomRobotArmParameters(rng));

	{
		RobotState state = randomState(robot, rng);
		robot_actors = visualization::vizualize_robot_state(viewer, robot, forwardKinematics(robot, state.joint_values, 0, state.base_tf));
	}

	int max_repeats = 10;

    viewer.addTimerCallback([&]() {
        static double t = 0.0;
        t += 0.02;

        if (t > 5.0) {

			max_repeats--;
			if (max_repeats == 0) {
				viewer.stop();
			}

            t = 0.0;

            // Create a new robot model with the random parameters
			robot = mgodpl::experiments::createProceduralRobotModel(generateRandomRobotArmParameters(rng));

            // Visualize the robot model
			for (const auto& actor : robot_actors.actors) {
				viewer.viewerRenderer->RemoveActor(actor);
			}

			RobotState state = randomState(robot, rng);
			robot_actors = visualization::vizualize_robot_state(viewer, robot, forwardKinematics(robot, state.joint_values, 0, state.base_tf));

        } else {

			RobotState state = randomState(robot, rng);

			// Use a sin/cos function to move the robot arm
			for (int joint_id = 0; joint_id < robot.count_joint_variables(); ++joint_id) {
				state.joint_values[joint_id] = 0.5 * M_PI * sin(t * 2.0 * M_PI + joint_id * M_PI / 4.0) / (double) robot.count_joint_variables();
			}

			visualization::update_robot_state(
				robot,
				forwardKinematics(robot, state.joint_values, 0, state.base_tf),
				robot_actors
					);
		}
    });

    viewer.addActor(visualization.getActor());

    viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

    viewer.start();
}
