// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/3/24
//

#include <random_numbers/random_numbers.h>
#include <CGAL/Polyhedron_3.h>
#include "../visualization/visualization_function_macros.h"

#include "../planning/RobotModel.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/state_tools.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../planning/swept_volume_ccd.h"

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

	auto robot_viz = vizualisation::vizualize_robot_state(viewer, robot, fk1);

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
		vizualisation::update_robot_state(robot, fk, robot_viz);

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