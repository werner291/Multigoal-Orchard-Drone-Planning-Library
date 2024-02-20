// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/20/24.
//

#include <random_numbers/random_numbers.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include "../visualization/visualization_function_macros.h"

#include "../planning/RobotModel.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/state_tools.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../planning/swept_volume_ccd.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(swept_volume) {

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

	visualization.updateTriangles(swept_volume_triangles(robot, state1, state2, 5));

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
			visualization.updateTriangles(swept_volume_triangles(robot, state1, state2, 5));

			max_repeats--;

			if (max_repeats == 0) {
				viewer.stop();
			}

		}

		auto st = interpolate(state1, state2, t);

		auto fk = forwardKinematics(robot, st.joint_values, 0, st.base_tf);

		// update.
		vizualisation::update_robot_state(robot, fk, robot_viz);
	});

	viewer.addActor(visualization.getActor());

	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

	viewer.start();

}