// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include "../visualization/visualization_function_macros.h"

#include "../planning/RobotModel.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/state_tools.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../planning/swept_volume_ccd.h"
#include "../planning/RandomNumberGenerator.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(shortcutting) {
	// Load a robot model.
	auto robot = experiments::createProceduralRobotModel();

	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

	auto state1 = fromEndEffectorAndVector(robot,
	                                       {
		                                       rng.uniformReal(-2.0, 0.0), rng.uniformReal(-2.0, 2.0),
		                                       rng.uniformReal(-2.0, 2.0)
	                                       },
	                                       {
		                                       rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0),
		                                       rng.uniformReal(-2.0, 2.0)
	                                       });
	auto state2 = fromEndEffectorAndVector(robot,
	                                       {
		                                       rng.uniformReal(0.0, 2.0), rng.uniformReal(-2.0, 2.0),
		                                       rng.uniformReal(-2.0, 2.0)
	                                       },
	                                       {
		                                       rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0),
		                                       rng.uniformReal(-2.0, 2.0)
	                                       });

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
			                                  {
				                                  rng.uniformReal(-2.0, 0.0), rng.uniformReal(-2.0, 2.0),
				                                  rng.uniformReal(-2.0, 2.0)
			                                  },
			                                  {
				                                  rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0),
				                                  rng.uniformReal(-2.0, 2.0)
			                                  });
			state2 = fromEndEffectorAndVector(robot,
			                                  {
				                                  rng.uniformReal(-0.0, 2.0), rng.uniformReal(-2.0, 2.0),
				                                  rng.uniformReal(-2.0, 2.0)
			                                  },
			                                  {
				                                  rng.uniformReal(-2.0, 2.0), rng.uniformReal(-2.0, 2.0),
				                                  rng.uniformReal(-2.0, 2.0)
			                                  });

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

	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

	viewer.start();
}
