// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 8/28/24.
//

#include "../experiment_utils/default_colors.h"
#include "../experiment_utils/procedural_fruit_placement.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RobotPath.h"
#include "../planning/RobotPathFn.h"
#include "../planning/state_tools.h"
#include "../visualization/robot_state.h"
#include "../visualization/visualization_function_macros.h"

using namespace mgodpl;
using namespace mgodpl::vizualisation;

RobotPathFn whole_body_orbit_path(const robot_model::RobotModel &robot_model, double distance = 0.5) {
	return [robot_model, distance](double t) {
		const double angle = t * 2.0 * M_PI;
		const math::Vec3d arm_vector = math::Vec3d{cos(angle), sin(angle), 0.0};
		const math::Vec3d offset_vector = arm_vector * distance;
		return fromEndEffectorAndVector(robot_model, offset_vector, arm_vector);
	};
}

RobotPathFn end_effector_orbit_path(const robot_model::RobotModel &robot_model, double distance = 0.5) {
	return [robot_model, distance](double t) {
		const double angle = t * 2.0 * M_PI;
		const math::Vec3d arm_vector = math::Vec3d{1.0, 0.0, 0.0};
		const math::Vec3d offset_vector = math::Vec3d{cos(angle), sin(angle), 0.0} * distance;
		return fromEndEffectorAndVector(robot_model, offset_vector, arm_vector);
	};
}

/**
 * Visualizes a set of different motions whereby the robot, starting from a configuration near a fruit,
 * will make motions to scan the fruit from different angles using its end-effector.
 */
REGISTER_VISUALIZATION(scanning_motions_straight_arm) {

	// The radius of the sphere representing the fruit.
	const double FRUIT_RADIUS = 0.1;

	// The distance from the surface at which we'd ideally like to scan the fruit.
	const double SCAN_DISTANCE = 0.1;

	double scan_radius = FRUIT_RADIUS + SCAN_DISTANCE;

	const math::Vec3d fruit_position = {0.0, 0.0, 0.0};
	viewer.addSphere(FRUIT_RADIUS, fruit_position, FRUIT_COLOR);

	const robot_model::RobotModel robot_model = experiments::createProceduralRobotModel(
			experiments::RobotArmParameters{.total_arm_length = 1.0, .joint_types = {experiments::HORIZONTAL}});

	const RobotState initial_state = fromEndEffectorAndVector(robot_model, {scan_radius, 0.0, 0.0}, {1.0, 0.0, 0.0});

	viewer.setCameraTransform(fruit_position + math::Vec3d{5.0, 0.0, 5}, fruit_position);

	auto rb = vizualize_robot_state(viewer, robot_model, forwardKinematics(robot_model, initial_state));

	const std::vector paths = {whole_body_orbit_path(robot_model, scan_radius),
							   end_effector_orbit_path(robot_model, scan_radius)};

	double t = 0.0;

	viewer.addTimerCallback([&]() {
		t += 0.01;
		if (t > static_cast<double>(paths.size() + 1)) {
			t = 0.0;
		}
		update_robot_state(robot_model, forwardKinematics(robot_model, paths[t](t - std::floor(t))), rb);
	});

	viewer.start();
}
