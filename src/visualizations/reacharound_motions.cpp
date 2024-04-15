// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/12/24.
//

#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../planning/RandomNumberGenerator.h"
#include "../planning/RobotState.h"
#include "../visualization/robot_state.h"
#include "../experiment_utils/declarative/PointScanExperiment.h"
#include "../experiment_utils/declarative_environment.h"
#include "../experiment_utils/default_colors.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../experiment_utils/tracing.h"
#include "../visualization/VtkPolyLineVisualization.h"

static const double fruit_radius = 0.2;
using namespace mgodpl;
using namespace declarative;
using namespace experiments;
//using namespace visualization;

REGISTER_VISUALIZATION(reach_through_motions) {

	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

	robot_model::RobotModel robot = experiments::createProceduralRobotModel({
																					.total_arm_length = 1.0,
																					.joint_types = {
																							experiments::JointType::HORIZONTAL,
																							experiments::JointType::VERTICAL},
																			});

	RobotState state{
			.base_tf = math::Transformd::fromTranslation({-1.0, 0.0, 0.0}),
			.joint_values = std::vector<double>(robot.count_joint_variables(), 1.0)
	};

	viewer.addSphere(0.2, {0.0, 0.0, 0.0}, {0.5, 0.5, 0.5});

	mgodpl::declarative::PointScanEvalParameters eval_params{
			.tree_params = TreeModelParameters{
					.name = "appletree",
					.leaf_scale = 1.5,
					.fruit_subset = Unchanged{}, //Replace {100},
					.seed = 42
			},
			.sensor_params = SensorScalarParameters{
					.maxViewDistance = INFINITY,
					.minViewDistance = 0.0,
					.fieldOfViewAngle = M_PI / 3.0,
					.maxScanAngle = M_PI / 3.0,
			}
	};

	mgodpl::experiments::TreeModelCache environment_cache;

	const PointScanEnvironment &env = create_environment(eval_params, environment_cache);

	viewer.addMesh(env.tree_model->meshes.trunk_mesh, WOOD_COLOR);
	viewer.addMesh(env.scaled_leaves, LEAF_COLOR);

	const math::Vec3d leaves_center = mesh_aabb(env.scaled_leaves).center();

	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

	auto robot_viz = vizualisation::vizualize_robot_state(viewer,
														  robot,
														  forwardKinematics(robot,
																			state.joint_values,
																			0,
																			state.base_tf));

	// Let's first generate an orbital motion around the tree:

	size_t N_SAMPLES = 100;

	for (size_t i = 0; i < N_SAMPLES; ++i) {
		double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(N_SAMPLES);

		math::Quaterniond base_rot = math::Quaterniond::fromAxisAngle({0, 1, 0}, angle);

	}

	viewer.start();

}

math::Vec3d point_near_fruit(const mgodpl::math::Vec3d &fruit_position,
							 const double fruit_radius,
							 const double max_distance,
							random_numbers::RandomNumberGenerator &rng) {
	math::Vec3d direction{rng.gaussian01(), rng.gaussian01(), rng.gaussian01()};
	direction.normalize();
	double distance = rng.uniformReal(0.0, max_distance) + fruit_radius;

	return fruit_position + direction * distance;

}

/**
 * @brief Generates a random robot state.
 *
 * This function generates a random robot state by creating a random base transformation and joint values.
 * The base transformation is a translation along the x-axis and a rotation around the z-axis.
 * The joint values are random angles between -pi/2 and pi/2.
 *
 * @param robot The robot model.
 * @param rng The random number generator.
 * @return The generated random robot state.
 */
RobotState generateRandomRobotState(robot_model::RobotModel& robot, random_numbers::RandomNumberGenerator& rng) {
    RobotState state {
        .base_tf = math::Transformd {
            .translation = {0.0, 0.0, 0.0},
            .orientation = math::Quaterniond::fromAxisAngle({0, 0, 1}, rng.uniformReal(-M_PI, M_PI))
        },
        .joint_values = {}
    };

    for (size_t j = 0; j < robot.count_joint_variables(); ++j) {
        state.joint_values.push_back(rng.uniformReal(-M_PI / 2.0, M_PI / 2.0));
    }

    return state;
}

/**
 * @brief Gets the position of the end effector.
 *
 * This function calculates the forward kinematics of the robot and returns the translation of the specified link.
 *
 * @param robot The robot model.
 * @param state The robot state.
 * @param linkName The name of the link.
 * @return The position of the end effector.
 */
math::Vec3d getEndEffectorPosition(robot_model::RobotModel& robot, RobotState& state, const std::string& linkName) {
    return forwardKinematics(robot, state.joint_values, 0, state.base_tf).forLink(robot.findLinkByName(linkName)).translation;
}

/**
 * @brief Moves the end effector to the goal position.
 *
 * This function modifies the base transformation of the robot state to move the end effector to the goal position.
 *
 * @param state The robot state.
 * @param target_end_effector_position The target end effector position.
 * @param end_effector_position The current end effector position.
 */
void moveEndEffectorToGoal(RobotState& state, const math::Vec3d& target_end_effector_position, const math::Vec3d& end_effector_position) {
    // Apply a translation to the base tf to move the end-effector to the goal position:
    state.base_tf.translation = state.base_tf.translation + (target_end_effector_position - end_effector_position);
}

/**
 * @brief Moves the end effector to the goal position.
 *
 * This overloaded function first calculates the current end effector position and then calls the original moveEndEffectorToGoal function.
 *
 * @param robot The robot model.
 * @param state The robot state.
 * @param target_end_effector_position The target end effector position.
 */
void moveEndEffectorToGoal(robot_model::RobotModel& robot, RobotState& state, const math::Vec3d& target_end_effector_position) {
    math::Vec3d end_effector_position = getEndEffectorPosition(robot, state, "end_effector");
    moveEndEffectorToGoal(state, target_end_effector_position, end_effector_position);
}

REGISTER_VISUALIZATION(near_goal_samples) {

	// Create a random number generator.
	random_numbers::RandomNumberGenerator rng;

	robot_model::RobotModel robot =
			experiments::createProceduralRobotModel(
					{
							.total_arm_length = 1.0,
							.joint_types = {experiments::JointType::HORIZONTAL,
											experiments::JointType::VERTICAL
							},
							.add_spherical_wrist = false
					});

	math::Vec3d goal_position{0.0, 0.0, 0.0};
	viewer.addSphere(fruit_radius, goal_position, FRUIT_COLOR);

	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

	const size_t N_STATES = 10;

	for (size_t i = 0; i < N_STATES; ++i) {

		math::Vec3d target_end_effector_position = point_near_fruit(goal_position, fruit_radius, 0.5, rng);

		RobotState state = generateRandomRobotState(robot, rng);

		moveEndEffectorToGoal(robot, state, target_end_effector_position);

		auto robot_viz = vizualisation::vizualize_robot_state(viewer,
															  robot,
															  forwardKinematics(robot,
																				state.joint_values,
																				0,
																				state.base_tf));

	}

	viewer.lockCameraUp();
	viewer.start();

}

struct GraphNode {
	RobotState state;
	std::vector<size_t> children;
	double total_distance;
};

std::vector<GraphNode> pseudoRRTStar(robot_model::RobotModel &robot,
									 const math::Vec3d &goal_position,
									 const double fruit_radius,
									 const RobotState &start_state,
									 random_numbers::RandomNumberGenerator &rng) {


	std::vector<GraphNode> states {
			GraphNode {
					.state = start_state,
					.children = {},
					.total_distance = 0.0
			}
	};

	int num_samples = 100;

	for (int i = 0; i < num_samples; ++i) {

		RobotState new_state = generateRandomRobotState(robot, rng);

		// Set the en-effector to a random position near the goal:
		math::Vec3d target_end_effector_position = point_near_fruit(goal_position, fruit_radius, 0.5, rng);
		moveEndEffectorToGoal(robot, new_state, target_end_effector_position);

		// Find the closest existing state:
		auto it = std::min_element(states.begin(), states.end(), [&](const GraphNode& a, const GraphNode& b) {
			double a_base_distance = (a.state.base_tf.translation - new_state.base_tf.translation).squaredNorm() + a.total_distance;
			double b_base_distance = (b.state.base_tf.translation - new_state.base_tf.translation).squaredNorm() + b.total_distance;
			return a_base_distance < b_base_distance;
		});

		double distance_from_closest = (it->state.base_tf.translation - new_state.base_tf.translation).norm();

		// Add it to the states and connect it to the closest state:
		it->children.push_back(states.size());
		states.emplace_back(GraphNode {
				.state = new_state,
				.children = {},
				.total_distance = it->total_distance + distance_from_closest
		});
	}
	return states;
}

std::vector<std::array<size_t, 2>> mkCrossConnections(std::vector<GraphNode> &states) {// For all states, find the closest state that is not a child:

	std::vector<std::array<size_t, 2>> cross_connections;

	for (size_t i = 0; i < states.size(); ++i) {

		double min_distance = std::numeric_limits<double>::infinity();
		size_t closest = 0;

		for (size_t j = i+1; j < states.size(); ++j) {
			if (std::find(states[i].children.begin(), states[i].children.end(), j) == states[i].children.end()) {
				double distance = (states[i].state.base_tf.translation - states[j].state.base_tf.translation).norm();
				if (distance < min_distance) {
					min_distance = distance;
					closest = j;
				}
			}
		}

		cross_connections.push_back({i, closest});

	}

	return cross_connections;
}

REGISTER_VISUALIZATION(coverage_rrt) {

	// Here, we wish to generate a motion whereby the robot approaches an apple,
	// and effectively "reaches around" it with its arm, creating a kind of "hook" shape with the arm.

	robot_model::RobotModel robot =
			experiments::createProceduralRobotModel(
					{
							.total_arm_length = 1.0,
							.joint_types = {experiments::JointType::HORIZONTAL,
											experiments::JointType::VERTICAL
							},
							.add_spherical_wrist = false
					});

	math::Vec3d goal_position{0.0, 0.0, 0.0};
	const double fruit_radius = 0.2;
	viewer.addSphere(fruit_radius, goal_position, FRUIT_COLOR);

	viewer.setCameraTransform({0.0, 8.0, 4.0}, {0.0, 0.0, 0.0});

	RobotState start_state = {
			.base_tf = math::Transformd::fromTranslation({0.0, -2.0, 0.0}),
			.joint_values = std::vector<double>(robot.count_joint_variables(), 0.0)
	};

	auto robot_viz = vizualisation::vizualize_robot_state(viewer,
														  robot,
														  forwardKinematics(robot,
																			start_state.joint_values,
																			0,
																			start_state.base_tf));

	random_numbers::RandomNumberGenerator rng;

	std::vector<GraphNode> states = pseudoRRTStar(robot, goal_position, fruit_radius, start_state, rng);

	// We'll visualize it using a network of lines connecting the base of the robot between linked states:
	std::vector<std::pair<math::Vec3d, math::Vec3d>> lines;

	for (const auto& state: states) {
		for (size_t child: state.children) {
			lines.push_back({state.state.base_tf.translation, states[child].state.base_tf.translation});
		}
	}

	VtkLineSegmentsVisualization lines_viz { 1,0,0 };
	lines_viz.updateLine(lines);
	viewer.addActor(lines_viz.getActor());

	// Idea: "Greatest possible shortcut" (to try later)
	// Ide being: create the edge that connects the two states that are the furthest apart by graph distance, replacing it with just that edge.
	std::vector<std::array<size_t, 2>> cross_connections = mkCrossConnections(states);

	// Now, we'll visualize the cross connections in a different color:
	std::vector<std::pair<math::Vec3d, math::Vec3d>> cross_lines;

	for (const auto& connection: cross_connections) {
		cross_lines.push_back({states[connection[0]].state.base_tf.translation, states[connection[1]].state.base_tf.translation});
	}

	VtkLineSegmentsVisualization cross_lines_viz { 0,1,0 };
	cross_lines_viz.updateLine(cross_lines);
	viewer.addActor(cross_lines_viz.getActor());

	RobotPath in_order_walk;

	// Perform a depth-first in-order tree walk of the graph:
	struct StackElement {
		size_t node;
		size_t child;
	};

	std::vector<StackElement> stack {
		{0, 0}
	};

	while (!stack.empty()) {
		StackElement top = stack.back();
		stack.pop_back();

		in_order_walk.append(states[top.node].state);

		if (top.child < states[top.node].children.size()) {
			stack.push_back({top.node, top.child + 1});
			stack.push_back({states[top.node].children[top.child], 0});
		}
	}

	PathPoint path_point = {0, 0.0};
	const double interpolation_speed = 0.2;
	const RobotPath& final_path = in_order_walk;

	// Register the timer callback function to be called at regular intervals
	viewer.addTimerCallback([&]() {
		// Advance the path point along the path
		if (advancePathPointWrap(in_order_walk, path_point, interpolation_speed, equal_weights_max_distance)) {
			viewer.stop();
		}

		// Interpolate the robot's state
		auto interpolated_state = interpolate(path_point, final_path);

		// Update the robot's state in the visualization
		const auto fk = forwardKinematics(robot, interpolated_state.joint_values,
										  robot.findLinkByName("flying_base"), interpolated_state.base_tf);

		update_robot_state(robot, fk, robot_viz);

	});

	viewer.start();

}
