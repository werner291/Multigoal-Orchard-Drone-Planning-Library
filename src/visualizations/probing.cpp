// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/19/24.
//

#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>
#include <random_numbers/random_numbers.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include "../planning/RobotModel.h"
#include "../experiment_utils/procedural_robot_models.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/fcl_utils.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../planning/ConvexHullSpace.h"
#include "../planning/state_tools.h"
#include "../planning/collision_detection.h"
#include "../planning/goal_sampling.h"
#include "../visualization/VtkTriangleSetVisualization.h"
#include "../visualization/robot_state.h"
#include "../planning/visitation_order.h"
#include "../planning/RobotPath.h"
#include "../planning/shell_path.h"
#include "../planning/approach_path_planning.h"
#include "../planning/shell_path_assembly.h"
#include "../planning/probing_motions.h"
#include <vtkActor.h>

#include "../visualization/visualization_function_macros.h"

using namespace mgodpl;
using namespace mgodpl::cgal;
using namespace mgodpl::approach_planning;
using namespace mgodpl::shell_path_planning;

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

VtkTriangleSetVisualization convex_hull_viz(const Surface_mesh& convex_hull)
{
	VtkTriangleSetVisualization viz(0.8, 0.8, 0.8, 0.5);

	std::vector<std::array<math::Vec3d, 3>> convex_hull_triangles;

	for (const auto& face : convex_hull.faces())
	{
		auto vit = convex_hull.vertices_around_face(convex_hull.halfedge(face)).begin();

		Point_3 a = convex_hull.point(*vit++);
		Point_3 b = convex_hull.point(*vit++);
		Point_3 c = convex_hull.point(*vit++);

		convex_hull_triangles.push_back({
			math::Vec3d{a.x(), a.y(), a.z()},
			math::Vec3d{b.x(), b.y(), b.z()},
			math::Vec3d{c.x(), c.y(), c.z()}
		});
	}

	viz.updateTriangles(convex_hull_triangles);
	return viz;
}

std::vector<bool> visited_by_path(const std::vector<math::Vec3d>& targets, const RobotPath& path,
                                  const robot_model::RobotModel& robot)
{
	std::vector<bool> visited(targets.size(), false);

	auto end_effector = robot.findLinkByName("end_effector");

	for (const auto& state : path.states)
	{
		auto fk = robot_model::forwardKinematics(robot, state.joint_values, robot.findLinkByName("flying_base"),
		                                         state.base_tf);
		auto ee_pose = fk.forLink(end_effector).translation;

		for (size_t i = 0; i < targets.size(); ++i)
		{
			if ((targets[i] - ee_pose).norm() < 0.01)
			{
				visited[i] = true;
			}
		}
	}

	return visited;
}

REGISTER_VISUALIZATION(probing_fullpath)
{
	const auto& robot = experiments::createProceduralRobotModel();

	const auto& tree_model = tree_meshes::loadTreeMeshes("appletree");

	auto start_time = std::chrono::high_resolution_clock::now();

	// Create a state outside the tree model.
	RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

	robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

	const auto& tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	CgalMeshData mesh_data(tree_model.leaves_mesh);

	// First, get some stats on how many straight-in motions we can do.
	const std::vector<math::Vec3d>& targets = computeFruitPositions(tree_model);

	// Plan the final path as a whole:
	RobotPath final_path = plan_multigoal_path(robot, tree_model, initial_state);

	auto end_time = std::chrono::high_resolution_clock::now();

	std::cout << "Planning took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).
		count() << " ms." << std::endl;

	// Check collision-freeness.
	{
		PathPoint collision_point;
		if (check_path_collides(robot, tree_trunk_object, final_path, collision_point))
		{
			std::cout << "Final path collides at segment " << collision_point.segment_i << " at " << collision_point.
				segment_t << std::endl;
		}
		else
		{
			std::cout << "Final path is collision-free." << std::endl;
		}
	}

	// Run visualization of the final result.

	// Lock the camera's up direction to prevent it from rotating
	viewer.lockCameraUp();

	// Add the tree model's trunk mesh to the viewer with a wood color
	viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

	// Determine which targets have been visited by the path
	const auto& visited = visited_by_path(targets, final_path, robot);

	// For each target, add a sphere to the viewer at the target's position
	// The sphere's color is yellow if the target has been visited, and red otherwise
	for (size_t target_i = 0; target_i < targets.size(); ++target_i)
	{
		// Yellow if visited, red if not.
		math::Vec3d color = visited[target_i] ? math::Vec3d{1, 1, 0} : math::Vec3d{1, 0, 0};

		viewer.addSphere(0.05, targets[target_i], color, 1.0);
	}

	// Create a visualization of the convex hull and add it to the viewer
	VtkTriangleSetVisualization chull_viz = convex_hull_viz(mesh_data.convex_hull);
	viewer.addActor(chull_viz.getActor());

	// Visualize the initial state of the robot
	auto robot_viz = mgodpl::vizualisation::vizualize_robot_state(viewer, robot,
	                                                              robot_model::forwardKinematics(
		                                                              robot, initial_state.joint_values,
		                                                              flying_base, initial_state.base_tf));

	// Initialize the segment time
	double segment_t = 0.0;

	// Add a timer callback to the viewer that will be called at regular intervals
	viewer.addTimerCallback([&]()
	{
		// Determine the current segment index and the fractional part of the segment time
		size_t segment_i = std::floor(segment_t);
		double segment_t_frac = segment_t - (double)segment_i;

		// If we have reached the end of the path, stop the viewer
		if (segment_i + 1 >= final_path.states.size())
		{
			viewer.stop();
			return;
		}
		else
		{
			std::cout << "Segment " << segment_i << " / " << final_path.states.size() << std::endl;
		}

		// Interpolate between the current and next state based on the fractional part of the segment time
		const auto& state1 = final_path.states[segment_i];
		const auto& state2 = final_path.states[segment_i + 1];
		auto interpolated_state = interpolate(state1, state2, segment_t_frac);

		// Compute the forward kinematics for the interpolated state
		auto fk = robot_model::forwardKinematics(robot, interpolated_state.joint_values, flying_base,
		                                         interpolated_state.base_tf);

		// Check if the interpolated state collides with the tree trunk
		bool collides = check_robot_collision(robot, tree_trunk_object, interpolated_state);

		// Update the robot's state in the visualization
		update_robot_state(robot, fk, robot_viz);

		// Compute the length of the segment
		double segment_length = equal_weights_max_distance(state1, state2);

		// If the segment length is very small, set it to a minimum value to prevent division by zero
		if (segment_length < 0.1)
		{
			segment_length = 0.1;
		}

		// If the state collides, advance the segment time slowly, otherwise advance it faster
		if (collides)
		{
			segment_t += 0.01 / segment_length;
		}
		else
		{
			segment_t += 0.05 / segment_length;
		}
	});

	// Set the camera's position and target
	viewer.setCameraTransform({8, 0, 2}, {0, 0, 2});

	// Start the viewer
	viewer.start();
}
