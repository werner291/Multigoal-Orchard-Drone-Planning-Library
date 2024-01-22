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
#include "../planning/visitation_order.h"
#include "../planning/RobotPath.h"
#include "../planning/shell_path.h"
#include <vtkActor.h>

using namespace mgodpl;
using namespace mgodpl::cgal;

const math::Vec3d WOOD_COLOR{0.5, 0.3, 0.1};
const math::Vec3d FLOOR_COLOR{0.3, 0.6, 0.3};

ApproachPath plan_initial_approach_path(const robot_model::RobotModel &robot,
										const RobotState &initial_state,
										const robot_model::RobotModel::LinkId flying_base,
										const Surface_mesh &convex_hull,
										const Surface_mesh_shortest_path &mesh_path,
										const CGAL::AABB_tree<mgodpl::cgal::AABBTraits> &tree) {


	ApproachPath initial_approach_path;

	auto initial_fk = robot_model::forwardKinematics(robot, initial_state.joint_values,
													 flying_base, initial_state.base_tf);

	// The convex hull point closest to the initial state.
	const math::Vec3d &ee_pose = initial_fk.forLink(flying_base).translation;
	const auto &[initial_face, initial_bary] = mesh_path.locate(Point_3(ee_pose.x(), ee_pose.y(), ee_pose.z()),
																tree);
	auto face_normal = CGAL::Polygon_mesh_processing::compute_face_normal(initial_face, convex_hull);
	auto pt = mesh_path.point(initial_face, initial_bary);

	// Put the robot state outside the tree in that point:
	auto robot_state = fromEndEffectorAndVector(robot,
												{pt.x(), pt.y(), pt.z()},
												{face_normal.x(), face_normal.y(), face_normal.z()});

	initial_approach_path.path.states = {initial_state, robot_state};
	initial_approach_path.shell_point = {initial_face, initial_bary};

	return initial_approach_path;
}

ApproachPath straight_in_motion(const robot_model::RobotModel &robot,
								const Surface_mesh &convex_hull,
								const Surface_mesh_shortest_path &mesh_path,
								const CGAL::AABB_tree<mgodpl::cgal::AABBTraits> &tree,
								const math::Vec3d &tgt) {

	// Find the nearest point on the hull.
	const auto &[face, barycentric] = mesh_path.locate(Point_3(tgt.x(), tgt.y(), tgt.z()), tree);

	// Find the normal vector:
	auto face_normal = CGAL::Polygon_mesh_processing::compute_face_normal(face, convex_hull);

	// Find the Cartesian coordinates of the point on the hull.
	const auto &pt = mesh_path.point(face, barycentric);

	// Put the robot state outside the tree in that point:
	auto robot_state = fromEndEffectorAndVector(robot,
												{pt.x(), pt.y(), pt.z()},
												{face_normal.x(), face_normal.y(), face_normal.z()});

	// Create a copy that has the end-effector at the target.
	auto robot_state_at_target = fromEndEffectorAndVector(robot,
														  tgt,
														  {face_normal.x(), face_normal.y(), face_normal.z()});

	// Create a path from the current state to the target state.
	ApproachPath path{
			.path = {.states={robot_state, robot_state_at_target}}, // TODO: actually compute the path.
			.shell_point = {face, barycentric}
	};

	return path;
}

RobotPath assemble_final_path(const robot_model::RobotModel &robot,
							  const Surface_mesh &convex_hull,
							  const std::vector<ApproachPath> &approach_paths,
							  ApproachPath &initial_approach_path,
							  const std::vector<size_t> &order) {// Finally, assemble the final path.
	RobotPath final_path;

	{
		const ApproachPath *last_path = &initial_approach_path;

		// We're going to do retreat-move-probe paths: backing away from one path, moving to the start of next, and then probing.
		for (size_t i = 0; i < approach_paths.size(); ++i) {

			final_path.states.insert(final_path.states.end(),
									 last_path->path.states.rbegin(),
									 last_path->path.states.rend());

			const auto &next_path = approach_paths[order[i]];

			auto between_paths = shell_path(last_path->shell_point, next_path.shell_point, convex_hull, robot);
			final_path.states.insert(final_path.states.end(), between_paths.states.begin(), between_paths.states.end());

			final_path.states.insert(final_path.states.end(),
									 next_path.path.states.begin(),
									 next_path.path.states.end());

			last_path = &next_path;

		}
	}
	return final_path;
}

struct RobotActors {
	std::vector<vtkSmartPointer<vtkActor>> actors;
};

RobotActors vizualize_robot_state(SimpleVtkViewer &viewer,
								  const robot_model::RobotModel &robot,
								  const robot_model::ForwardKinematicsResult &fk,
								  const math::Vec3d& color = {0.8,0.8,0.8}) {

	std::vector<vtkSmartPointer<vtkActor>> actors;

	for (size_t link_id = 0; link_id < robot.getLinks().size(); ++link_id) {
		auto link_tf = fk.forLink(link_id);

		// If it has visual shapes, add them.
		if (const auto &visual_geometry = robot.getLinks()[link_id].visual_geometry; !visual_geometry.empty()) {
			for (const auto &shape: visual_geometry) {
				PositionedShape global{
						.shape = shape.shape,
						.transform = link_tf.then(shape.transform)
				};
				actors.push_back(viewer.addPositionedShape(global, color, 1.0));
			}
		} else {
			// Use the collision geometry as a fallback.
			for (const auto &shape: robot.getLinks()[link_id].collision_geometry) {
				PositionedShape global{
						.shape = shape.shape,
						.transform = link_tf.then(shape.transform)
				};
				actors.push_back(viewer.addPositionedShape(global, color, 1.0));
			}
		}
	}

	return RobotActors{.actors = actors};
}

void update_robot_state(const robot_model::RobotModel &robot,
						const robot_model::ForwardKinematicsResult &fk,
						RobotActors &actors) {

	auto actor_it = actors.actors.begin();

	for (size_t link_id = 0; link_id < robot.getLinks().size(); ++link_id) {
		auto link_tf = fk.forLink(link_id);

		// If it has visual shapes, add them.
		if (const auto &visual_geometry = robot.getLinks()[link_id].visual_geometry; !visual_geometry.empty()) {
			for (const auto &shape: visual_geometry) {
				PositionedShape global{
						.shape = shape.shape,
						.transform = link_tf.then(shape.transform)
				};
				SimpleVtkViewer::set_transform(global.transform, actor_it->Get());
				++actor_it;
			}
		} else {
			// Use the collision geometry as a fallback.
			for (const auto &shape: robot.getLinks()[link_id].collision_geometry) {
				PositionedShape global{
						.shape = shape.shape,
						.transform = link_tf.then(shape.transform)
				};
				SimpleVtkViewer::set_transform(global.transform, actor_it->Get());
				++actor_it;
			}
		}
	}
}

mgodpl::cgal::Surface_mesh cgal_convex_hull_around_leaves(const tree_meshes::TreeMeshes &tree_model) {
	Surface_mesh convex_hull;
	{
		std::vector<Point_3> cgal_points;
		for (const auto &point: tree_model.leaves_mesh.vertices) {
			cgal_points.emplace_back(point.x, point.y, point.z);
		}
		CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), convex_hull);
	}
	return convex_hull;
}


bool check_motion_collides(const robot_model::RobotModel &robot,
						   const fcl::CollisionObjectd &tree_trunk_object,
						   const RobotState &state1,
						   const RobotState &state2,
						   double &toi) {

	// Compute the distance between the two.
	double distance = equal_weights_distance(state1, state2);
	const double MAX_STEP = 0.1;

	size_t n_steps = std::ceil(distance / MAX_STEP);

	for (size_t step_i = 0; step_i < n_steps; ++step_i) {
		double t = (double) step_i / (double) n_steps;
		auto interpolated_state = interpolate(state1, state2, t);

		// Check if the robot is in collision at the interpolated state.
		if (check_robot_collision(robot, tree_trunk_object, interpolated_state)) {
			toi = t;
			return true;
		}
	}
	return false;
}

math::Vec3d arm_axis(const robot_model::RobotModel &robot, const robot_model::ForwardKinematicsResult &fk) {
	return fk.forLink(robot.findLinkByName("stick")).orientation.rotate(math::Vec3d::UnitY());
}

mgodpl::cgal::Point_3 to_cgal_point(const math::Vec3<double> &v) {
	return {v.x(), v.y(), v.z()};
}

mgodpl::cgal::Direction_3 to_cgal_direction(const math::Vec3<double> &v) {
	return {v.x(), v.y(), v.z()};
}

/**
 * A method to find a path from a given target point out of the tree by moving along the axis of the arm.
 * @return
 */
ApproachPath straightout(const robot_model::RobotModel &robot,
						 const RobotState& target_state,
						 const CGAL::AABB_tree<AABBTraits>& tree,
						 const Surface_mesh_shortest_path & algo) {

	auto fk = robot_model::forwardKinematics(robot,
											 target_state.joint_values,
											 robot.findLinkByName("flying_base"),
											 target_state.base_tf);

	math::Vec3d end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;
	auto exit_vector = -arm_axis(robot, fk);

	auto isect = tree.first_intersection(mgodpl::cgal::Ray_3(to_cgal_point(end_effector_position), to_cgal_direction(exit_vector)));

	assert(isect.has_value());

	Point_3 intersection = boost::get<Point_3>(isect->first);

	// Get a barycentric coordinate for the intersection point.
	auto [face, barycentric] = algo.locate(intersection, tree);

	math::Vec3d delta = math::Vec3d(intersection.x(), intersection.y(), intersection.z()) - end_effector_position;

	RobotState outside_tree = target_state;
	outside_tree.base_tf.translation = outside_tree.base_tf.translation + delta;

	return {
		.path = {
			.states = {outside_tree, target_state}
		},
		.shell_point = {face, barycentric}
	};

}

struct PathPoint {
	size_t segment_i;
	double segment_t;
};

bool check_path_collides(const robot_model::RobotModel &robot,
						 const fcl::CollisionObjectd &tree_trunk_object,
						 const RobotPath &path,
						 PathPoint& collision_point) {
	for (size_t segment_i = 0; segment_i + 1 < path.states.size(); ++segment_i) {
		const auto &state1 = path.states[segment_i];
		const auto &state2 = path.states[segment_i + 1];

		double toi;
		if (check_motion_collides(robot, tree_trunk_object, state1, state2, toi)) {
			collision_point.segment_i = segment_i;
			return true;
		}
	}
	return false;
}

/**
 * A method that combined straightout with uniform random sampling.
 */
std::optional<ApproachPath> uniform_straightout_approach(const mgodpl::math::Vec3d &target,
													  const mgodpl::robot_model::RobotModel &robot,
													  const fcl::CollisionObjectd &tree_trunk_object,
													  const CGAL::AABB_tree<AABBTraits>& tree,
													  const Surface_mesh_shortest_path &mesh_path,
													  random_numbers::RandomNumberGenerator& rng,
													  size_t max_attempts
													  ) {


	const auto flying_base = robot.findLinkByName("flying_base");
	const auto end_effector = robot.findLinkByName("end_effector");

	auto sample = findGoalStateByUniformSampling(target,
												 robot,
												 flying_base,
												 end_effector,
												 tree_trunk_object,
												 rng,
												 max_attempts);

	if (!sample) {
		return std::nullopt;
	}

	// Then, try the straight-out motion:
	auto path = straightout(robot, *sample, tree, mesh_path);

	PathPoint collision_point {};
	if (!check_path_collides(robot, tree_trunk_object, path.path, collision_point)) {
		return path;
	}

	return std::nullopt;

}

RobotPath plan_multigoal_path(const robot_model::RobotModel &robot,
							  const tree_meshes::TreeMeshes &tree_model,
							  const RobotState &initial_state) {// Create a collision object for the tree trunk.

	auto flying_base = robot.findLinkByName("flying_base");

	// Allocate a BVH convex_hull for the tree trunk.
	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	random_numbers::RandomNumberGenerator rng(42);

	// First, create the convex hull.
	Surface_mesh convex_hull = cgal_convex_hull_around_leaves(tree_model);
	Surface_mesh_shortest_path mesh_path(convex_hull);
	CGAL::AABB_tree<AABBTraits> tree;
	mesh_path.build_aabb_tree(tree);

	ApproachPath initial_approach_path = plan_initial_approach_path(robot,
																	initial_state,
																	flying_base,
																	convex_hull,
																	mesh_path,
																	tree);

	std::vector<ApproachPath> approach_paths;

	// For every fruit position...
	for (const auto &tgt: computeFruitPositions(tree_model)) {
		auto straightout = uniform_straightout_approach(tgt, robot, tree_trunk_object, tree, mesh_path, rng, 1000);

		if (straightout) {
			approach_paths.push_back(*straightout);
		}
	}

	// And one for the initial state:
	const std::vector<double>& initial_state_distances = shell_distances(initial_approach_path.shell_point,
																		 approach_paths,
																		 convex_hull);

	// Now, compute the distance matrix.
	std::vector<std::vector<double>> target_to_target_distances;
	target_to_target_distances.reserve(approach_paths.size());
	for (const ApproachPath &path1: approach_paths) {
		target_to_target_distances.emplace_back(shell_distances(path1.shell_point, approach_paths, convex_hull));
	}

	const std::vector<size_t>& order = visitation_order_greedy(target_to_target_distances,initial_state_distances);

	const RobotPath& final_path = assemble_final_path(robot, convex_hull, approach_paths, initial_approach_path, order);
	return final_path;
}

VtkTriangleSetVisualization convex_hull_viz(const Surface_mesh &convex_hull) {
	VtkTriangleSetVisualization viz(0.8, 0.8, 0.8, 0.5);

	std::vector<std::array<math::Vec3d, 3>> convex_hull_triangles;

	for (const auto &face: convex_hull.faces()) {
		auto vit = convex_hull.vertices_around_face(convex_hull.halfedge(face)).begin();

		Point_3 a = convex_hull.point(*vit++);
		Point_3 b = convex_hull.point(*vit++);
		Point_3 c = convex_hull.point(*vit++);

		convex_hull_triangles.push_back({
			math::Vec3d {a.x(), a.y(), a.z()},
			math::Vec3d {b.x(), b.y(), b.z()},
			math::Vec3d {c.x(), c.y(), c.z()}
		});
	}

	viz.updateTriangles(convex_hull_triangles);
	return viz;
}

std::vector<bool> visited_by_path(const std::vector<math::Vec3d> targets, const RobotPath &path, const robot_model::RobotModel& robot) {

	std::vector<bool> visited(targets.size(), false);

	auto end_effector = robot.findLinkByName("end_effector");

	for (const auto& state: path.states) {
		auto fk = robot_model::forwardKinematics(robot, state.joint_values, robot.findLinkByName("flying_base"), state.base_tf);
		auto ee_pose = fk.forLink(end_effector).translation;

		for (size_t i = 0; i < targets.size(); ++i) {
			if ((targets[i] - ee_pose).norm() < 0.01) {
				visited[i] = true;
			}
		}
	}

	return visited;

}

// TODO: Idea: Can plan to 5 nearest and optimize *that*!

int main() {

	const auto &robot = experiments::createProceduralRobotModel();

	const auto &tree_model = tree_meshes::loadTreeMeshes("appletree");

	auto start_time = std::chrono::high_resolution_clock::now();

	// Create a state outside the tree model.
	RobotState initial_state = fromEndEffectorAndVector(robot, {0, 5, 5}, {0, 1, 1});

	robot_model::RobotModel::LinkId flying_base = robot.findLinkByName("flying_base");

	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	Surface_mesh convex_hull = cgal_convex_hull_around_leaves(tree_model);
	Surface_mesh_shortest_path mesh_path(convex_hull);
	CGAL::AABB_tree<AABBTraits> tree;
	mesh_path.build_aabb_tree(tree);

	// First, get some stats on how many straight-in motions we can do.
	const std::vector<math::Vec3d> &targets = computeFruitPositions(tree_model);
	{
		size_t successes = 0;
		for (const auto &tgt: targets) {
			auto path = straight_in_motion(robot, convex_hull, mesh_path, tree, tgt);

			PathPoint collision_point {};
			if (!check_path_collides(robot, tree_trunk_object, path.path, collision_point)) {
				++successes;
			}
		}
		std::cout << "Straight-in motion success rate: " << successes << "/" << targets.size() << std::endl;
	}

	// Now, get some stats on whether we can easily get goal states:
	{
		size_t sample_successes = 0;

		random_numbers::RandomNumberGenerator rng(42);

		for (const auto &tgt: targets) {
			auto sample = findGoalStateByUniformSampling(tgt,
														 robot,
														 flying_base,
														 robot.findLinkByName("end_effector"),
														 tree_trunk_object,
														 rng,
														 1000);

			if (sample) {
				++sample_successes;
			}
		}
		std::cout << "Goal sampling success rate: " << sample_successes << "/" << targets.size() << std::endl;
	}

	// And some stats on uniform_straightout_approach
	{
		size_t sample_successes = 0;

		random_numbers::RandomNumberGenerator rng(42);

		for (const auto &tgt: targets) {
			auto sample = uniform_straightout_approach(tgt,
														 robot,
														 tree_trunk_object,
														 tree,
													   	mesh_path,
														 rng,
														 1000);

			if (sample) {
				++sample_successes;
			}
		}
		std::cout << "Uniform straightout approach success rate: " << sample_successes << "/" << targets.size() << std::endl;
	}

	// Plan the final path as a whole:
	RobotPath final_path = plan_multigoal_path(robot, tree_model, initial_state);

	auto end_time = std::chrono::high_resolution_clock::now();

	std::cout << "Planning took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms." << std::endl;

	// Check collision-freeness.
	{
		PathPoint collision_point;
		if (check_path_collides(robot, tree_trunk_object, final_path, collision_point)) {
			std::cout << "Final path collides at segment " << collision_point.segment_i << " at " << collision_point.segment_t << std::endl;
		} else {
			std::cout << "Final path is collision-free." << std::endl;
		}
	}

	// Run visualization of the final result.
	{
		SimpleVtkViewer viewer;
		viewer.lockCameraUp();
		viewer.addMesh(tree_model.trunk_mesh, WOOD_COLOR);

		const auto& visited = visited_by_path(targets, final_path, robot);

		for (size_t target_i = 0; target_i < targets.size(); ++target_i) {

			// Yellow if visited, red if not.
			math::Vec3d color = visited[target_i] ? math::Vec3d{1,1,0} : math::Vec3d{1,0,0};

			viewer.addSphere(0.05, targets[target_i], color, 1.0);
		}


		VtkTriangleSetVisualization chull_viz = convex_hull_viz(convex_hull);
		viewer.addActor(chull_viz.getActor());

		// Visualize the initial state:
		auto robot_viz = vizualize_robot_state(viewer, robot, robot_model::forwardKinematics(robot, initial_state.joint_values, flying_base, initial_state.base_tf));

		double segment_t = 0.0;

		viewer.addTimerCallback([&]() {

			size_t segment_i = std::floor(segment_t);
			double segment_t_frac = segment_t - (double) segment_i;

			const auto &state1 = final_path.states[segment_i];
			const auto &state2 = final_path.states[segment_i + 1];

			auto interpolated_state = interpolate(state1, state2, segment_t_frac);
			auto fk = robot_model::forwardKinematics(robot, interpolated_state.joint_values, flying_base, interpolated_state.base_tf);

			bool collides = check_robot_collision(robot, tree_trunk_object, interpolated_state);

			update_robot_state(robot, fk, robot_viz);

			double segment_length = equal_weights_max_distance(state1, state2);

			if (segment_length < 0.1) {
				segment_length = 0.1;
			}

			if (collides) {
				segment_t += 0.01 / segment_length;
			} else {
				segment_t += 0.05 / segment_length;
			}

//			std::cout << "Segment " << segment_i << " at " << segment_t_frac << " of " << segment_length << std::endl;

			// If that outs it into the next segment, round it back to zero.
			if (std::floor(segment_t) > (double) segment_i) {
				segment_t = std::floor(segment_t);
			}

			if (segment_t > (double) final_path.states.size() - 1.0) {
				std::cout << "Done!" << std::endl;
				segment_t = 0.0;
			}
		});

		viewer.start();
	}

}