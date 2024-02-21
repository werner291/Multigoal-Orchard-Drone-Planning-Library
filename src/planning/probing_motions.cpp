// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision.h>
#include "probing_motions.h"
#include "../experiment_utils/fcl_utils.h"
#include "collision_detection.h"
#include "shell_path_assembly.h"
#include "approach_path_planning.h"
#include "shell_path.h"
#include "visitation_order.h"
#include "goal_sampling.h"

using namespace mgodpl;
using namespace cgal;
using namespace approach_planning;

mgodpl::ApproachPath mgodpl::straightout(const mgodpl::robot_model::RobotModel &robot,
										 const mgodpl::RobotState &target_state,
										 const CGAL::AABB_tree<cgal::AABBTraits> &tree,
										 const mgodpl::cgal::Surface_mesh_shortest_path &algo) {

	auto fk = robot_model::forwardKinematics(robot,
											 target_state.joint_values,
											 robot.findLinkByName("flying_base"),
											 target_state.base_tf);

	math::Vec3d end_effector_position = fk.forLink(robot.findLinkByName("end_effector")).translation;
	auto exit_vector = -arm_axis(robot, fk);

	auto isect = tree.first_intersection(mgodpl::cgal::Ray_3(to_cgal_point(end_effector_position),
															 to_cgal_direction(exit_vector)));

	if (!isect.has_value()) {
		// We're out of the tree; try the opposite direction.
		exit_vector = -exit_vector;
		isect = tree.first_intersection(mgodpl::cgal::Ray_3(to_cgal_point(end_effector_position),
															to_cgal_direction(exit_vector)));

		assert(isect.has_value());
	}

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

RobotPath mgodpl::plan_multigoal_path(const robot_model::RobotModel &robot,
							  const tree_meshes::TreeMeshes &tree_model,
							  const RobotState &initial_state) {// Create a collision object for the tree trunk.

	auto flying_base = robot.findLinkByName("flying_base");

	// Allocate a BVH convex_hull for the tree trunk.
	const auto &tree_trunk_bvh = fcl_utils::meshToFclBVH(tree_model.trunk_mesh);
	fcl::CollisionObjectd tree_trunk_object(tree_trunk_bvh);

	random_numbers::RandomNumberGenerator rng(42);

	// First, create the convex hull.
	CgalMeshData mesh_data(tree_model.leaves_mesh);

	ApproachPath initial_approach_path = plan_initial_approach_path(robot,
																	initial_state,
																	flying_base,
																	mesh_data);

	std::vector <ApproachPath> approach_paths;

	// For every fruit position...
	for (const auto &tgt: computeFruitPositions(tree_model)) {
		auto straightout = uniform_straightout_approach(tgt, robot, tree_trunk_object, mesh_data, rng, 1000);

		if (straightout) {
			approach_paths.push_back(*straightout);
		}
	}

	// And one for the initial state:
	const std::vector<double> &initial_state_distances = shell_distances(initial_approach_path.shell_point,
																		 approach_paths,
																		 mesh_data.convex_hull);

	// Now, compute the distance matrix.
	std::vector <std::vector<double>> target_to_target_distances;
	target_to_target_distances.reserve(approach_paths.size());
	for (const ApproachPath &path1: approach_paths) {
		target_to_target_distances.emplace_back(shell_distances(path1.shell_point,
																approach_paths,
																mesh_data.convex_hull));
	}

	const std::vector <size_t> &order = visitation_order_greedy(target_to_target_distances, initial_state_distances);

	const RobotPath &final_path = mgodpl::shell_path_planning::assemble_final_path(robot,
													  mesh_data.convex_hull,
													  approach_paths,
													  initial_approach_path,
													  order);
	return final_path;
}

std::optional<ApproachPath> mgodpl::uniform_straightout_approach(const math::Vec3d &target,
														 const robot_model::RobotModel &robot,
														 const fcl::CollisionObjectd &tree_trunk_object,
														 const CgalMeshData &mesh_data,
														 random_numbers::RandomNumberGenerator &rng,
														 size_t max_attempts,
														 double ee_distance) {


	const auto flying_base = robot.findLinkByName("flying_base");
	const auto end_effector = robot.findLinkByName("end_effector");

	// auto sample = findGoalStateByUniformSampling(target,
	// 											 robot,
	// 											 flying_base,
	// 											 end_effector,
	// 											 tree_trunk_object,
	// 											 rng,
	// 											 max_attempts);

	auto sample = generateUniformRandomArmVectorState(robot, tree_trunk_object, target, rng, max_attempts, ee_distance);

	if (!sample) {
		return std::nullopt;
	}

	// Then, try the straight-out motion:
	auto path = straightout(robot, *sample, mesh_data.tree, mesh_data.mesh_path);

	PathPoint collision_point{};
	if (!check_path_collides(robot, tree_trunk_object, path.path, collision_point)) {
		return path;
	}

	return std::nullopt;

}
