// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-11-23.
//

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

#include "../math/Triangle.h"
#include "../planning/BlindlyMoveToNextFruit.h"
#include "../planning/moveit_state_tools.h"
#include "../visibility/GridVec.h"
#include "../visibility/voxel_visibility.h"

#include "VoxelShroudedSceneInfo.h"
#include "VoxelShroudedSimulation.h"
#include "mesh_utils.h"


using namespace mgodpl;
using namespace moveit_facade;
using namespace math;

Vec3d toVec3d(const geometry_msgs::msg::Point &p) {
	return {p.x, p.y, p.z};
}

std::vector<Triangle> extractOccludingTriangles(const tree_meshes::TreeMeshes &tree_model) {
	std::vector<Triangle> triangles;

	// For every leaf in the tree, set the corresponding grid cell to true.
	for (const auto &triangle: tree_model.leaves_mesh.triangles) {

		Vec3d a = toVec3d(tree_model.leaves_mesh.vertices[triangle.vertex_indices[0]]);
		Vec3d b = toVec3d(tree_model.leaves_mesh.vertices[triangle.vertex_indices[1]]);
		Vec3d c = toVec3d(tree_model.leaves_mesh.vertices[triangle.vertex_indices[2]]);

		triangles.emplace_back(a, b, c);

	}
	return triangles;
}

std::vector<Vec3d> integrate_seen_space(const Grid3D<std::vector<Vec3d>> &apples_in_cells,
										Grid3D<bool> &seen_space,
										const Grid3D<bool> &occluded_space) {

	std::vector<Vec3d> newly_detected_fruits;

	for (size_t x = 0; x < seen_space.size().x(); x++) {
		for (size_t y = 0; y < seen_space.size().y(); y++) {
			for (size_t z = 0; z < seen_space.size().z(); z++) {
				Vec3i coords = {(int) x, (int) y, (int) z};

				if (!seen_space[coords] && !occluded_space[coords]) {
					seen_space[coords] = true;
					for (const auto &apple: apples_in_cells[coords]) {
						newly_detected_fruits.push_back(apple);
					}
				}
			}
		}
	}

	return newly_detected_fruits;
}

simulation::VoxelShroudedSimulation::VoxelShroudedSimulation(moveit::core::RobotModelPtr robot_model,
															 tree_meshes::TreeMeshes tree_model,
															 int seed,
															 const double stepSize)
		: robot_model(std::move(robot_model)),
		  tree_model(std::move(tree_model)),
		  fruit_positions(this->tree_model.fruit_meshes | ranges::views::transform([](const auto &mesh) {
			  return mesh_aabb(mesh).center();
		  }) | ranges::to<std::vector>()),
		  algorithm(std::make_shared<planning::BlindlyMoveToNextFruit>(this->robot_model)),
		  current_state(experiment_state_tools::randomStateOutsideTree(*this->robot_model, seed)),
		  collision_detection({this->tree_model.trunk_mesh}, this->robot_model),
		  grid_coords(AABBd(Vec3d(-3.0, -3.0, 0.0), Vec3d(3.0, 3.0, 6.0)), 50, 50, 50),
		  seen_space(grid_coords.size(), false),
		  occluded_space(grid_coords.size(), false),
		  apples_in_cells(grid_coords.size(), {}),
		  step_size(stepSize) {

	// Create the grid index of apples in cells.
	for (const auto &apple: fruit_positions) {
		Vec3i coords = grid_coords.getGridCoordinates(apple).value();
		apples_in_cells[coords].push_back(apple);
	}

	// Initialize the occluded space.
	occluded_space = voxel_visibility::cast_occlusion(grid_coords,
													  extractOccludingTriangles(this->tree_model),
													  computeEndEffectorPosition(*this->robot_model, current_state));

	// Update the seen space.
	auto new_apples = integrate_seen_space(apples_in_cells, seen_space, occluded_space);

	// Update the algorithm.
	next_state = algorithm->nextMovement(experiments::VoxelShroudedSceneInfoUpdate::make_filtered(this->tree_model,
																								  grid_coords,
																								  seen_space,
																								  occluded_space,
																								  current_state,
																								  new_apples));

}

void simulation::VoxelShroudedSimulation::update() {

	if (!next_state.has_value()) {
		return;
	}

	double distance = moveit_joint_distance(*robot_model, current_state, *next_state);

	if (distance < step_size) {
		total_distance += distance;
		current_state = *next_state;
	} else {
		total_distance += step_size;
		current_state = interpolate(*robot_model, current_state, *next_state, step_size / distance);
	}

	if (collision_detection.collides(current_state)) {
		std::cout << "Collision!" << std::endl;
		hasCollided = true;
	}

	// Update the occluded space.
	occluded_space = voxel_visibility::cast_occlusion(grid_coords,
													  extractOccludingTriangles(tree_model),
													  computeEndEffectorPosition(*robot_model, current_state));

	// Update the seen space.
	auto new_apples = integrate_seen_space(apples_in_cells, seen_space, occluded_space);

	// Update the algorithm.
	next_state = algorithm->nextMovement(experiments::VoxelShroudedSceneInfoUpdate::make_filtered(tree_model,
																								  grid_coords,
																								  seen_space,
																								  occluded_space,
																								  current_state,
																								  new_apples));

}

bool simulation::VoxelShroudedSimulation::is_done() const {
	return !next_state.has_value();
}
