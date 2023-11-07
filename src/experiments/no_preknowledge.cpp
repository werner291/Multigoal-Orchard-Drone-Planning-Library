// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

#include <vtkProperty.h>
#include <unordered_set>

#include "../experiment_utils/load_robot_model.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkRobotModel.h"
#include "../planning/moveit_state_tools.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../planning/RobotAlgorithm.h"
#include "../planning/BlindlyMoveToNextFruit.h"
#include "../experiment_utils/mesh_utils.h"
#include "../planning/CollisionDetection.h"
#include "../visualization/voxels.h"
#include "../math/AABBGrid.h"
#include "../math/Vec3.h"
#include "../visibility/GridVec.h"
#include "../math/Triangle.h"
#include "../visibility/voxel_visibility.h"
#include "../experiment_utils/VoxelShroudedSceneInfo.h"
#include "../visualization/VtkEndAndBaseTraceVisualization.h"
#include "../planning/JointSpacePath.h"

static const double STEP_SIZE = 0.05;

static const double FRUIT_VISIT_DISTANCE_THRESHOLD = 0.15;

void updateTrace(const moveit::core::RobotModelPtr &robot,
				 mgodpl::moveit_facade::JointSpacePoint &current_state,
				 const std::shared_ptr<mgodpl::planning::BlindlyMoveToNextFruit> &algorithm,
				 mgodpl::visualization::VtkEndAndBaseTraceVisualization &trace_visualization) {

	mgodpl::moveit_facade::JointSpacePath interpolated_path;

	for (int segment = 0; segment < algorithm->plan.size(); ++segment) {
		const auto &a = segment == 0 ? current_state : algorithm->plan[segment - 1].point;
		const auto &b = algorithm->plan[segment].point;

		for (int i = 0; i < 20; ++i) {
			interpolated_path.path.push_back(interpolate(*robot, a, b, i / 20.0));
		}
	}

	trace_visualization.updateLine(interpolated_path);
}


using namespace mgodpl;
using namespace visualization;
using namespace experiment_state_tools;
using namespace moveit_facade;
using namespace planning;
using namespace math;

Vec3d toVec3d(const geometry_msgs::msg::Point &p) {
	return {p.x, p.y, p.z};
}

// hash for unordered map of vec3d
namespace std {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedStructInspection" // Clang is wrong; I use this in the unordered_map of actors.
	template<>
	struct hash<Vec3d> {
		size_t operator()(const Vec3d &v) const {
			return std::hash<double>()(v.x()) ^ std::hash<double>()(v.y()) ^ std::hash<double>()(v.z());
		}
	};
#pragma clang diagnostic pop
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

int main() {

	// TODO: Reminder, write down the idea of the snake path!

	const auto &robot = experiment_assets::loadRobotModel(1.0);

	const auto &tree_model = tree_meshes::loadTreeMeshes("appletree");

	const auto &fruit_positions = tree_model.fruit_meshes | ranges::views::transform([](const auto &mesh) {
		return mesh_aabb(mesh).center();
	}) | ranges::to<std::vector>();

	SimpleVtkViewer viewer;
	viewer.lockCameraUp();

	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1});
	viewer.addMesh(tree_model.leaves_mesh, {0.1, 0.5, 0.1});
	viewer.addMesh(createGroundPlane(5.0, 5.0), {0.3, 0.5, 0.1});

	// Create an actor for every fruit:
	std::unordered_map<math::Vec3d, vtkActor *> fruit_actors;

	for (size_t i = 0; i < fruit_positions.size(); i++) {
		fruit_actors[fruit_positions[i]] = viewer.addMesh(tree_model.fruit_meshes[i], {0.5, 0.1, 0.1}).Get();
	}

	JointSpacePoint current_state = randomStateOutsideTree(*robot, 0);

	visualization::VtkRobotModel robotModelViz(robot, current_state, {0.5, 0.5, 0.5});
	viewer.addActorCollection(robotModelViz.getLinkActors());

	const auto algorithm = std::make_shared<BlindlyMoveToNextFruit>(robot);

	std::unordered_set<math::Vec3d> visited_fruits;
	std::unordered_set<math::Vec3d> rejected_fruits;

	algorithm->on_target_rejected = [&](const Vec3d &fruit) {

		// If it was visited, abort, this is wrong.
		if (visited_fruits.find(fruit) != visited_fruits.end()) {
			std::cout << "Visited fruit rejected!" << std::endl;
			exit(1);
		}

		fruit_actors[fruit]->GetProperty()->SetColor(0.0, 0.0, 0.0);
		rejected_fruits.insert(fruit);
	};

	algorithm->on_target_reached = [&](const Vec3d &fruit) {

		// If it was rejected, abort, this is wrong.
		if (rejected_fruits.find(fruit) != rejected_fruits.end()) {
			std::cout << "Rejected fruit reached: " << fruit << std::endl;
			exit(1);
		}

		fruit_actors[fruit]->GetProperty()->SetColor(1.0, 1.0, 0.0);
		visited_fruits.insert(fruit);
	};


	std::optional<JointSpacePoint> next_state{};

	double total_distance = 0.0;

	CollisionDetection collision_detection({tree_model.trunk_mesh}, robot);

	const size_t SUBDIVISIONS = 50;

	AABBGrid grid_coords(AABBd(Vec3d(-3.0, -3.0, 0.0), Vec3d(3.0, 3.0, 6.0)), SUBDIVISIONS, SUBDIVISIONS, SUBDIVISIONS);

	Grid3D<bool> seen_space(grid_coords.size(), false);

	VtkVoxelGrid voxel_grid(grid_coords, seen_space, {0.3, 0.3, 0.3}, true);
	viewer.addActor(voxel_grid.getActor());

	std::vector<Triangle> triangles = extractOccludingTriangles(tree_model);

	Grid3D<std::vector<Vec3d>> apples_in_cells(grid_coords.size(), {});

	for (const auto &apple: fruit_positions) {
		Vec3i coords = grid_coords.getGridCoordinates(apple).value();
		apples_in_cells[coords].push_back(apple);
	}

	{
		const Grid3D<bool> &occluded_space = voxel_visibility::cast_occlusion(grid_coords,
																			  triangles,
																			  computeEndEffectorPosition(*robot,
																										 current_state));

		std::vector<Vec3d> newly_detected_fruits = integrate_seen_space(apples_in_cells, seen_space, occluded_space);

		voxel_grid.update(seen_space);

		const experiments::VoxelShroudedSceneInfoUpdate &state = experiments::VoxelShroudedSceneInfoUpdate::make_filtered(
				tree_model,
				grid_coords,
				seen_space,
				occluded_space,
				current_state,
				newly_detected_fruits);

		next_state = algorithm->nextMovement(state);
	}

	VtkEndAndBaseTraceVisualization trace_visualization(robot);

	updateTrace(robot, current_state, algorithm, trace_visualization);

	viewer.addActor(trace_visualization.barTrace.getActor());
	viewer.addActor(trace_visualization.endTrace.getActor());
	viewer.addActor(trace_visualization.baseTrace.getActor());

	viewer.addTimerCallback([&]() {
		if (next_state) {

			const Grid3D<bool> &occluded_space = voxel_visibility::cast_occlusion(grid_coords,
																				  triangles,
																				  computeEndEffectorPosition(*robot,
																											 current_state));

			std::vector<Vec3d> newly_detected_fruits = integrate_seen_space(apples_in_cells, seen_space, occluded_space);

			voxel_grid.update(seen_space);

			double distance = moveit_joint_distance(*robot, current_state, *next_state);

			if (newly_detected_fruits.empty()) {
				if (distance < STEP_SIZE) {
					total_distance += distance;
					current_state = *next_state;
					next_state = algorithm->nextMovement(experiments::VoxelShroudedSceneInfoUpdate::make_filtered(
							tree_model,
							grid_coords,
							seen_space,
							occluded_space,
							current_state,
							newly_detected_fruits));

					updateTrace(robot, current_state, algorithm, trace_visualization);

				} else {
					total_distance += STEP_SIZE;
					current_state = interpolate(*robot, current_state, *next_state, STEP_SIZE / distance);
				}

				robotModelViz.applyState(current_state);

			} else {
				std::cout << "Newly detected fruits: " << newly_detected_fruits.size() << std::endl;
				next_state = algorithm->nextMovement(experiments::VoxelShroudedSceneInfoUpdate::make_filtered(tree_model,
																											  grid_coords,
																											  seen_space,
																											  occluded_space,
																											  current_state,
																											  newly_detected_fruits));

				updateTrace(robot, current_state, algorithm, trace_visualization);
			}

			if (collision_detection.collides(current_state)) {
				std::cout << "Collision!" << std::endl;
				next_state = std::nullopt;
			}

		}

	});


	//	viewer.startRecording("no_preknowledge.ogv");

	viewer.start();

	return 0;

}
