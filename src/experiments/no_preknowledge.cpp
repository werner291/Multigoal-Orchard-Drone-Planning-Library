// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

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

static const double STEP_SIZE = 0.1;
using namespace mgodpl;
using namespace visualization;
using namespace experiment_state_tools;
using namespace moveit_facade;
using namespace planning;
using namespace math;

Vec3d toVec3d(const geometry_msgs::msg::Point& p) {
	return {p.x, p.y, p.z};
}

int main() {

	// TODO: Reminder, write down the idea of the snake path!

	const auto& robot = experiment_assets::loadRobotModel(1.0);

	const auto& tree_model = tree_meshes::loadTreeMeshes("appletree");

	const auto& fruit_positions = tree_model.fruit_meshes | ranges::views::transform([](const auto& mesh) {
		return mesh_aabb(mesh).center();
	}) | ranges::to<std::vector>();

	SimpleVtkViewer viewer;

	JointSpacePoint current_state = randomStateOutsideTree(*robot, 0);

	visualization::VtkRobotModel robotModelViz(robot, current_state, {0.5, 0.5, 0.5});
	viewer.addActorCollection(robotModelViz.getLinkActors());

	const std::shared_ptr<RobotAlgorithm> algorithm = std::make_shared<BlindlyMoveToNextFruit>(robot);

	std::optional<JointSpacePoint> next_state = algorithm->nextMovement({current_state, fruit_positions});

	double total_distance = 0.0;

	CollisionDetection collision_detection({tree_model.trunk_mesh}, robot);

	const size_t SUBDIVISIONS = 30;

	AABBGrid grid_coords(
			AABBd(Vec3d(-3.0, -3.0, 0.0), Vec3d(3.0, 3.0, 6.0)),
			SUBDIVISIONS,SUBDIVISIONS,SUBDIVISIONS);

	Grid3D<bool> seen_space(grid_coords.size(), false);

	VtkVoxelGrid voxel_grid(grid_coords, seen_space, {0.3,0.3,0.3}, true);
	viewer.addActor(voxel_grid.getActor());

	std::vector<Triangle> triangles;

	// For every leaf in the tree, set the corresponding grid cell to true.
	for (const auto &triangle: tree_model.leaves_mesh.triangles) {

		Vec3d a = toVec3d(tree_model.leaves_mesh.vertices[triangle.vertex_indices[0]]);
		Vec3d b = toVec3d(tree_model.leaves_mesh.vertices[triangle.vertex_indices[1]]);
		Vec3d c = toVec3d(tree_model.leaves_mesh.vertices[triangle.vertex_indices[2]]);

		triangles.emplace_back(a,b,c);
	}

	Grid3D<std::vector<Vec3d>> apples_in_cells(grid_coords.size(), {});

	for (const auto& apple : fruit_positions) {
		Vec3i coords = grid_coords.getGridCoordinates(apple).value();
		apples_in_cells[coords].push_back(apple);
	}

	viewer.addTimerCallback([&]() {
		if (next_state) {

			const Grid3D<bool>& occluded_space = voxel_visibility::cast_occlusion(grid_coords, triangles, computeEndEffectorPosition(*robot, current_state));

			std::vector<Vec3d> newly_detected_fruits;

			for (size_t x = 0; x < SUBDIVISIONS; x++) {
				for (size_t y = 0; y < SUBDIVISIONS; y++) {
					for (size_t z = 0; z < SUBDIVISIONS; z++) {

						Vec3i coords = {(int)x, (int)y, (int)z};

						if (seen_space[coords] && !occluded_space[coords]) {
							seen_space[coords] = true;
							for (const auto& apple : apples_in_cells[coords]) {
								newly_detected_fruits.push_back(apple);
							}
						}

					}
				}
			}

			double distance = moveit_joint_distance(*robot, current_state, *next_state);

			if (newly_detected_fruits.empty()) {
				if (distance < STEP_SIZE) {
					total_distance += distance;
					current_state = *next_state;
					next_state = algorithm->nextMovement({current_state, {}});
				} else {
					total_distance += STEP_SIZE;
					current_state = interpolate(*robot, current_state, *next_state, STEP_SIZE / distance);
				}

				robotModelViz.applyState(current_state);
			} else {
				std::cout << "Newly detected fruits: " << newly_detected_fruits.size() << std::endl;
				next_state = algorithm->nextMovement({current_state, newly_detected_fruits});
			}

			if (collision_detection.collides(current_state)) {
				std::cout << "Collision!" << std::endl;
				next_state = std::nullopt;
			}

			std::cout << "Total distance:  " << total_distance << std::endl;

		}

	});

	viewer.addMesh(tree_model.trunk_mesh, {0.5, 0.3, 0.1});
	viewer.addMesh(tree_model.leaves_mesh, {0.1, 0.5, 0.1});
	for (const auto& fruit : tree_model.fruit_meshes) viewer.addMesh(fruit, {0.5, 0.1, 0.1});
	viewer.addMesh(createGroundPlane(5.0,5.0), {0.3, 0.5, 0.1});

	viewer.start();

	return 0;

}