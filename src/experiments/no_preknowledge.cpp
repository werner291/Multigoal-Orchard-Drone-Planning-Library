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

static const double STEP_SIZE = 0.1;
using namespace mgodpl;
using namespace visualization;
using namespace experiment_state_tools;
using namespace moveit_facade;
using namespace planning;

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

	viewer.addTimerCallback([&]() {
		if (next_state) {

			double distance = moveit_joint_distance(*robot, current_state, *next_state);

			if (distance < STEP_SIZE) {
				total_distance += distance;
				current_state = *next_state;
				next_state = algorithm->nextMovement({current_state, {}});
			} else {
				total_distance += STEP_SIZE;
				current_state = interpolate(*robot, current_state, *next_state, STEP_SIZE / distance);
			}

			robotModelViz.applyState(current_state);

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