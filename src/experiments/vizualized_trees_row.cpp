// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <moveit/robot_state/robot_state.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/take.hpp>

#include "../TreeMeshes.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../utilities/vtk.h"
#include "../utilities/experiment_utils.h"
#include "../visualization/VtkRobotModel.h"
#include "../orchards.h"

int main() {

	const auto modelNames = getTreeModelNames();

	auto robot = loadRobotModel();

	moveit::core::RobotState robotState(robot);

	robotState.setToDefaultValues();

	Eigen::Isometry3d world_joint_pose;

	world_joint_pose.setIdentity();

	world_joint_pose.translate(Eigen::Vector3d(3.0, 0.0, 1.0));
	world_joint_pose.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

	robotState.setJointPositions("world_joint", world_joint_pose);

	robotState.update(true);

	SimpleVtkViewer viewer;

	viewer.viewerRenderer->GetActiveCamera()->ParallelProjectionOn();
	viewer.viewerRenderer->GetActiveCamera()->SetParallelScale(5.0);
	viewer.visualizerWindow->SetAlphaBitPlanes(1); // Enable usage of alpha channel

	// Set up a side view
	viewer.viewerRenderer->GetActiveCamera()->SetPosition(0.0, 10.0, 3.0);
	viewer.viewerRenderer->GetActiveCamera()->SetFocalPoint(0.0, 0.0, 2.0);
	viewer.viewerRenderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);

	viewer.viewerRenderer->SetBackground(0.5, 0.8, 0.9);

	const auto tree_names = getTreeNames();

	const auto treeModels =
			tree_names |
			ranges::views::transform(loadTreeMeshes) |
			ranges::views::filter([](const auto& treeMeshes) {
		return treeMeshes.fruit_meshes.size() <= 200;
	}) | ranges::to_vector;

	const std::vector<TreeAtPosition> trees = createOrchardRow(treeModels, SPACING, 5);

	TreeMeshes treeMeshes = mergeTreeMeshes(trees);

	viewer.addMesh(treeMeshes.trunk_mesh, {0.5, 0.3, 0.1}, 1.0);
	viewer.addMesh(treeMeshes.leaves_mesh, {0.0, 0.5, 0.0}, 1.0);

	for (const auto& fruit : treeMeshes.fruit_meshes) {
		viewer.addMesh(fruit, {1.0, 0.0, 0.0}, 1.0);
	}

	VtkRobotmodel robotModel(robot, robotState);
	viewer.addActorCollection(robotModel.getLinkActors());

	viewer.start();


}

