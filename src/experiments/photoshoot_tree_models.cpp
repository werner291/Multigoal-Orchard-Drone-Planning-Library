// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <filesystem>
#include <iostream>
#include <vector>
#include <range/v3/view/enumerate.hpp>
#include "../TreeMeshes.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../utilities/vtk.h"
#include "../utilities/experiment_utils.h"
#include "../utilities/mesh_utils.h"
#include "../visualization/VtkRobotModel.h"

static const double SPACING = 1.0;

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

	// Load tree meshes, add them to viewer one at a time, snap a picture, and remove them
	for (const auto& [index, modelName] : ranges::views::enumerate(modelNames)) {

		SimpleVtkViewer viewer;

		viewer.viewerRenderer->GetActiveCamera()->ParallelProjectionOn();
		viewer.viewerRenderer->GetActiveCamera()->SetParallelScale(5.0);
		viewer.visualizerWindow->SetAlphaBitPlanes(1); // Enable usage of alpha channel

		// Set up a side view
		viewer.viewerRenderer->GetActiveCamera()->SetPosition(0.0, 10.0, 4.0);
		viewer.viewerRenderer->GetActiveCamera()->SetFocalPoint(0.0, 0.0, 3.9);
		viewer.viewerRenderer->GetActiveCamera()->SetViewUp(0.0, 0.0, 1.0);

		viewer.viewerRenderer->SetBackground(0.0, 0.0, 0.0);
		viewer.viewerRenderer->SetBackgroundAlpha(0); // Set the alpha value to 0 for transparency

		std::cout << modelName << std::endl;

		TreeMeshes treeMeshes = loadTreeMeshes(modelName);

		viewer.addMesh(treeMeshes.trunk_mesh, {0.5, 0.3, 0.1}, 1.0, {0.0, 0.0, 0.0});
		viewer.addMesh(treeMeshes.leaves_mesh, {0.0, 0.5, 0.0}, 1.0, {0.0, 0.0, 0.0});

		for (const auto& fruit : treeMeshes.fruit_meshes) {
			viewer.addMesh(fruit, {1.0, 0.0, 0.0}, 1.0, {0.0, 0.0, 0.0});
		}

		VtkRobotmodel robotModel(robot, robotState);
//		viewer.addActorCollection(robotModel.getLinkActors());

		viewer.captureScreenshot("tree_pics/" + modelName + ".png");

	}

}

