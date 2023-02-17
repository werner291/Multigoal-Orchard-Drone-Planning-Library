//
// Created by werner on 16-2-23.
//

#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkActorCollection.h>
#include <range/v3/view/enumerate.hpp>

#include "visualize_dynamic.h"
#include "VtkRobotModel.h"
#include "SimpleVtkViewer.h"

#include "../utilities/moveit.h"
#include "../utilities/vtk.h"

#include "../exploration/ColorEncoding.h"


void createActors(const TreeMeshes &meshes,
				  const std::vector<AppleDiscoverabilityType> &apple_discoverability,
				  std::vector<vtkActor *> &apple_actors,
				  SimpleVtkViewer &viewer) {
	vtkNew<vtkActorCollection> actors;

	auto tree_actor = createActorFromMesh(meshes.trunk_mesh);
	setColorsByEncoding(tree_actor, TRUNK_RGB, false);

	auto leaves_actor = createActorFromMesh(meshes.leaves_mesh);
	setColorsByEncoding(leaves_actor, LEAVES_RGB, false);

	actors->AddItem(tree_actor);
	actors->AddItem(leaves_actor);

	for (const auto &[fruit_index, fruit_mesh]: meshes.fruit_meshes | ranges::views::enumerate) {
		auto fruit_actor = createActorFromMesh(fruit_mesh);

		std::array<double, 3> rgb{0.0, 0.0, 0.0};

		if (apple_discoverability[fruit_index]) {
			rgb = {1.0, 0.0, 0.0};
		} else {
			rgb = {0.0, 0.0, 1.0};
		}

		fruit_actor->GetProperty()->SetDiffuseColor(rgb.data());

		actors->AddItem(fruit_actor);

		apple_actors.push_back(fruit_actor);
	}

	viewer.addActorCollection(actors);
}


int visualizeEvaluation(const TreeMeshes &meshes,
						const AppleTreePlanningScene &scene,
						const moveit::core::RobotModelPtr &robot,
						const moveit::core::RobotState &start_state,
						const std::vector<AppleDiscoverabilityType> &apple_discoverability,
						DynamicGoalVisitationEvaluation &eval) {
	robot_trajectory::RobotTrajectory traj = *eval.computeNextTrajectory();

	std::vector<vtkActor *> apple_actors;

	// Create a VtkRobotmodel object to visualize the robot itself.
	VtkRobotmodel robotModel(robot, start_state);

	SimpleVtkViewer viewer;

	// Add the robot model to the viewer.
	viewer.addActorCollection(robotModel.getLinkActors());

	// Add the tree meshes to the viewer.
	createActors(meshes, apple_discoverability, apple_actors, viewer);

	double time = 0.0;

	// The "main loop" of the program, called every frame.
	auto callback = [&]() {

		time += 0.01;

		if (eval.getUpcomingGoalEvent() && time > traj.getDuration()) {
			time = 0.0;
			traj = *eval.computeNextTrajectory();
		}

		// Update the robot's visualization to match the current state.
		moveit::core::RobotState state(robot);

		setStateToTrajectoryPoint(state, time, traj);

		robotModel.applyState(state);

		for (const auto &[apple_actor, apple]: ranges::views::zip(apple_actors, scene.apples)) {
			if ((state.getGlobalLinkTransform("end_effector").translation() - apple.center).norm() < 0.05) {
				apple_actor->GetProperty()->SetDiffuseColor(0.0, 1.0, 0.0);
			}
		}

	};

	viewer.addTimerCallback(callback);

	viewer.start();

	return EXIT_SUCCESS;
}
