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
#include "../utilities/alpha_shape.h"
#include "../utilities/MeshOcclusionModel.h"


void createActors(const TreeMeshes &meshes,
				  const std::vector<AppleDiscoverabilityType> &apple_discoverability,
				  std::vector<vtkActor *> &apple_actors,
				  SimpleVtkViewer &viewer) {
	vtkNew<vtkActorCollection> actors;

	auto tree_actor = createActorFromMesh(meshes.trunk_mesh);
	setColorsByEncoding(tree_actor, TRUNK_RGB, false);
	actors->AddItem(tree_actor);

	auto leaves_actor = createActorFromMesh(meshes.leaves_mesh);
	setColorsByEncoding(leaves_actor, LEAVES_RGB, false);
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

	auto traj = eval.computeNextTrajectory();

	std::vector<vtkActor *> apple_actors;

	// Create a VtkRobotmodel object to visualize the robot itself.
	VtkRobotmodel robotModel(robot, start_state);

	SimpleVtkViewer viewer;

	// Add the robot model to the viewer.
	viewer.addActorCollection(robotModel.getLinkActors());

	// Add the tree meshes to the viewer.
	createActors(meshes, apple_discoverability, apple_actors, viewer);

	auto alphashape = alphaShape(meshes.leaves_mesh.vertices | ranges::views::transform([](const auto &v) {
		return Eigen::Vector3d{v.x, v.y, v.z};
	}) | ranges::to_vector, sqrt(0.0001));

	MeshOcclusionModel occlusion_model(alphashape);

	{
		auto alphashape_actor = createActorFromMesh(alphashape);

		alphashape_actor->GetProperty()->SetOpacity(0.5);

		viewer.addActor(alphashape_actor);
	}

	VtkLineSegmentsVisualization occlusion_visualization(1.0,0.0,1.0);

	viewer.addActor(occlusion_visualization.getActor());

	double time = 0.0;

	// The "main loop" of the program, called every frame.
	auto callback = [&]() {

		time += 0.01;

		if (eval.getUpcomingGoalEvent() && (traj.has_value() && time > traj->getDuration())) {
			time = 0.0;

			traj = eval.computeNextTrajectory();
		}

		if (traj) {
			// Update the robot's visualization to match the current state.
			moveit::core::RobotState state(robot);

			setStateToTrajectoryPoint(state, time, *traj);

			robotModel.applyState(state);

			for (const auto &[apple_actor, status]: ranges::views::zip(apple_actors, eval.getDiscoveryStatus())) {
				switch (status) {
					case utilities::DiscoveryStatus::VISITED:
						apple_actor->GetProperty()->SetDiffuseColor(1.0, 0.0, 0.0);
						break;
					case utilities::DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT:
						apple_actor->GetProperty()->SetDiffuseColor(1.0, 0.0, 1.0);
						break;
					case utilities::DiscoveryStatus::KNOWN_TO_ROBOT:
						apple_actor->GetProperty()->SetDiffuseColor(1.0, 0.0, 0.5);
						break;
				}
			}

			occlusion_visualization.updateLine(scene.apples | ranges::views::transform([&](const auto &apple) -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
				return {state.getGlobalLinkTransform("end_effector").translation(), apple.center};
			}) | ranges::views::filter([&](const auto &line) {
				return !occlusion_model.checkOcclusion(line.first, line.second);
			}) | ranges::to_vector);
		}

	};

	viewer.addTimerCallback(callback);

	viewer.start();

	return EXIT_SUCCESS;
}
