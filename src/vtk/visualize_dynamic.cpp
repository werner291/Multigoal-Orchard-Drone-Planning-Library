//
// Created by werner on 16-2-23.
//

#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkActorCollection.h>
#include <range/v3/view/enumerate.hpp>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <range/v3/algorithm/count.hpp>
#include <utility>

#include "visualize_dynamic.h"
#include "VtkRobotModel.h"
#include "SimpleVtkViewer.h"

#include "../utilities/moveit.h"
#include "../utilities/vtk.h"

#include "../exploration/ColorEncoding.h"
#include "../utilities/MeshOcclusionModel.h"
#include "../RunPlannerThreaded.h"

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

	{

		shape_msgs::msg::Mesh ground_plane;
		ground_plane.vertices.resize(4);
		ground_plane.vertices[0].x = -10.0;
		ground_plane.vertices[0].y = -10.0;
		ground_plane.vertices[0].z = 0.0;

		ground_plane.vertices[1].x = 10.0;
		ground_plane.vertices[1].y = -10.0;
		ground_plane.vertices[1].z = 0.0;

		ground_plane.vertices[2].x = 10.0;
		ground_plane.vertices[2].y = 10.0;
		ground_plane.vertices[2].z = 0.0;

		ground_plane.vertices[3].x = -10.0;
		ground_plane.vertices[3].y = 1.0;
		ground_plane.vertices[3].z = 0.0;

		ground_plane.triangles.resize(2);
		ground_plane.triangles[0].vertex_indices[0] = 0;
		ground_plane.triangles[0].vertex_indices[1] = 1;
		ground_plane.triangles[0].vertex_indices[2] = 2;

		ground_plane.triangles[1].vertex_indices[0] = 0;
		ground_plane.triangles[1].vertex_indices[1] = 2;
		ground_plane.triangles[1].vertex_indices[2] = 3;

		// Ground plane.
		auto ground_actor = createActorFromMesh(ground_plane);

		ground_actor->GetProperty()->SetDiffuseColor(0.5, 0.8, 0.5);

		actors->AddItem(ground_actor);

	}

	viewer.addActorCollection(actors);
}


int visualizeEvaluation(const TreeMeshes &meshes,
						const AppleTreePlanningScene &scene,
						const moveit::core::RobotModelPtr &robot,
						const moveit::core::RobotState &start_state,
						const std::vector<AppleDiscoverabilityType> &apple_discoverability,
						DynamicGoalVisitationEvaluation &eval) {

	std::vector<robot_trajectory::RobotTrajectory> trajectories;
	size_t traj_index = 0;
	double traj_time = 0.0;

	std::vector<vtkActor *> apple_actors;

	// Create a VtkRobotmodel object to visualize the robot itself.
	VtkRobotmodel robotModel(robot, start_state);

	SimpleVtkViewer viewer;

	// Add the robot model to the viewer.
	viewer.addActorCollection(robotModel.getLinkActors());

	// Add the tree meshes to the viewer.
	createActors(meshes, apple_discoverability, apple_actors, viewer);

	VtkLineSegmentsVisualization occlusion_visualization(1.0, 0.0, 1.0);

	viewer.addActor(occlusion_visualization.getActor());

	vtkNew<vtkTextActor> textActor;
	{ // Setup the text and add it to the renderer
		textActor->SetInput("Hello world");
		textActor->SetPosition2(10, 40);
		textActor->GetTextProperty()->SetFontSize(10);
		//		textActor->GetTextProperty()->SetColor(colors->GetColor3d("Gold").GetData());
		viewer.viewerRenderer->AddActor2D(textActor);
	}

	RunPlannerThreaded eval_thread(eval, true);

	vtkNew<vtkCamera> camera;
	viewer.viewerRenderer->SetActiveCamera(camera);

	// The "main loop" of the program, called every frame.
	auto callback = [&]() {

		if (auto new_traj = eval_thread.poll_trajectory()) {
			trajectories.push_back(*new_traj);
		}

		while (traj_index < trajectories.size() && traj_time > trajectories[traj_index].getDuration()) {
			traj_time = 0.0;
			traj_index++;
		}

		Eigen::Vector3d robot_position = start_state.getGlobalLinkTransform("base_link").translation();

		if (traj_index < trajectories.size()) {

			auto &current_trajectory = trajectories[traj_index];

			traj_time += 0.05;

			// Update the robot's visualization to match the current state.
			moveit::core::RobotState state(robot);

			setStateToTrajectoryPoint(state, traj_time, current_trajectory);

			robotModel.applyState(state);

			{

				robot_position = robot_position * 0.2 + 0.8 * state.getGlobalLinkTransform("base_link").translation();

				Eigen::Vector3d tree_center(0.0, 0.0, 2.0);

				Eigen::Vector3d relative = robot_position - tree_center;

				relative = Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()) * relative;

				Eigen::Vector3d cam_pos = tree_center + relative.normalized() * 8.0;
				cam_pos.z() = 4.0;

				std::cout << "Cam pos: " << cam_pos.transpose() << std::endl;

				camera->SetViewUp(0, 0, 1);
				camera->SetFocalPoint(robot_position.x(), robot_position.y(), robot_position.z());
				camera->SetPosition(cam_pos.x(), cam_pos.y(), cam_pos.z());

			}

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
					default:
						apple_actor->GetProperty()->SetDiffuseColor(0.0, 0.0, 1.0);
						break;
				}
			}

			{
				std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> occlusion_lines;

				for (const auto &apple: scene.apples) {
					if (eval.getCanSeeApple()(state, apple)) {
						occlusion_lines.emplace_back(state.getGlobalLinkTransform("end_effector").translation(),
													 apple.center);
					}
				}

				occlusion_visualization.updateLine(occlusion_lines);
			}

		}

		{
			std::stringstream ss;

			size_t n_total = eval.getDiscoveryStatus().size();
			size_t n_visited = ranges::count(eval.getDiscoveryStatus(), utilities::DiscoveryStatus::VISITED);
			size_t n_discoverable = ranges::count(eval.getDiscoveryStatus(),
												  utilities::DiscoveryStatus::EXISTS_BUT_UNKNOWN_TO_ROBOT);
			size_t n_false = ranges::count(eval.getDiscoveryStatus(),
										   utilities::DiscoveryStatus::ROBOT_THINKS_EXISTS_BUT_DOESNT);

			ss << "Vis: " << n_visited << "/" << n_total << " (" << n_discoverable << " disc, " << n_false << " false)"
			   << std::endl;

			ss.precision(1);
			ss << "T = (" << traj_time << "s/" << trajectories[traj_index].getDuration() << "s, " << traj_index << "/"
			   << trajectories.size() << ")";

			if (eval_thread.n_segments_requested() > 0) {
				ss << " (computing more)";
			}

			textActor->SetInput(ss.str().c_str());
			textActor->Modified();
		}

	};

	viewer.addTimerCallback(callback);

	eval_thread.start();

	viewer.start();

	return EXIT_SUCCESS;
}
