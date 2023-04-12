//
// Created by werner on 16-2-23.
//

#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkTextProperty.h>
#include <utility>
#include <vtkImageMapper.h>
#include <vtkImageViewer2.h>
#include <vtkPNGReader.h>

#include "visualize_dynamic.h"
#include "VtkRobotModel.h"
#include "SimpleVtkViewer.h"

#include "../utilities/moveit.h"
#include "../utilities/vtk.h"

#include "../exploration/ColorEncoding.h"
#include "../utilities/MeshOcclusionModel.h"
#include "../RunPlannerThreaded.h"
#include "../apple_status_color_coding.h"
#include "orchard_actors.h"
#include "StatusTextViz.h"
#include "../RobotCameraTracker.h"
#include "../utilities/trajectoryTiming.h"
#include <vtkProperty2D.h>

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
getOcclusionLineCoordinates(const moveit::core::RobotState &state,
							const std::vector<Apple> &apples,
							const utilities::CanSeeAppleFn &can_see_apple) {

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> occlusion_lines;

	for (const auto &apple: apples) {
		if (can_see_apple(state, apple)) {
			occlusion_lines.emplace_back(state.getGlobalLinkTransform("end_effector").translation(), apple.center);
		}
	}

	return occlusion_lines;
}

std::string mkStatusString(const std::vector<RunPlannerThreaded::PlannerUpdate> &trajectories,
						   size_t traj_index,
						   double traj_time,
						   const RunPlannerThreaded &eval_thread,
						   const utilities::DiscoveryStatusStats &stats) {
	std::stringstream ss;
	ss.precision(1);

	ss << "Visited " << stats.visited << " (" << stats.discoverable << " undiscovered, " << stats.false_positives
	   << " false apples, " << stats.known_unvisited << " known unvisited)" << std::endl;

	ss << "Time: " << traj_time << "s/" << trajectories[traj_index].traj.getDuration() << "s, Segment " << traj_index
	   << "/" << trajectories.size();

	if (eval_thread.n_segments_requested() > 0) {
		ss << " (computing more)";
	}

	std::string status = ss.str();
	return status;
}

void addAppleColorLegend(SimpleVtkViewer &viewer) {

	vtkNew<vtkPNGReader> reader;
	reader->SetFileName("assets/apple_colors.png");
	reader->Update();

	// Display as a flat 2D image in the top-left corner of the window.

	vtkNew<vtkImageMapper> mapper;
	mapper->SetInputConnection(reader->GetOutputPort());
	mapper->SetColorWindow(255);
	mapper->SetColorLevel(127.5);

	vtkNew<vtkActor2D> actor;
	actor->SetMapper(mapper);
	actor->SetPosition(0, 20);

	viewer.viewerRenderer->AddActor2D(actor);

}


int visualizeEvaluation(const TreeMeshes &meshes,
						const AppleTreePlanningScene &scene,
						const moveit::core::RobotModelPtr &robot,
						const moveit::core::RobotState &start_state,
						const std::vector<AppleDiscoverabilityType> &apple_discoverability,
						DynamicGoalVisitationEvaluation &eval) {

	std::vector<RunPlannerThreaded::PlannerUpdate> trajectories;
	size_t traj_index = 0;
	double traj_time = 0.0;

	// Create a VtkRobotmodel object to visualize the robot itself.
	VtkRobotmodel robotModel(robot, start_state);

	SimpleVtkViewer viewer;

#ifdef RECORD_VIDEO
	//	viewer.startRecording("video.ogv");
#endif

	// Add the robot model to the viewer.
	viewer.addActorCollection(robotModel.getLinkActors());

	// Create the orchard actors, automatically adding them to the viewer and returning 
	// a vector of pointers to the actors representing the apples.
	auto apple_actors = createActors(meshes, viewer);

	VtkLineSegmentsVisualization occlusion_visualization(1.0, 0.0, 1.0);
	viewer.addActor(occlusion_visualization.getActor());

	StatusTextViz status_text(viewer.viewerRenderer);

	std::vector<utilities::DiscoveryStatus> initial_status(eval.getDiscoveryStatus());

	RunPlannerThreaded eval_thread(eval, true);

	addAppleColorLegend(viewer);

	vtkNew<vtkCamera> camera;
	viewer.viewerRenderer->SetActiveCamera(camera);

	RobotCameraTracker camera_tracker(camera, start_state);

	auto start_time = std::chrono::high_resolution_clock::now();

	// The "main loop" of the program, called every frame.
	auto callback = [&]() {

		if (auto new_traj = eval_thread.poll_trajectory_update()) {
			adjustTiming(new_traj->traj);
			trajectories.push_back(*new_traj);
		}

		while (traj_index < trajectories.size() && traj_time > trajectories[traj_index].traj.getDuration()) {
			traj_time = 0.0;
			traj_index++;
		}

		// We pick the "previous" status, because the current trajectory is not yet finished,
		// and we want to show the status before the end of the current trajectory.
		auto visit_status = traj_index == 0 ? initial_status : trajectories[traj_index - 1].status;

		if (traj_index < trajectories.size()) {

			auto &current_trajectory = trajectories[traj_index];

			traj_time += 0.05;

			// Update the robot's visualization to match the current state.
			moveit::core::RobotState state(robot);
			setStateToTrajectoryPoint(state, traj_time, current_trajectory.traj);

			robotModel.applyState(state);
			camera_tracker.update(state);
			updateAppleColors(visit_status, apple_actors);

			occlusion_visualization.updateLine(getOcclusionLineCoordinates(state, scene.apples, eval.getCanSeeApple()));

		}


		status_text.updateText(mkStatusString(trajectories,
											  traj_index,
											  traj_time,
											  eval_thread,
											  getDiscoveryStatusStats(visit_status)));

#ifdef RECORD_VIDEO
		auto now = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(now - start_time).count();

		if (elapsed > 2) {
			viewer.stop();
		}
#endif

	};

	viewer.addTimerCallback(callback);

	eval_thread.start();

	viewer.start();

	eval_thread.finish();

	return EXIT_SUCCESS;
}
