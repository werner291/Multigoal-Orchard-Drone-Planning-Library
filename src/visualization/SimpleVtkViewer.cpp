// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <vtkProperty.h>
#include <vtkPNGWriter.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <boost/algorithm/string/predicate.hpp>

#include "SimpleVtkViewer.h"
#include "VtkFunctionalCallback.h"
#include "camera_controls.h"
#include "vtk.h"
#include "../experiment_utils/TreeMeshes.h"

namespace mgodpl {

	using namespace math;

	SimpleVtkViewer::SimpleVtkViewer() {

		// Set up the render window.
		visualizerWindow->SetSize(800, 600);
		visualizerWindow->SetWindowName("Robot Path Planning");
		visualizerWindow->AddRenderer(viewerRenderer);

		// Set up the render window interactor.
		renderWindowInteractor->SetRenderWindow(visualizerWindow);
		renderWindowInteractor->CreateRepeatingTimer(33);

		viewerRenderer->SetBackground(0.5, 0.8, 0.9);

		addTimerCallback([&]() {
			renderWindowInteractor->GetRenderWindow()->Render();

			if (videoRecorder.has_value()) {
				videoRecorder->exportFrame();
			}
		});

	}

	void SimpleVtkViewer::lockCameraUp() {
		enforceCameraUp(viewerRenderer, renderWindowInteractor);
	}

	void SimpleVtkViewer::setCameraTransform(const math::Vec3d& position, const math::Vec3d& lookAt)
	{
		viewerRenderer->GetActiveCamera()->SetPosition(position.data());
		viewerRenderer->GetActiveCamera()->SetFocalPoint(lookAt.data());
		viewerRenderer->GetActiveCamera()->SetViewUp(0, 0, 1);
	}

	void SimpleVtkViewer::addActor(vtkActor *actor) {
		viewerRenderer->AddActor(actor);
	}

	void SimpleVtkViewer::addTimerCallback(std::function<void()> callback) {
		// Set the "main loop" callback to be called every frame.
		vtkNew<vtkFunctionalCallback> cb;
		cb->setEventId(vtkCommand::TimerEvent);
		cb->setCallback(callback);
		renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
	}

	void SimpleVtkViewer::start() {
		renderWindowInteractor->Start();
	}

	void SimpleVtkViewer::addActorCollection(vtkActorCollection *actors) {
		for (int i = 0; i < actors->GetNumberOfItems(); i++) {
			viewerRenderer->AddActor(vtkActor::SafeDownCast(actors->GetItemAsObject(i)));
		}
	}

	void SimpleVtkViewer::startRecording(const std::string &filename) {
		videoRecorder.emplace(visualizerWindow, filename);
	}

	void SimpleVtkViewer::stop() {

		renderWindowInteractor->TerminateApp();

		if (videoRecorder.has_value()) {
			videoRecorder->finish();
		}
	}

	void SimpleVtkViewer::discardVideo() {
		videoRecorder.reset();
	}

	vtkSmartPointer<vtkActor> SimpleVtkViewer::addMesh(const shape_msgs::msg::Mesh &mesh,
													   const Vec3d &color,
													   double opacity,
													   const Vec3d &position) {

		Vec3d origin = position;

		auto actor = createActorFromMesh(mesh);

		actor->GetProperty()->SetColor(color[0], color[1], color[2]);

		if (opacity < 1.0) {
			actor->GetProperty()->SetOpacity(opacity);
		}

		actor->SetPosition(origin[0], origin[1], origin[2]);

		addActor(actor);

		return actor;

	}

	void SimpleVtkViewer::captureScreenshot(const std::string &filename, bool render) {

		assert(boost::algorithm::ends_with(filename, ".png"));

		if (render) {
			renderWindowInteractor->GetRenderWindow()->Render();
		}

		vtkNew<vtkWindowToImageFilter> windowToImageFilter;
		windowToImageFilter->SetInput(visualizerWindow);
		windowToImageFilter->SetInputBufferTypeToRGBA(); // Also record the alpha (transparency) channel
		windowToImageFilter->ReadFrontBufferOff();
		windowToImageFilter->Update();

		vtkNew<vtkPNGWriter> writer;
		writer->SetFileName(filename.c_str());
		writer->SetInputConnection(windowToImageFilter->GetOutputPort());
		writer->Write();

	}

	vtkSmartPointer<vtkImageData> SimpleVtkViewer::currentImage()
	{

		std::cout << "rendering" << std::endl;

		visualizerWindow->Render();

		vtkNew<vtkWindowToImageFilter> windowToImageFilter;
		windowToImageFilter->SetInput(visualizerWindow);
		windowToImageFilter->SetInputBufferTypeToRGBA(); // Also record the alpha (transparency) channel
		windowToImageFilter->ReadFrontBufferOff();
		windowToImageFilter->Update();
		//
		// return windowToImageFilter->GetOutput();

		return nullptr;
	}


	//	void SimpleVtkViewer::addStaticPolyline(const std::vector<Vec3d> &points, const Vec3d &color) {
//		VtkPolyLineVisualization ee_trace_viz(color.x(), color.y(), color.z());
//		ee_trace_viz.updateLine(points);
//		addActor(ee_trace_viz.getActor());
//	}
//
//	void SimpleVtkViewer::addStaticLines(const std::vector<std::pair<Vec3d, Vec3d>> &lines,
//										 const Vec3d &color) {
//		VtkLineSegmentsVisualization path_viz(color.x(), color.y(), color.z());
//		path_viz.updateLine(lines);
//		addActor(path_viz.getActor());
//	}
//
//
//	std::vector<std::pair<Vec3d, Vec3d>>
//	zipTraces(const std::vector<Vec3d> &ee_trace, const std::vector<Vec3d> &base_trace) {
//		std::vector<std::pair<Vec3d, Vec3d>> path_segments;
//		for (size_t i = 0; i < ee_trace.size(); ++i) {
//			path_segments.emplace_back(ee_trace[i], base_trace[i]);
//		}
//		return path_segments;
//	}

	//void visualizeBaseEndEffectorLadderTrace(SimpleVtkViewer &viewer, const RobotPath &rpath_moveit) {
	//	auto ee_trace = computeLinkTrace(rpath_moveit, "end_effector");
	//	auto base_trace = computeLinkTrace(rpath_moveit, "base_link");
	//	viewer.addStaticPolyline(ee_trace, {1.0, 0.0, 0.0});
	//	viewer.addStaticPolyline(base_trace, {0.0, 1.0, 0.0});
	//
	//	viewer.addStaticLines(zipTraces(ee_trace, base_trace), {1.0, 1.0, 0.0});
	//}

	void addTreeMeshesToViewer(SimpleVtkViewer &viewer, const tree_meshes::TreeMeshes &current_tree_models) {
		viewer.addActor(createColoredMeshActor(current_tree_models.trunk_mesh, {0.5, 0.3, 0.1, 1.0}, true));
		viewer.addActor(createColoredMeshActor(current_tree_models.leaves_mesh, {0.1, 0.5, 0.1, 1.0}, true));
		for (const auto &mesh: current_tree_models.fruit_meshes) {
			viewer.addActor(createColoredMeshActor(mesh, {0.9, 0.0, 0.0, 1.0}, true));
		}
	}

	void addSimplifiedOrchardToViewer(SimpleVtkViewer &viewer, const tree_meshes::SimplifiedOrchard &orchard) {

		for (const auto &[pos, tree]: orchard.trees) {

			auto tree_actor = createColoredMeshActor(tree.trunk_mesh, {0.5, 0.3, 0.1, 1.0}, true);
			tree_actor->SetPosition(pos.x(), pos.y(), 0.0);
			viewer.addActor(tree_actor);

			auto leaves_actor = createColoredMeshActor(tree.leaves_mesh, {0.1, 0.5, 0.1, 1.0}, true);
			leaves_actor->SetPosition(pos.x(), pos.y(), 0.0);
			viewer.addActor(leaves_actor);

			for (const auto &fruit: tree.fruit_meshes) {
				auto fruit_actor = createColoredMeshActor(fruit, {0.9, 0.0, 0.0, 1.0}, true);
				fruit_actor->SetPosition(pos.x(), pos.y(), 0.0);
				viewer.addActor(fruit_actor);
			}

		}

	}

	mgodpl::SimpleVtkViewer::~SimpleVtkViewer() {
	}
}
