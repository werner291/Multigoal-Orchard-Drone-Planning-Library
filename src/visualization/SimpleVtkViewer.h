// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.


#ifndef NEW_PLANNERS_SIMPLEVTKVIEWER_H
#define NEW_PLANNERS_SIMPLEVTKVIEWER_H

#include <vtkNew.h>
#include <functional>
#include <optional>
#include <shape_msgs/msg/mesh.hpp>

#include "../math/Vec3.h"

#include "VideoRecorder.h"

// Forward declarations.
class vtkActor;
class vtkActorCollection;
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;

namespace mgodpl {

	namespace tree_meshes {
		struct TreeMeshes;
	}
	struct SimplifiedOrchard;

	class SimpleVtkViewer {

	public:
		// Create the VTK rendering objects.
		vtkNew<vtkRenderer> viewerRenderer;
		vtkNew<vtkRenderWindow> visualizerWindow;
		vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;

		// Optionally, a video recorder to record a video of the simulation.
		std::optional<VideoRecorder> videoRecorder;

		/**
		 * Create a new VTK viewer.
		 */
		SimpleVtkViewer();

		~SimpleVtkViewer();;

		/**
		 * Start recording a video.
		 *
		 * @param filename The filename to save the video to.
		 */
		void startRecording(const std::string &filename);

		/**
		 * Capture a screenshot.
		 *
		 * @param filename The filename to save the screenshot to. (Must end in .png)
		 * @param bool Whether to render the window before capturing the screenshot.
		 */
		void captureScreenshot(const std::string &filename, bool render = true);

		/**
		 * \brief Renders a single frame of the simulation and returns as an image.
		 */
		vtkSmartPointer<vtkImageData> currentImage();

		/**
		 * Stop recording a video and discard the results.
		 */
		void discardVideo();

		/**
		 * Add a vtkActor to the scene.
		 * @param actor 		The actor to add.
		 */
		void addActor(vtkActor *actor);

		/**
		 * Add a collection of vtkActors to the scene.
		 * @param actors 		The actors to add.
		 */
		void addActorCollection(vtkActorCollection *actors);

		/**
		 * Add a mesh to the scene, adsuming no transformations.
		 *
		 * @param mesh 			The mesh to add.
		 * @param color 		The color of the mesh.
		 */
		vtkSmartPointer<vtkActor> addMesh(const shape_msgs::msg::Mesh &mesh,
										  const math::Vec3d &color,
										  double opacity = 1.0,
										  const math::Vec3d &position = math::Vec3d::Zero());

		/**
		 * Add a callback that is called every time the simulation is rendered.
		 * @param callback 		The callback to add.
		 */
		void addTimerCallback(std::function<void()> callback);

		/**
		 * Start rendering the scene; won't return until the user closes the window,
		 * or stop() is called.
		 */
		void start();

		/**
		 * Stop rendering the scene. This will cause start() to return,
		 * and if a video is being recorded, it will be saved to the file.
		 */
		void stop();

		void addStaticPolyline(const std::vector<math::Vec3d> &points, const math::Vec3d &color);

		void lockCameraUp();

		void setCameraTransform(const math::Vec3d &position, const math::Vec3d &lookAt);
	};

}

#endif //NEW_PLANNERS_SIMPLEVTKVIEWER_H
