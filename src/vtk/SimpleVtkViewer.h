// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.


#ifndef NEW_PLANNERS_SIMPLEVTKVIEWER_H
#define NEW_PLANNERS_SIMPLEVTKVIEWER_H

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNew.h>
#include <functional>
#include <optional>
#include "VideoRecorder.h"

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

	/**
	 * Start recording a video.
	 *
	 * @param filename The filename to save the video to.
	 */
	void startRecording(const std::string &filename);

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

};

#endif //NEW_PLANNERS_SIMPLEVTKVIEWER_H
