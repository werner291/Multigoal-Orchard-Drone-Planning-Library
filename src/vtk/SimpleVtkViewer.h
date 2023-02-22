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

class SimpleVtkViewer {

	// Create the VTK rendering objects.
	vtkNew<vtkRenderer> viewerRenderer;
	vtkNew<vtkRenderWindow> visualizerWindow;
	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;

public:

	SimpleVtkViewer();

	void addActor(vtkActor *actor);

	void addActorCollection(vtkActorCollection *actors);

	void addTimerCallback(std::function<void()> callback);

	void start();

};

#endif //NEW_PLANNERS_SIMPLEVTKVIEWER_H
