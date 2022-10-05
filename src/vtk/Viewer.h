
#ifndef NEW_PLANNERS_VIEWER_H
#define NEW_PLANNERS_VIEWER_H

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <functional>

class Viewer {

public:
	vtkNew<vtkRenderer> viewerRenderer;
	vtkNew<vtkRenderWindow> visualizerWindow;
	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;

	Viewer();

	void addActor(vtkActor *actor);

	void addActorCollection(vtkActorCollection *actors);

	void requestRender();

	void start();

	void setIntervalCallback(const std::function<void()>& callback);
};


#endif //NEW_PLANNERS_VIEWER_H
