
#ifndef NEW_PLANNERS_VIEWER_H
#define NEW_PLANNERS_VIEWER_H

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <functional>
#include "VisualizationSpecifics.h"
#include "../utilities/vtk.h"

class Viewer {

public:
	vtkNew<vtkRenderer> viewerRenderer;
	vtkNew<vtkRenderWindow> visualizerWindow;
	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;

	// Visualization of the convex hull as computed by the motion-planning algorithm.
	ConvexHullActor convexHullActor;

	// Visualization of the target points, as line segments between their actual position, and their projected position on the convex hull.
	VtkLineSegmentsVisualization targetToHullLineSegments;

	// A visualization of the planned visitation order of the robot as a polyline starting from the robot's end-effector and passing through all the target points in order.
	VtkPolyLineVisualization visitOrderVisualization;

	// A visualization of the robot's end-effector as the line it will trace out according to the last-emitted path
	VtkPolyLineVisualization ee_trace_visualization;

	/// A visualization of the robot's model of the hard surface of the tree.
	VtkPointCloudVisualization obstacleSurfacepoints;

	Viewer(const SimplifiedOrchard &orchard, VtkRobotmodel &robotModel, const std::vector<vtkActor *> &actors);

	void addActor(vtkActor *actor);

	void addActorCollection(vtkActorCollection *actors);

	void requestRender();

	void start();

	void setIntervalCallback(const std::function<void()> &callback);

	void updateAlgorithmVisualization(const DynamicMeshHullAlgorithm &algorithm);
};

#endif //NEW_PLANNERS_VIEWER_H
