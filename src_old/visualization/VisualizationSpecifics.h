
/*
 * File containing code for relating to VTK stuff that is specific to the orchard visualization.
 */

#ifndef NEW_PLANNERS_VISUALIZATIONSPECIFICS_H
#define NEW_PLANNERS_VISUALIZATIONSPECIFICS_H

#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include "../StreamingConvexHull.h"
#include "../TreeMeshes.h"
#include "VtkRobotModel.h"

/**
 * A wrapper around the Vtk stuff necessary to visualize the StreamingConvexHull.
 */
struct ConvexHullActor {

	vtkNew<vtkActor> actor;
	vtkNew<vtkPolyData> polyData;
	vtkNew<vtkPolyDataMapper> mapper;

	ConvexHullActor();

	/**
	 * Extract the current convex hull from the StreamingConvexHull and update the actor.
	 *
	 * @param convexHull 		The StreamingConvexHull to extract the convex hull from.
	 */
	void update(const shape_msgs::msg::Mesh &mesh);
};

#endif //NEW_PLANNERS_VISUALIZATIONSPECIFICS_H
