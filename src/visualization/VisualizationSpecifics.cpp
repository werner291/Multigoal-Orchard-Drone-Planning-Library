
#include "VisualizationSpecifics.h"
#include "../utilities/vtk.h"

#include <vtkProperty.h>
#include <vtkPointData.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

ConvexHullActor::ConvexHullActor() {
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(0.0, 1.0, 0.0);
	actor->GetProperty()->SetOpacity(0.8);
}

void ConvexHullActor::update(const shape_msgs::msg::Mesh &mesh) {
	mapper->SetInputData(rosMeshToVtkPolyData(mesh));
}