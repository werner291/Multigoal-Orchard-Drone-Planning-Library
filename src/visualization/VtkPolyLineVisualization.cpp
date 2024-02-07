// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#include "VtkPolyLineVisualization.h"
#include <vtkProperty.h>

VtkPolyLineVisualization::VtkPolyLineVisualization(float r, float g, float b) {
	mapper->SetInputData(polyData);
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(r,g,b);
	actor->GetProperty()->SetLineWidth(5);
	actor->GetProperty()->SetPointSize(8);
}

vtkActor *VtkPolyLineVisualization::getActor() {
	return actor;
}

void VtkPolyLineVisualization::updateLine(const std::vector<mgodpl::math::Vec3d> &points) {

	vtkNew<vtkPoints> pointsVtk;
	vtkNew<vtkCellArray> cells;

	vtkNew<vtkCellArray> pointsCells;

	if (!points.empty()) {
		auto previousPointId = pointsVtk->InsertNextPoint(points[0].data());

		pointsCells->InsertNextCell(1);
		pointsCells->InsertCellPoint(previousPointId);

		for (size_t i = 1; i < points.size(); ++i) {
			cells->InsertNextCell(2);
			cells->InsertCellPoint(previousPointId);
			cells->InsertCellPoint(previousPointId = pointsVtk->InsertNextPoint(points[i].data()));

			pointsCells->InsertNextCell(1);
			pointsCells->InsertCellPoint(previousPointId);
		}
	}

	polyData->SetPoints(pointsVtk);
	polyData->SetLines(cells);
	polyData->SetVerts(pointsCells);

	polyData->Modified();
}