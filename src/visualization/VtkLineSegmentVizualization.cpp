// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#include "VtkLineSegmentVizualization.h"

#include <vtkProperty.h>

VtkLineSegmentsVisualization::VtkLineSegmentsVisualization(float r, float g, float b) {
	visitOrderVisualizationMapper->SetInputData(visitOrderVisualizationData);
	visitOrderVisualizationActor->SetMapper(visitOrderVisualizationMapper);
	visitOrderVisualizationActor->GetProperty()->SetColor(r, g, b);
	visitOrderVisualizationActor->GetProperty()->SetLineWidth(2);
}

vtkActor *VtkLineSegmentsVisualization::getActor() {
	return visitOrderVisualizationActor;
}

void VtkLineSegmentsVisualization::updateLine(const std::vector<std::pair<mgodpl::math::Vec3d, mgodpl::math::Vec3d>> &points) {

	vtkNew<vtkPoints> pointsVtk;
	vtkNew<vtkCellArray> cells;

	for (const auto& [p1, p2] : points) {
		cells->InsertNextCell(2);
		cells->InsertCellPoint(pointsVtk->InsertNextPoint(p1.components.data()));
		cells->InsertCellPoint(pointsVtk->InsertNextPoint(p2.components.data()));
	}

	visitOrderVisualizationData->SetPoints(pointsVtk);
	visitOrderVisualizationData->SetLines(cells);

	visitOrderVisualizationData->Modified();

}