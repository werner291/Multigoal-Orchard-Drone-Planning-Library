//
// Created by werner on 29-11-23.
//

#include "VtkTriangleSetVisualization.h"

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>

VtkTriangleSetVisualization::VtkTriangleSetVisualization(float r, float g, float b, float opacity) {
    mapper->SetInputData(data);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(r, g, b);
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->SetLineWidth(2);
}

vtkActor *VtkTriangleSetVisualization::getActor() {
    return actor;
}

void VtkTriangleSetVisualization::updateTriangles(const std::vector<std::array<mgodpl::math::Vec3d, 3>>& points) {

    vtkNew<vtkPoints> pointsVtk;
    vtkNew<vtkCellArray> cells;

    for (const auto& [p1, p2, p3] : points) {
        cells->InsertNextCell(3);
        cells->InsertCellPoint(pointsVtk->InsertNextPoint(p1.components.data()));
        cells->InsertCellPoint(pointsVtk->InsertNextPoint(p2.components.data()));
        cells->InsertCellPoint(pointsVtk->InsertNextPoint(p3.components.data()));
    }

    data->SetPoints(pointsVtk);
    data->SetPolys(cells);

    data->Modified();

}