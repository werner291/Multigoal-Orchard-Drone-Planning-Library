//
// Created by werner on 29-11-23.
//

#ifndef VTKTRIANGLESETVISUALIZATION_H
#define VTKTRIANGLESETVISUALIZATION_H

#include <vector>
#include <vtkNew.h>
#include "../math/Vec3.h"

class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;

class VtkTriangleSetVisualization {
    vtkNew<vtkPolyData> data;
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;

public:
    VtkTriangleSetVisualization(float r, float g, float b);

    vtkActor* getActor();

    void updateTriangles(const std::vector<std::array<mgodpl::math::Vec3d, 3>>& points);
};


#endif //VTKTRIANGLESETVISUALIZATION_H
