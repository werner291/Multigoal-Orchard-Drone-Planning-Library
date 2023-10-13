// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_VTKPOLYLINEVISUALIZATION_H
#define MGODPL_VTKPOLYLINEVISUALIZATION_H

#include <vtkActor.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include "../math/Vec3.h"

class VtkPolyLineVisualization {
	vtkNew<vtkPolyData> visitOrderVisualizationData;
	vtkNew<vtkPolyDataMapper> visitOrderVisualizationMapper;
	vtkNew<vtkActor> visitOrderVisualizationActor;

public:
	VtkPolyLineVisualization(float r, float g, float b);

	vtkActor* getActor();

	void updateLine(const std::vector<mgodpl::math::Vec3d>& points);
};

#endif //MGODPL_VTKPOLYLINEVISUALIZATION_H
