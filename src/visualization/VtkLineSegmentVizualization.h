// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#ifndef MGODPL_VTKLINESEGMENTVIZUALIZATION_H
#define MGODPL_VTKLINESEGMENTVIZUALIZATION_H

#include <vtkActor.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>

#include <vector>

#include "../math/Vec3.h"

class VtkLineSegmentsVisualization {
	vtkNew<vtkPolyData> visitOrderVisualizationData;
	vtkNew<vtkPolyDataMapper> visitOrderVisualizationMapper;
	vtkNew<vtkActor> visitOrderVisualizationActor;

public:
	VtkLineSegmentsVisualization(float r, float g, float b);

	vtkActor* getActor();

	void updateLine(const std::vector<std::pair<mgodpl::math::Vec3d,mgodpl::math::Vec3d>>& points);

	void setColors(const std::vector<mgodpl::math::Vec3d>& colors);
};

#endif //MGODPL_VTKLINESEGMENTVIZUALIZATION_H
