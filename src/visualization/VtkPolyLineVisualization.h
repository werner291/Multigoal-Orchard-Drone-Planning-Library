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

/**
 * @class VtkPolyLineVisualization
 * @brief A class to visualize polylines using VTK.
 *
 * This class provides an interface for creating, updating and retrieving a polyline visualization using the VTK library.
 */
class VtkPolyLineVisualization {
 vtkNew<vtkPolyData> polyData; ///< The vtkPolyData object to store the polyline data.
 vtkNew<vtkPolyDataMapper> mapper; ///< The vtkPolyDataMapper to map the polyline data to graphics primitives.
 vtkNew<vtkActor> actor; ///< The vtkActor to represent the polyline in a rendering scene.

public:
 /**
  * @brief Construct a new VtkPolyLineVisualization object.
  *
  * @param r The red component of the color of the polyline.
  * @param g The green component of the color of the polyline.
  * @param b The blue component of the color of the polyline.
  */
 VtkPolyLineVisualization(float r, float g, float b);

 /**
  * @brief Get the vtkActor representing the polyline.
  *
  * @return vtkActor* The vtkActor representing the polyline.
  */
 vtkActor* getActor();

 /**
  * @brief Update the polyline with a new set of points.
  *
  * @param points The new set of points for the polyline.
  */
 void updateLine(const std::vector<mgodpl::math::Vec3d>& points);
};

#endif //MGODPL_VTKPOLYLINEVISUALIZATION_H