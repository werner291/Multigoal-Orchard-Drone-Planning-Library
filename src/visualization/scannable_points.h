// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#ifndef MGODPL_SCANNABLE_POINTS_H
#define MGODPL_SCANNABLE_POINTS_H

#include "VtkLineSegmentVizualization.h"
#include "../experiment_utils/surface_points.h"

/**
   * @brief Creates a VtkLineSegmentsVisualization object for fruit points.
   *
   * This function creates a VtkLineSegmentsVisualization object that represents
   * the fruit points in the 3D space. Each point is represented as a line segment
   * that starts at the point's position and extends in the direction of the point's normal.
   *
   * @param scannable_points The scannable points on the fruit surface.
   * @return A VtkLineSegmentsVisualization object that can be used to visualize the fruit points.
   */
VtkLineSegmentsVisualization createFruitLinesVisualization(const mgodpl::ScannablePoints& scannable_points);

/**
   * @brief Creates a VtkLineSegmentsVisualization object for fruit points.
   *
   * This function creates a VtkLineSegmentsVisualization object that represents
   * the fruit points in the 3D space. Each point is represented as a line segment
   * that starts at the point's position and extends in the direction of the point's normal.
   *
   * @param scannable_points The scannable points on the fruit surface.
   * @return A VtkLineSegmentsVisualization object that can be used to visualize the fruit points.
   */
VtkLineSegmentsVisualization createFruitLinesVisualization(const std::vector<mgodpl::SurfacePoint>& scannable_points);

/**
 * @brief Generates colors for the visualization based on visibility of points.
 *
 * This function generates a vector of colors (represented as math::Vec3d objects)
 * for the visualization. Each color corresponds to a point. If the point has been seen,
 * the color is green (0.0, 1.0, 0.0). If the point has not been seen, the color is red (1.0, 0.0, 0.0).
 *
 * @param ever_seen The visibility status of each point.
 * @return A vector of colors for the visualization.
 */
std::vector<mgodpl::math::Vec3d> generateVisualizationColors(const mgodpl::SeenPoints& ever_seen);

#endif //MGODPL_SCANNABLE_POINTS_H
