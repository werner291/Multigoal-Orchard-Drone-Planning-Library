// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#ifndef MGODPL_VISUALIZATION_SCANNABLE_POINTS_H
#define MGODPL_VISUALIZATION_SCANNABLE_POINTS_H

#include "VtkLineSegmentVizualization.h"
#include "../experiment_utils/surface_points.h"
#include "SimpleVtkViewer.h"

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
VtkLineSegmentsVisualization createFruitLinesVisualization(const mgodpl::ScannablePoints &scannable_points);

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
VtkLineSegmentsVisualization createFruitLinesVisualization(const std::vector<mgodpl::SurfacePoint> &scannable_points);

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
std::vector<mgodpl::math::Vec3d> generateVisualizationColors(const mgodpl::SeenPoints &ever_seen);


/**
 * \brief Visualizes the fruit points using VtkLineSegmentsVisualization.
 *
 * This function creates a visualization of the fruit points and adds it to the viewer.
 *
 * \param viewer The viewer to which the visualization will be added.
 * \param scannable_points The scannable points representing the fruit surface.
 * \param initial_seen_status The SeenPoints object tracking which points have been seen.
 */
VtkLineSegmentsVisualization
visualize(mgodpl::SimpleVtkViewer &viewer,
          const mgodpl::ScannablePoints &scannable_points,
          const mgodpl::SeenPoints &initial_seen_status);

/**
 * \brief Visualizes the fruit points using VtkLineSegmentsVisualization.
 *
 * This function creates a visualization of the fruit points and adds it to the viewer.
 *
 * \param viewer The viewer to which the visualization will be added.
 * \param scannable_points The scannable points representing the fruit surface.
 * \param initial_seen_status The SeenPoints object tracking which points have been seen.
 */
VtkLineSegmentsVisualization visualize(mgodpl::SimpleVtkViewer &viewer,
                                       const std::vector<mgodpl::SurfacePoint> &scannable_points,
                                       const mgodpl::SeenPoints &initial_seen_status);

/**
 * \brief Updates the visualization of the fruit points based on the seen points.
 *
 * This function updates the colors of the fruit points visualization to reflect which points have been seen.
 *
 * \param ever_seen The SeenPoints object tracking which points have been seen.
 * \param fruit_points_visualization The visualization object for the fruit points.
 */
void update_visualization(const mgodpl::SeenPoints &ever_seen,
						  VtkLineSegmentsVisualization &fruit_points_visualization);

#endif //MGODPL_VISUALIZATION_SCANNABLE_POINTS_H
