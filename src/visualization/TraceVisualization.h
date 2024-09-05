// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3-9-24.
//

#ifndef MGODPL_TRACEVISUALIZATION_H
#define MGODPL_TRACEVISUALIZATION_H

#include "VtkPolyLineVisualization.h"
#include "SimpleVtkViewer.h"
#include <memory>

namespace mgodpl::visualization {

	/**
	 * \brief A class to visualize the trace of the end-effector in a VTK viewer.
	 *
	 * This class manages a polyline visualization of the end-effector's trace, adding points to the trace
	 * and removing old points when the maximum number of points is exceeded.
	 */
	class TraceVisualisation {
	private:
		std::unique_ptr<VtkPolyLineVisualization> ee_trace_vis;
		std::vector<math::Vec3d> ee_trace;
		size_t max_points;

	public:
		/**
		 * \brief Constructor for TraceVisualisation.
		 *
		 * \param viewer The VTK viewer to which the trace visualization will be added.
		 * \param color The color of the trace visualization.
		 * \param max_points The maximum number of points in the trace. Old points are removed when this limit is exceeded.
		 */
		TraceVisualisation(SimpleVtkViewer &viewer, const std::array<double, 3> &color, size_t max_points = SIZE_MAX);

		/**
		 * \brief Adds a point to the trace visualization.
		 *
		 * \param point The point to be added to the trace.
		 */
		void add_point(const math::Vec3d &point);

		/**
		 * \brief Clears the trace visualization.
		 */
		void clear();
	};

} // visualization
// mgodpl

#endif //MGODPL_TRACEVISUALIZATION_H
