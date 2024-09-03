// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3-9-24.
//

#include "TraceVisualization.h"

namespace mgodpl::visualization {
	TraceVisualisation::TraceVisualisation(SimpleVtkViewer &viewer,
										   const std::array<double, 3> &color,
										   size_t max_points)
			: max_points(max_points) {
		ee_trace_vis = std::make_unique<VtkPolyLineVisualization>(color[0], color[1], color[2]);
		viewer.addActor(ee_trace_vis->getActor());
	}

	void TraceVisualisation::add_point(const math::Vec3d &point) {
		ee_trace.push_back(point);
		if (ee_trace.size() > max_points) {
			ee_trace.erase(ee_trace.begin());
		}
		ee_trace_vis->updateLine(ee_trace);
	}

	void TraceVisualisation::clear() {
		ee_trace.clear();
		ee_trace_vis->updateLine(ee_trace);
	}
} // mgodpl::visualization