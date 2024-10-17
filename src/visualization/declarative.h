// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/12/24.
//

#ifndef MGODPL_DECLARATIVE_H
#define MGODPL_DECLARATIVE_H

#include "SimpleVtkViewer.h"
#include "../experiment_utils/tree_models.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/parametric_paths.h"
#include "VtkPolyLineVisualization.h"

namespace mgodpl::cgal {
	struct CgalMeshData;
}

namespace mgodpl::visualization {
	void visualize(mgodpl::SimpleVtkViewer &viewer, const mgodpl::declarative::FruitModels &fruitModels);

	void visualize(mgodpl::SimpleVtkViewer &viewer, const mgodpl::declarative::FullTreeModel &model);

	void visualize(mgodpl::SimpleVtkViewer &viewer, const std::vector<std::vector<SurfacePoint> > &scannablePoints);

	void visualize(mgodpl::SimpleVtkViewer &viewer,
	               const ParametricPath &path,
	               int n_points = 100,
	               const math::Vec3d &color = {1, 0, 1});

	vtkSmartPointer<vtkActor> visualize(SimpleVtkViewer &viewer, const cgal::CgalMeshData &tree_convex_hull);
}

#endif //MGODPL_DECLARATIVE_H
