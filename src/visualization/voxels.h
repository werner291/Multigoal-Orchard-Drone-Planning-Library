// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.


#pragma once

#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkActor.h>
#include <vtkNamedColors.h>
#include <vtkPolyData.h>
#include "../math/Vec3.h"
#include "../math/AABBGrid.h"

namespace mgodpl {
	template<typename T>
	class Grid3D;
}

namespace mgodpl::math {
	class AABBGrid;
}

namespace mgodpl::visualization {

	/**
	 * Convert a grid to a set of points.
	 * @param SUBDIVISIONS The number of subdivisions in the grid.
	 * @param grid_coords The grid coordinates.
	 * @param grid The grid.
	 * @param negate If true, the grid is negated.
	 * @return The points.
	 */
	vtkNew <vtkPoints> grid_to_points(const size_t SUBDIVISIONS,
									  const mgodpl::math::AABBGrid &grid_coords,
									  const Grid3D<bool> &grid,
									  bool negate);

	class VtkVoxelGrid {

		mgodpl::math::AABBGrid grid_coords;
		vtkNew<vtkPolyData> polydata;
		bool negate;

	public:
		vtkNew<vtkActor> actor;

		explicit VtkVoxelGrid(const mgodpl::math::AABBGrid &grid_coords,
							  const Grid3D<bool> &grid,
							  const mgodpl::math::Vec3d &rgb,
							  bool negate = false);

		void update(const Grid3D<bool> &grid);

		[[nodiscard]] inline vtkActor *getActor() const {
			return actor;
		}
	};
}