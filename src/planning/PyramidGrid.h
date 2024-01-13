// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/13/24.
//

#ifndef MGODPL_PYRAMIDGRID_H
#define MGODPL_PYRAMIDGRID_H

#include <cstddef>
#include <vector>
#include "../math/Vec3.h"
#include "../math/Triangle.h"

namespace mgodpl {

	struct PyramidGrid {

		size_t x_cells, y_cells;
		const math::Vec3d local_x, local_y, local_z, center;
		double cos_fov, sin_fov;
		double arm_radius = 0.0;
		std::vector<std::vector<math::Triangle>> triangles;

		// Constructor.
		PyramidGrid(size_t x_cells,
					size_t y_cells,
					const math::Vec3d &local_x,
					const math::Vec3d &local_y,
					const math::Vec3d &local_z,
					const math::Vec3d &center,
					const double arm_radius,
					double fov) :
			x_cells(x_cells), y_cells(y_cells),
			local_x(local_x), local_y(local_y), local_z(local_z),
			center(center),
			cos_fov(cos(fov)), sin_fov(sin(fov)),
			arm_radius(arm_radius){
			triangles.resize(x_cells * y_cells);
		}

		void insert_triangle(const math::Triangle& tri) {

			// Step 1: transform to local coordinates.
			math::Vec3d a = tri.a - center;
			math::Vec3d b = tri.b - center;
			math::Vec3d c = tri.c - center;

			// Step 2: project onto the local x,y,z axes.
			math::Vec3d a_local(a.dot(local_x), a.dot(local_y), a.dot(local_z));
			math::Vec3d b_local(b.dot(local_x), b.dot(local_y), b.dot(local_z));
			math::Vec3d c_local(c.dot(local_x), c.dot(local_y), c.dot(local_z));

			// Combine to a local triangle.
			math::Triangle local_triangle(a_local, b_local, c_local);

			// Now, we do frustrum clipping.

		}

	};
}

#endif //MGODPL_PYRAMIDGRID_H
