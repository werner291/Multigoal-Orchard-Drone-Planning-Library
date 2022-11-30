// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "shape_generation.h"

std::vector<Eigen::Vector3d>
spherifiedCubeVertices(size_t segments) {// Generate the vertices of a subdivided cube and normalize them.
	std::vector<Eigen::Vector3d> directions;

	for (size_t i = 0; i <= segments; i++) {
		double x = ((double) i / (double) segments) - 0.5;

		for (size_t j = 0; j <= segments; j++) {
			double y = ((double) j / (double) segments) - 0.5;

			for (size_t k = 0; k <= segments; k++) {

				double z = ((double) k / (double) segments) - 0.5;

				// To avoid the interior points, check if we're either minimizing or maximizing one of the coordinates.
				if (i == 0 || i == segments || j == 0 || j == segments || k == 0 || k == segments) {
					directions.emplace_back(x, y, z);
					directions.back().normalize();
				}
			}

		}
	}
	return directions;
}
