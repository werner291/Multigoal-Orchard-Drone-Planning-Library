// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 30-1-24.
//

#ifndef MGODPL_SURFACE_POINTS_H
#define MGODPL_SURFACE_POINTS_H

#include <random_numbers/random_numbers.h>
#include <shape_msgs/msg/mesh.hpp>
#include "../math/Vec3.h"

namespace mgodpl {
	struct SurfacePoint {
		math::Vec3d position;
		math::Vec3d normal;
	};

	/**
	 * Generate a random barycentric coordinate uniformly distributed over the triangle.
	 *
	 * @param rng	The random number generator.
	 * @return		The random barycentric coordinate.
	 */
	math::Vec3d random_barycentric(random_numbers::RandomNumberGenerator &rng);

	/**
	 * This function samples points on a mesh surface. It first calculates the cumulative areas of all triangles in the mesh.
	 * Then, it generates random points on the mesh surface by selecting a triangle based on its area and generating a random
	 * point within it.
	 *
	 * @param rng 			A reference to a random number generator.
	 * @param mesh 			A reference to the mesh from which points are to be sampled.
	 * @param num_points 	The number of points to be sampled from the mesh.
	 *
	 * @return A vector of SurfacePoint structures. Each SurfacePoint contains a position and a normal vector.
	 */
	std::vector<SurfacePoint> sample_points_on_mesh(random_numbers::RandomNumberGenerator &rng,
													const shape_msgs::msg::Mesh &mesh,
													size_t num_points);
}

#endif //MGODPL_SURFACE_POINTS_H
