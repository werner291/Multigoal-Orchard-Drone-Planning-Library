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

	/**
	 * @brief A struct representing a point on a surface.
	 *
	 * This struct encapsulates the position and normal vector of a point on a surface.
	 */
	struct SurfacePoint {
	    math::Vec3d position; ///< The position of the point on the surface.
	    math::Vec3d normal; ///< The normal vector at the point on the surface.
	};

	/**
	 * @brief A struct encapsulating scannable points parameters.
	 *
	 * This struct encapsulates the maximum distance, minimum distance, and maximum angle for scanning checks,
	 * as well as a vector of SurfacePoint objects for which scanning is to be performed.
	 */
	struct ScannablePoints {
	    double max_distance; ///< The maximum distance for scanning checks.
	    double min_distance; ///< The minimum distance for scanning checks.
	    double max_angle; ///< The maximum angle for scanning checks.
	    std::vector<SurfacePoint> surface_points; ///< The vector of SurfacePoint objects for which scanning is to be performed.

	    /**
	     * @brief Constructor for the ScannablePoints struct.
	     *
	     * This constructor initializes the max_distance, min_distance, max_angle, and surface_points members.
	     *
	     * @param max_distance The maximum distance for scanning checks.
	     * @param min_distance The minimum distance for scanning checks.
	     * @param max_angle The maximum angle for scanning checks.
	     * @param surface_points The vector of SurfacePoint objects for which scanning is to be performed.
	     */
	    ScannablePoints(double max_distance, double min_distance, double max_angle, const std::vector<SurfacePoint>& surface_points)
	        : max_distance(max_distance), min_distance(min_distance), max_angle(max_angle), surface_points(surface_points) {}
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

    /**
	 * @brief Creates a ScannablePoints object.
	 *
	 * This function encapsulates the `sample_points_on_mesh` function and returns a `ScannablePoints` object.
	 * The `ScannablePoints` object contains the maximum distance, minimum distance, and maximum angle for scanning checks,
	 * as well as a vector of SurfacePoint objects for which scanning is to be performed.
	 *
	 * @param rng A reference to a random number generator.
	 * @param mesh A reference to the mesh from which points are to be sampled.
	 * @param num_points The number of points to be sampled from the mesh.
	 * @param max_distance The maximum distance for scanning checks.
	 * @param min_distance The minimum distance for scanning checks.
	 * @param max_angle The maximum angle for scanning checks.
	 * @return A ScannablePoints object.
	 */
	ScannablePoints createScannablePoints(random_numbers::RandomNumberGenerator &rng,
	                                      const shape_msgs::msg::Mesh &mesh,
	                                      size_t num_points,
	                                      double max_distance,
	                                      double min_distance,
	                                      double max_angle);
}

#endif //MGODPL_SURFACE_POINTS_H
