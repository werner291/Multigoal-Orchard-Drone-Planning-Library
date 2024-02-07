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


	/**
	 * @brief Checks if a point is visible from a given position.
	 *
	 * This function takes a ScannablePoints object, a point index, and a Vec3d object representing the eye position.
	 * It computes the visibility of the point from the eye position based on the distance and angle.
	 * A point is considered visible if it is within the maximum distance and the angle between the point's normal
	 * and the vector from the point to the eye is less than the maximum angle.
	 *
	 * Note: This function does not account for occlusions. It only checks the visibility based on distance and angle.
	 *
	 * @param scannable_points A ScannablePoints object. Each SurfacePoint object in ScannablePoints represents a point in 3D space
	 *                         and has a position and a normal. The ScannablePoints object also contains the maximum distance and
	 *                         maximum angle to consider a point visible.
	 * @param point_index      The index of the point in the ScannablePoints object to check for visibility.
	 * @param eye_position     A Vec3d object representing the position of the eye in 3D space.
	 * @return                 A boolean value. If true, the point is visible from the eye position. If false, the point is not visible.
	 */
	bool is_visible(const ScannablePoints& scannable_points, size_t point_index, const math::Vec3d& eye_position);

	/**
	 * @brief Updates the visibility status of a set of points from a given eye position.
	 *
	 * This function takes a ScannablePoints object, a Vec3d object representing the eye position,
	 * and a vector of booleans representing whether each point has ever been seen.
	 * It updates the visibility status of each point in the vector. A point is considered visible
	 * if the dot product of the point's normal and the vector from the point to the eye is negative.
	 *
	 * @param scannable_points A ScannablePoints object. Each SurfacePoint object in ScannablePoints represents a point in 3D space
	 *                         and has a position and a normal. The ScannablePoints object also contains the maximum distance and
	 *                         maximum angle to consider a point visible.
	 * @param eye_position     A Vec3d object representing the position of the eye in 3D space.
	 * @param ever_seen        A vector of boolean values. Each value corresponds to a point in the input vector. If the value is true,
	 *                         the point has ever been seen from the eye position. If the value is false, the point has never been seen.
	 */
	void update_visibility(const ScannablePoints& scannable_points,
	                       const math::Vec3d& eye_position,
	                       std::vector<bool>& ever_seen);
}

#endif //MGODPL_SURFACE_POINTS_H
