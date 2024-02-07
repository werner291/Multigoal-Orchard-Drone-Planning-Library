// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-2-24.
//

#ifndef SCAN_PATHS_H
#define SCAN_PATHS_H

#include <functional>
#include "../math/Vec3.h"
#include "surface_points.h"

namespace mgodpl
{
    // Type alias for a function that takes a double and returns a Vec3d.
    // The interpretation is that of a parametric path in 3D space,
    // where the input parameter is abstract time (between 0 and 1).
    using ParametricPath = std::function<math::Vec3d(double)>;

    /**
     * @brief Creates a parametric path representing a fixed-radius equatorial orbit around a given center point.
     *
     * This function generates a fixed-radius orbit around a point in the form of a parametric path.
     * The path is defined in the xy plane (z = 0) and is parameterized by a time parameter `t` between 0 and 1.
     * The position on the orbit at time `t` is calculated using the cosine and sine of `t * 2.0 * M_PI` to generate the x and y coordinates.
     *
     * @param center The center point of the orbit.
     * @param radius The radius of the orbit.
     * @return A function that takes a time parameter `t` between 0 and 1 and returns a `math::Vec3d` representing the position on the orbit at that time.
     */
    ParametricPath fixed_radius_equatorial_orbit(const math::Vec3d& center, double radius);

    /**
     * @brief A struct to hold the result of a path evaluation.
     *
     * This struct encapsulates the number of points seen and the total distance traveled
     * during the evaluation of a parametric path.
     */
    struct PathEvaluationResult {
        /**
         * @brief The number of points seen during the path evaluation.
         */
        size_t num_points_seen;

        /**
         * @brief The total distance traveled during the path evaluation.
         */
        double total_distance_traveled;
    };

    /**
     * @brief Evaluates a given path.
     *
     * This function takes a ParametricPath and evaluates it by calculating the number of points seen
     * and the total distance traveled. The function also updates the visibility status of each point
     * in the SeenPoints object.
     *
     * @param path The ParametricPath to evaluate.
     * @param scannable_points The ScannablePoints object containing the points to check for visibility.
     * @param ever_seen The SeenPoints object to update with the visibility status of each point.
     * @param num_segments The number of segments in the path.
     * @return A PathEvaluationResult struct containing the number of points seen and the total distance traveled.
     */
    PathEvaluationResult evaluatePath(const ParametricPath& path, const ScannablePoints& scannable_points,
                                      SeenPoints& ever_seen, int num_segments);
}

#endif //SCAN_PATHS_H
