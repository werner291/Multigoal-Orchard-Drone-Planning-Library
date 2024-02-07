// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-2-24.
//

#include "scan_paths.h"

mgodpl::ParametricPath mgodpl::fixed_radius_equatorial_orbit(const math::Vec3d& center, double radius)
{
    return [center, radius](double t) {
        const double angle = t * 2.0 * M_PI;
        return center + math::Vec3d{std::cos(angle), std::sin(angle), 0.0} * radius;
    };
}

mgodpl::PathEvaluationResult mgodpl::evaluatePath(const ParametricPath& path, const ScannablePoints& scannable_points,
    SeenPoints& ever_seen, int num_segments)
{
    // Variable to keep track of the total distance traversed
    double total_distance = 0.0;
    // Store the previous eye position
    math::Vec3d previous_eye_position = path(0.0);

    // Loop over each segment
    for (int i = 0; i <= num_segments; ++i) {
        // Calculate t for the current segment
        double t = static_cast<double>(i) / num_segments;
        // Calculate the new eye position using the path function
        math::Vec3d eye_position = path(t);

        // Calculate the distance traversed
        total_distance += (eye_position - previous_eye_position).norm();
        // Update the previous eye position
        previous_eye_position = eye_position;

        // Update the visibility of the points
        update_visibility(scannable_points, eye_position, ever_seen);
    }

    // Calculate the number of points seen
    size_t num_seen = ever_seen.count_seen();

    // Return the number of points seen and the total distance traversed
    return {num_seen, total_distance};
}
