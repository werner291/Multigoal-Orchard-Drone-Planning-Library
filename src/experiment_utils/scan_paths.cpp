// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-2-24.
//

#include "scan_paths.h"
#include "../math/Ray.h"
#include "../planning/state_tools.h"

namespace mgodpl
{
    ParametricPath fixed_radius_equatorial_orbit(const math::Vec3d center, double radius)
    {
        return [center, radius](double t) {
            const double angle = t * 2.0 * M_PI;
            return center + math::Vec3d{std::cos(angle), std::sin(angle), 0.0} * radius;
        };
    }

    ParametricPath polar_orbit(const math::Vec3d center, double radius,
                               double ascending_node_longitude)
    {
        return [center, radius, ascending_node_longitude](double t) {
            const double angle = t * 2 * M_PI; // angle from -PI (North Pole) to PI (South Pole) and back to -PI (North Pole)
            const double z = std::sin(angle) * radius;
            const double r_xy = std::cos(angle) * radius; // radius in the xy plane
            const double x = std::cos(ascending_node_longitude) * r_xy;
            const double y = std::sin(ascending_node_longitude) * r_xy;
            return center + math::Vec3d{x, y, z};
        };
    }

    ParametricPath ray_path(const math::Vec3d& center, const math::Vec3d& vec,
        double distance)
    {
        // Calculate the point through which the ray passes
        math::Vec3d point = center + vec;

        // Create a ray that passes through the calculated point and is perpendicular to the vector
        math::Ray ray(point, vec.cross(math::Vec3d::UnitZ()));

        return [ray, distance](double t) {
            // The path extends an equal distance on either side of the point, so we adjust `t` to range from -1 to 1
            return ray.pointAt((t - 0.5) * 2 * distance);
        };
    }

    ParametricPath helical_path(const math::Vec3d& center, double radius, int turns, double height)
    {
        return [center, radius, turns, height](double t) {
            const double angle = t * 2.0 * M_PI * turns;
            return center + math::Vec3d{std::cos(angle) * radius, std::sin(angle) * radius, t * height - height / 2};
        };
    }

    mgodpl::ParametricPath vertical_oscillation_path(const mgodpl::math::Vec3d& center, double radius, double amplitude,
        int cycles)
    {
        return [center, radius, amplitude, cycles](double t) {
            const double angle = t * 2.0 * M_PI;
            const double z = std::sin(angle * cycles) * amplitude;
            return center + mgodpl::math::Vec3d{std::cos(angle) * radius, std::sin(angle) * radius, z};
        };
    }

    mgodpl::ParametricPath latitude_oscillation_path(const mgodpl::math::Vec3d& center, double radius, double amplitude,
        int cycles)
    {
        return [center, radius, amplitude, cycles](double t) {
            const double angle = t * 2.0 * M_PI;
            const double latitude = std::sin(angle * cycles) * amplitude * M_PI / 2.0;
            const double x = std::cos(angle) * radius * std::cos(latitude);
            const double y = std::sin(angle) * radius * std::cos(latitude);
            const double z = radius * std::sin(latitude);
            return center + mgodpl::math::Vec3d{x, y, z};
        };
    }

    PathEvaluationResult evaluatePath(const ParametricPath& path, const ScannablePoints& scannable_points,
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

	RobotPath parametricPathToRobotPath(const robot_model::RobotModel &robot,
										const math::Vec3d &tree_center,
										const ParametricPath &euclidean_path,
										size_t n_steps) {

		// Create a RobotPath from these states
		RobotPath path;

		// Generate a sequence of time values between 0 and 1
		for (size_t i = 0; i <= n_steps; ++i) {

			// Calculate the time parameter in range [0, 1] (inclusive)
			double t = static_cast<double>(i) / static_cast<double>(n_steps);

			// Call the ParametricPath function to get the position at time t
			math::Vec3d position = euclidean_path(t);

			// Calculate the relative vector from the tree center
			math::Vec3d relative_vector = position - tree_center;

			// Add the state to the path
			path.states.push_back(fromEndEffectorAndVector(robot, position, relative_vector));
		}

		// Return the path
		return path;
	}
}