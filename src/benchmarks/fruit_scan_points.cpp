// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include <map>

#include "benchmark_function_macros.h"
#include "../visualization/SimpleVtkViewer.h"
#include "../visualization/VtkLineSegmentVizualization.h"
#include "../experiment_utils/TreeMeshes.h"
#include "../experiment_utils/surface_points.h"
#include "../experiment_utils/scan_paths.h"
#include "../visualization/VtkPolyLineVisualization.h"
#include "../visualization/visualization_function_macros.h"
#include "../experiment_utils/scan_path_generators.h"
#include "../visualization/scannable_points.h"

using namespace mgodpl;

REGISTER_VISUALIZATION(visualize_several_orbits_simultaneously) {
    // Load the tree meshes
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Grab the fruit mesh
    Mesh fruit_mesh = tree_model.fruit_meshes[0];

    // Calculate the center of the fruit mesh
    math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

    // Initialize a random number generator
    random_numbers::RandomNumberGenerator rng;

    // Define constants for the scannable points
    const size_t NUM_POINTS = 200;
    const double MAX_DISTANCE = INFINITY;
    const double MIN_DISTANCE = 0;
    const double MAX_ANGLE = M_PI / 3.0;

    // Create the scannable points
    ScannablePoints scannable_points = createScannablePoints(rng,
                                                             fruit_mesh,
                                                             NUM_POINTS,
                                                             MAX_DISTANCE,
                                                             MIN_DISTANCE,
                                                             MAX_ANGLE);

    // Add the fruit mesh to the viewer
    viewer.addMesh(tree_model.fruit_meshes[0], {1.0, 0.0, 0.0}, 1.0);

    // Define the initial orbit radius
    const double EYE_ORBIT_RADIUS = 0.1;

    // Create several orbit functions
    std::vector<ParametricPath> orbits;
    orbits.reserve(20);
    for (int i = 0; i < 20; ++i) {
        // Create an orbit function for each radius and add it to the orbits vector
        orbits.push_back(fixed_radius_equatorial_orbit(fruit_center, EYE_ORBIT_RADIUS + i * 0.1));
    }

    // Declare a vector to store the eye positions for each orbit
    std::vector<std::vector<mgodpl::math::Vec3d> > eye_positions(orbits.size());

    // Create several instances of VtkPolyLineVisualization
    std::vector<VtkPolyLineVisualization> eye_positions_visualizations;
    for (size_t i = 0; i < orbits.size(); ++i) {
        // Create a VtkPolyLineVisualization for each orbit and add it to the viewer
        eye_positions_visualizations.emplace_back(1, 0, 0); // Red color
        viewer.addActor(eye_positions_visualizations.back().getActor());
    }

    // Add a timer callback to the viewer
    viewer.addTimerCallback([&]() {
        static double t = 0.0;
        t += 0.01;

        for (size_t i = 0; i < orbits.size(); ++i) {
            // Use the orbit function to set the eye_position
            math::Vec3d eye_position = orbits[i](t);

            // Update the vector with the new eye position
            eye_positions[i].push_back(eye_position);

            // Update the polyline with the new set of eye positions
            eye_positions_visualizations[i].updateLine(eye_positions[i]);
        }

        // Stop the viewer when the timer exceeds 1.0
        if (t > 1.0) {
            viewer.stop();
        }
    });

    // Set the camera transform for the viewer
    viewer.setCameraTransform(fruit_center + math::Vec3d{1.5, 0.0, 1.0}, fruit_center);

    // Start the viewer
    viewer.start();
}

REGISTER_VISUALIZATION(scan_progressive_orbit) {
    // Load the tree meshes
    auto tree_model = tree_meshes::loadTreeMeshes("appletree");

    // Grab the fruit mesh
    Mesh fruit_mesh = tree_model.fruit_meshes[0];

    // Calculate the center of the fruit mesh
    math::Vec3d fruit_center = mesh_aabb(fruit_mesh).center();

    // Initialize a random number generator
    random_numbers::RandomNumberGenerator rng;

    // Create the scannable points
    ScannablePoints scannable_points = createScannablePoints(rng, fruit_mesh, 1000, INFINITY, 0, M_PI / 3.0);

    // Initialize all points as unseen
    SeenPoints ever_seen = SeenPoints::create_all_unseen(scannable_points);

    // Create the fruit points visualization
    VtkLineSegmentsVisualization fruit_points_visualization = createFruitLinesVisualization(scannable_points);

    // Add the fruit points visualization to the viewer
    viewer.addActor(fruit_points_visualization.getActor());

    // Add the fruit mesh to the viewer
    viewer.addMesh(tree_model.fruit_meshes[0], {0.8, 0.8, 0.8}, 1.0);

    // Define the initial orbit radius
    double EYE_ORBIT_RADIUS = 0.1;

    // Declare a vector to store the eye positions
    std::vector<mgodpl::math::Vec3d> eye_positions;

    // Create an instance of VtkPolyLineVisualization
    VtkPolyLineVisualization eye_positions_visualization(1, 0, 0); // Red color

    // Add the polyline visualization to the viewer
    viewer.addActor(eye_positions_visualization.getActor());

    std::vector<JsonMeta<ParametricPath> > orbits = getOrbits(fruit_center, EYE_ORBIT_RADIUS);

    const bool LOOP_ANIMATION = false;

    // Add a timer callback to the viewer
    viewer.addTimerCallback([&]() {
        // Define a static variable for the timer
        static double t = 0.0;
        // Define a static variable for the current orbit index
        static size_t current_orbit_index = 0;

        t += 0.01;

        // If the timer exceeds 1.0, reset it and move to the next orbit
        if (t > 1.0) {
            t = 0.0;
            // Increment the current orbit index, modulo the number of orbits
            current_orbit_index = (current_orbit_index + 1) % orbits.size();

            if (current_orbit_index == 0 && !LOOP_ANIMATION) {
                viewer.stop();
            }

            // Clear the eye positions vector
            eye_positions.clear();

            // Reset the scanned points
            ever_seen = SeenPoints::create_all_unseen(scannable_points);
        }

        // Use the orbit function to set the eye_position
        math::Vec3d eye_position = orbits[current_orbit_index].data(t);

        // Update the vector with the new eye position
        eye_positions.push_back(eye_position);

        // Update the polyline with the new set of eye positions
        eye_positions_visualization.updateLine(eye_positions);

        // Update the visibility of the scannable points
        update_visibility(scannable_points, eye_position, ever_seen);

        // Set the colors of the fruit points visualization
        fruit_points_visualization.setColors(generateVisualizationColors(ever_seen));
    });

    // Set the camera transform for the viewer
    viewer.setCameraTransform(fruit_center + math::Vec3d{2.0, 1.0, 1.0}, fruit_center);

    // Start the viewer
    viewer.start();
}

REGISTER_VISUALIZATION(visualize_sphere_samples) {
    // Initialize the random number generator
    random_numbers::RandomNumberGenerator rng(42);

    const bool USE_QUASI_RANDOM = true;

    // Define the radius of the sphere
    const double radius = 0.05;

    // Number of points to sample
    size_t num_points = 1024;

    const auto points = USE_QUASI_RANDOM
                            ? sample_points_on_sphere_quasi_random(rng, num_points, radius)
                            : sample_points_on_sphere(rng, num_points, radius);

    const SeenPoints ever_seen = SeenPoints::create_all_unseen(points);

    visualize(viewer, points, ever_seen);

    // Add a sphere with radius 1.0 at the origin
    viewer.addSphere(radius, {0.0, 0.0, 0.0}, {0.8, 0.8, 0.5}, 1.0, 32);

    // Start the viewer
    viewer.start();
}

/**
 * A sensor model `SensorModelParameters` is defined by a tuple of real-valued parameters `(min_distance, max_distance, fov_angle, surface_max_angle)`, where:
 *
 * Given sensor model parameters `SensorModelParameters` and inputs:
 *
 * - `(eye_pos, eye_forward)`: Sensor position and direction, derived from a configuration by forward kinematics.
 * - `(point.position, point.normal)`: Surface point and normal vector.
 *
 * TODO: This is redundant with ScalarModelParameters, consider merging them.
 */
struct SensorModelParameters {
    double min_distance = 0.0;
    double max_distance = INFINITY;
    double surface_max_angle = M_PI / 2.0;
    double fov_angle = M_PI / 2.0;

    /**
     * Checks if a given surface point is visible from a specified eye position and direction.
     *
     * @param eye_pos The position of the eye (sensor).
     * @param eye_forward The forward direction vector of the eye (sensor).
     * @param point The surface point to check visibility for.
     * @return True if the point is visible, false otherwise.
     */
    [[nodiscard]] bool is_visible(
        math::Vec3d eye_pos,
        math::Vec3d eye_forward,
        const SurfacePoint &point) const {
        math::Vec3d delta = eye_pos - point.position;
        double distance = delta.norm();

        return distance <= max_distance &&
               distance >= min_distance &&
               std::acos(point.normal.dot(delta) / distance) <= surface_max_angle &&
               std::acos(eye_forward.dot(-delta) / distance) <= fov_angle;
    }

    [[nodiscard]] Json::Value toJson() const {
        Json::Value json;
        json["min_distance"] = min_distance;
        json["max_distance"] = max_distance;
        json["surface_max_angle"] = surface_max_angle;
        json["fov_angle"] = fov_angle;
        return json;
    }
};

REGISTER_BENCHMARK(sphere_sampling_orbit_radii) {
    // Initialize the random number generator
    random_numbers::RandomNumberGenerator rng(42);

    const double target_radius = 0.05;

    const bool USE_QUASI_RANDOM = true;

    // Number of points to sample
    size_t num_points = 1024;

    const auto points = USE_QUASI_RANDOM
                            ? sample_points_on_sphere_quasi_random(rng, num_points, target_radius)
                            : sample_points_on_sphere(rng, num_points, target_radius);

    // Keep it simple: infinite distance, 180 degree field of view, 90 degree surface angle
    const SensorModelParameters sensor_model{
        .min_distance = 0.0,
        .max_distance = INFINITY,
        .surface_max_angle = M_PI / 2.0,
        .fov_angle = M_PI
    };

    // Record some metadata:
    results["num_points"] = num_points;
    results["sensor_model"] = sensor_model.toJson();
    results["target_radius"] = target_radius;

    // Iterate over different radii, starting at the target radius, and ending at 50 times the target radius
    for (int radius_index = 1; radius_index <= 50; ++radius_index) {
        Json::Value radius_results;

        radius_results["radius"] = target_radius * static_cast<double>(radius_index);

        SeenPoints ever_seen = SeenPoints::create_all_unseen(points);

        const double radius = target_radius * static_cast<double>(radius_index);

        // For 1000 iterations, check the visibility of each point:
        for (int sample_index = 0; sample_index <= 1000; ++sample_index) {
            const double t = static_cast<double>(sample_index) / 1000.0;

            math::Vec3d eye_position = {radius * std::cos(2 * M_PI * t), radius * std::sin(2 * M_PI * t), 0.0};
            math::Vec3d eye_forward = -eye_position.normalized(); // Looking towards the origin (target center)

            // Update the visibility of the scannable points
            for (size_t i = 0; i < points.size(); ++i) {
                if (!ever_seen.ever_seen[i] && sensor_model.is_visible(eye_position, eye_forward, points[i])) {
                    ever_seen.ever_seen[i] = true;
                }
            }

            if (sample_index % 100 == 0) {
                Json::Value sample_results;

                double distance_traveled = radius * 2 * M_PI * t;
                sample_results["distance_traveled"] = distance_traveled;

                // Count the number of points that have been seen
                size_t n_seen = 0;
                for (const auto &seen: ever_seen.ever_seen) {
                    if (seen) {
                        ++n_seen;
                    }
                }

                sample_results["n_seen"] = n_seen;
                radius_results["samples"].append(sample_results);
            }
        }

        results["per_radius"].append(radius_results);
    }
}

// Define the radius of the sphere
const double TARGET_RADIUS = 0.05;

void visualize_paths(
    const std::vector<mgodpl::ParametricPath> &path,
    const SensorModelParameters &sensor_model,
    SimpleVtkViewer &viewer) {
    // Initialize the random number generator
    random_numbers::RandomNumberGenerator rng(42);


    // Number of points to sample
    size_t num_points = 1024;

    // Sample points on the sphere
    const auto points = sample_points_on_sphere_quasi_random(rng, num_points, TARGET_RADIUS);

    // Initialize all points as unseen
    SeenPoints ever_seen = SeenPoints::create_all_unseen(points);

    // Create the sphere points visualization
    VtkLineSegmentsVisualization sphere_points_visualization = createFruitLinesVisualization(points);

    // Add the sphere points visualization to the viewer
    viewer.addActor(sphere_points_visualization.getActor());

    // Add a sphere with the specified radius at the origin
    viewer.addSphere(TARGET_RADIUS, {0.0, 0.0, 0.0}, {0.8, 0.8, 0.8}, 1.0, 32);

    // Declare a vector to store the eye positions
    std::vector<mgodpl::math::Vec3d> eye_positions;

    // Create an instance of VtkPolyLineVisualization
    VtkPolyLineVisualization eye_positions_visualization(1, 0, 0); // Red color

    // Add the polyline visualization to the viewer
    viewer.addActor(eye_positions_visualization.getActor());

    // Loop the animation unless we're recording.
    bool loop_animation = !viewer.isRecording();

    size_t current_path_index = 0;

    // Add a timer callback to the viewer
    viewer.addTimerCallback([&]() {
        // Define a static variable for the timer
        static double t = 0.0;

        math::Vec3d delta = path[current_path_index](t + 0.01) - path[current_path_index](t - 0.01);

        t += 0.0005 / delta.norm(); // Adjust the speed of the animation as needed

        // If the timer exceeds 1.0, reset it
        if (t > 1.0) {
            t = 0.0;
            current_path_index += 1;
            current_path_index %= path.size();

            if (current_path_index == 0 && !loop_animation) {
                viewer.stop();
            } else {
                // Clear the eye positions vector
                eye_positions.clear();

                // Reset the scanned points
                ever_seen = SeenPoints::create_all_unseen(points);
            }
        }

        // Use the path function to set the eye_position
        math::Vec3d eye_position = path[current_path_index](t);

        // Update the vector with the new eye position
        eye_positions.push_back(eye_position);

        // Update the polyline with the new set of eye positions
        eye_positions_visualization.updateLine(eye_positions);

        // Update the visibility of the scannable points
        for (size_t i = 0; i < points.size(); ++i) {
            if (!ever_seen.ever_seen[i] && sensor_model.
                is_visible(eye_position, -eye_position.normalized(), points[i])) {
                ever_seen.ever_seen[i] = true;
            }
        }

        // Set the colors of the sphere points visualization
        sphere_points_visualization.setColors(generateVisualizationColors(ever_seen));
    });

    // Set the camera transform for the viewer
    viewer.setCameraTransform({1.5, 0.0, 1.0}, {0.0, 0.0, 0.0});

    // Start the viewer
    viewer.start();
}

REGISTER_VISUALIZATION(scan_progressive_orbit_sphere) {
    std::vector<ParametricPath> paths;

    for (size_t orbit = 1; orbit < 10; ++orbit) {
        paths.push_back(fixed_radius_equatorial_orbit({0.0, 0.0, 0.0}, TARGET_RADIUS * static_cast<double>(orbit)));
    }

    visualize_paths(paths,
                    {
                        .min_distance = 0.0,
                        .max_distance = INFINITY,
                        .surface_max_angle = M_PI / 2.0,
                        .fov_angle = M_PI
                    },
                    viewer);
}

REGISTER_VISUALIZATION(scan_latitude_oscillation_sphere) {
    const int cycles = 4;
    const double amplitude = 0.5;

    std::vector<ParametricPath> paths;
    for (int i = 0; i < 5; ++i) {
        paths.push_back(
            latitude_oscillation_path({0.0, 0.0, 0.0}, TARGET_RADIUS * static_cast<double>(i + 1), amplitude, cycles)
        );
    }

    // Keep it simple: infinite distance, 180 degree field of view, 90 degree surface angle
    const SensorModelParameters sensor_model{
        .min_distance = 0.0,
        .max_distance = INFINITY,
        .surface_max_angle = M_PI / 2.0,
        .fov_angle = M_PI
    };

    visualize_paths(
        paths,
        sensor_model,
        viewer
    );
}

REGISTER_VISUALIZATION(scan_latitude_oscillation_sphere_limited_angle) {
    const double amplitude = 0.5;
    const double radius = TARGET_RADIUS * 3.0;

    std::vector<ParametricPath> paths;
    for (int i = 0; i < 5; ++i) {
        paths.push_back(
            latitude_oscillation_path({0.0, 0.0, 0.0}, radius, amplitude, i + 1)
        );
    }

    // Keep it simple: infinite distance, 180 degree field of view, 90 degree surface angle
    const SensorModelParameters sensor_model{
        .min_distance = 0.0,
        .max_distance = INFINITY,
        .surface_max_angle = 0.5 * M_PI / 2.0,
        .fov_angle = M_PI
    };

    visualize_paths(
        paths,
        sensor_model,
        viewer
    );
}

/**
 *  Generates a spiral pattern along a sphere, where the latitude and longitude increase linearly over time,
 *  with the longitude increasing faster than the latitude such as to create a spiral pattern.
 *
 * @param radius    The radius of the sphere.
 * @param cycles    The number of cycles of the spiral.
 *
 * @return          A function (double -> math::Vec3d) that returns the position on the path at time `t`.
 */
ParametricPath spiral(const double radius, const int cycles = 4) {
    return [=](double t) {
        double longitude = 2.0 * M_PI * t * static_cast<double>(cycles);
        double latitude = M_PI * (-0.5 + t);
        return math::Vec3d{
            radius * std::cos(longitude) * std::cos(latitude),
            radius * std::sin(longitude) * std::cos(latitude),
            radius * std::sin(latitude)
        };
    };
}

REGISTER_VISUALIZATION(scan_spiral_sphere) {
    // Keep it simple: infinite distance, 180 degree field of view, 90 degree surface angle
    const SensorModelParameters sensor_model{
        .min_distance = 0.0,
        .max_distance = INFINITY,
        .surface_max_angle = M_PI / 2.0,
        .fov_angle = M_PI
    };

    const int cycles = 2;

    std::vector<ParametricPath> paths;
    for (int i = 0; i < 5; ++i) {
        paths.push_back(spiral(TARGET_RADIUS * static_cast<double>(i + 1), cycles));
    }

    visualize_paths(
        paths,
        sensor_model,
        viewer);
}
