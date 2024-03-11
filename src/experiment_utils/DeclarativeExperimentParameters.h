// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/8/24.
//

#ifndef MGODPL_DECLARATIVEEXPERIMENTPARAMETERS_H
#define MGODPL_DECLARATIVEEXPERIMENTPARAMETERS_H

#include <json/value.h>
#include "scan_paths.h"

/**
 * @struct SensorScalarParameters
 * @brief A structure to hold the scalar parameters related to a sensor.
 */
struct SensorScalarParameters {
	double maxViewDistance; //< The maximum distance from the sensor a point can be to be considered visible.
	double minViewDistance; //< The minimum distance from the sensor a point can be to be considered visible.
	double fieldOfViewAngle; //< The field of view angle of the sensor (from the center to the edge).
	double maxScanAngle; //< The maximum angle a point can be from the forward direction of the sensor to be considered visible.
};

Json::Value toJson(const SensorScalarParameters &sensorParameters);

/**
 * @struct CircularOrbitParameters
 *
 * @brief A structure to hold the parameters related to a circular orbit.
 */
struct CircularOrbitParameters {
	double radius = 1.0; //< The radius of the orbit, in radii of the tree canopy.
	double inclination = 0.0; //< The inclination of the orbit, in radians.
	double ascendingNodeLongitude = 0.0; //< The longitude of the ascending node, in radians.
};

Json::Value toJson(const CircularOrbitParameters &orbitParameters);

/**
 * @struct SphericalOscillationParameters.
 */
struct SphericalOscillationParameters {
	double radius = 1.0; //< The radius of the oscillation, in radii of the tree canopy.
	double amplitude = 0.0; //< The amplitude of the oscillation, as a factor of PI/2 radius/
	unsigned int cycles = 1; //< The number of cycles of the oscillation.
};

Json::Value toJson(const SphericalOscillationParameters &oscillationParameters);

struct OrbitPathParameters {
	const std::variant<CircularOrbitParameters, SphericalOscillationParameters> parameters; //< The parameters for the orbit path.
};

namespace mgodpl {
	/**
	 * Instantiate a ParametricPath from an OrbitPathParameters.
	 *
	 * @param orbit 			The orbit parameters to use.
	 * @param tree_center 		The center of the tree that these paths are centered around.
	 * @param canopy_radius 	The radius of the tree's canopy.
	 * @return 					A ParametricPath that represents the given orbit with the given parameters.
	 */
	ParametricPath instantiatePath(const OrbitPathParameters &orbit,
								   const math::Vec3d &tree_center,
								   const double canopy_radius);
}

Json::Value toJson(const OrbitPathParameters &orbitPathParameters);

/// Use the original set of fruit.
struct Unchanged {
};

/// Select a random subset of the fruit.
struct RandomSubset {
	int count; //< The number of elements to select from the set.
};

/// Replace the set of fruit with a new set.
struct Replace {
	int count; //< The number of elements to replace in the set.
};

/**
 * @struct TreeModelParameters
 *
 * @brief A structure to hold the parameters related to a tree model in an experiment.
 */
struct TreeModelParameters {
	const std::string name; //< A model name; corresponds to one of the models in the `3d-models` directory.
	const double leaf_scale; //< The scaling factor for the leaves; 1.0 skips the rescaling.
	const std::variant<Unchanged, RandomSubset, Replace> fruit_subset; //< The subset of fruits to use in the experiment.
	const int seed; //< The seed for randomizing the tree model.
};

Json::Value toJson(const TreeModelParameters &treeModelParameters);

#endif //MGODPL_DECLARATIVEEXPERIMENTPARAMETERS_H
