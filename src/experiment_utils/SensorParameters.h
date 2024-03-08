// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/8/24.
//

#ifndef MGODPL_SENSORPARAMETERS_H
#define MGODPL_SENSORPARAMETERS_H

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

Json::Value toJson(const SensorScalarParameters& sensorParameters) {
	Json::Value json;
	json["maxViewDistance"] = sensorParameters.maxViewDistance;
	json["minViewDistance"] = sensorParameters.minViewDistance;
	json["fieldOfViewAngle"] = sensorParameters.fieldOfViewAngle;
	json["maxScanAngle"] = sensorParameters.maxScanAngle;
	return json;
}

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

Json::Value toJson(const CircularOrbitParameters& orbitParameters) {
	Json::Value json;
	json["radius"] = orbitParameters.radius;
	json["inclination"] = orbitParameters.inclination;
	json["ascendingNodeLongitude"] = orbitParameters.ascendingNodeLongitude;
	return json;
}

/**
 * @struct SphericalOscillationParameters.
 */
struct SphericalOscillationParameters {
	double radius = 1.0; //< The radius of the oscillation, in radii of the tree canopy.
	double amplitude = 0.0; //< The amplitude of the oscillation, as a factor of PI/2 radius/
	unsigned int cycles = 1; //< The number of cycles of the oscillation.
};

Json::Value toJson(const SphericalOscillationParameters& oscillationParameters) {
	Json::Value json;
	json["radius"] = oscillationParameters.radius;
	json["amplitude"] = oscillationParameters.amplitude;
	json["cycles"] = oscillationParameters.cycles;
	return json;
}

struct OrbitPathParameters {
	const std::variant<CircularOrbitParameters, SphericalOscillationParameters> parameters; //< The parameters for the orbit path.
};

Json::Value toJson(const OrbitPathParameters& orbitPathParameters) {
	Json::Value json;

	if (auto circularOrbitParameters = std::get_if<CircularOrbitParameters>(&orbitPathParameters.parameters)) {
		json["type"] = "circular";
		json["parameters"] = toJson(*circularOrbitParameters);
	} else if (auto sphericalOscillationParameters = std::get_if<SphericalOscillationParameters>(&orbitPathParameters.parameters)) {
		json["type"] = "spherical";
		json["parameters"] = toJson(*sphericalOscillationParameters);
	}

	return json;
}

/// Use the original set of fruit.
struct Unchanged {};

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
};

Json::Value toJson(const TreeModelParameters& treeModelParameters) {
	Json::Value json;
	json["name"] = treeModelParameters.name;
	json["leaf_scale"] = treeModelParameters.leaf_scale;
	return json;
}

#endif //MGODPL_SENSORPARAMETERS_H
