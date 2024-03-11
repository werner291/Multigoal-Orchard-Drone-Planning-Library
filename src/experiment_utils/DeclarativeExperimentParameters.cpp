// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include <variant>
#include "DeclarativeExperimentParameters.h"

Json::Value toJson(const SensorScalarParameters &sensorParameters) {
	Json::Value json;
	json["maxViewDistance"] = sensorParameters.maxViewDistance;
	json["minViewDistance"] = sensorParameters.minViewDistance;
	json["fieldOfViewAngle"] = sensorParameters.fieldOfViewAngle;
	json["maxScanAngle"] = sensorParameters.maxScanAngle;
	return json;
}

Json::Value toJson(const SphericalOscillationParameters &oscillationParameters) {
	Json::Value json;
	json["radius"] = oscillationParameters.radius;
	json["amplitude"] = oscillationParameters.amplitude;
	json["cycles"] = oscillationParameters.cycles;
	return json;
}

Json::Value toJson(const OrbitPathParameters &orbitPathParameters) {
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

Json::Value toJson(const TreeModelParameters &treeModelParameters) {
	Json::Value json;
	json["name"] = treeModelParameters.name;
	json["leaf_scale"] = treeModelParameters.leaf_scale;
	if (auto unchanged = std::get_if<Unchanged>(&treeModelParameters.fruit_subset)) {
		json["fruit_subset"] = "unchanged";
	} else if (auto randomSubset = std::get_if<RandomSubset>(&treeModelParameters.fruit_subset)) {
		json["fruit_subset"] = "random_subset";
		json["count"] = randomSubset->count;
	} else if (auto replace = std::get_if<Replace>(&treeModelParameters.fruit_subset)) {
		json["fruit_subset"] = "replace";
		json["count"] = replace->count;
	}
	json["seed"] = treeModelParameters.seed;
	return json;
}

Json::Value toJson(const CircularOrbitParameters &orbitParameters) {
	Json::Value json;
	json["radius"] = orbitParameters.radius;
	json["inclination"] = orbitParameters.inclination;
	json["ascendingNodeLongitude"] = orbitParameters.ascendingNodeLongitude;
	return json;
}

mgodpl::ParametricPath mgodpl::instantiatePath(const OrbitPathParameters &orbit,
											   const mgodpl::math::Vec3d &tree_center,
											   const double canopy_radius) {

	// Go by the type of orbit
	if (auto circularOrbitParameters = std::get_if<CircularOrbitParameters>(&orbit.parameters)) {

		assert(circularOrbitParameters->inclination == 0.0); // We don't support inclined orbits yet.
		assert(circularOrbitParameters->ascendingNodeLongitude == 0.0); // We don't support inclined orbits yet.

		// Create a circular orbit
		return fixed_radius_equatorial_orbit(tree_center, circularOrbitParameters->radius * canopy_radius);

	} else if (auto sphericalOscillationParameters = std::get_if<SphericalOscillationParameters>(&orbit.parameters)) {

		// Create a spherical oscillation orbit
		return latitude_oscillation_path(tree_center,
										 sphericalOscillationParameters->radius * canopy_radius,
										 sphericalOscillationParameters->amplitude,
										 sphericalOscillationParameters->cycles);

	}

	// Should not reach here unless we forgot to implement a new orbit type
	throw std::runtime_error("Unimplemented orbit type");
}
