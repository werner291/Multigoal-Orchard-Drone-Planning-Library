// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include <variant>
#include "SensorParameters.h"

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
