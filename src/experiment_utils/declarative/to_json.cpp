// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/12/24.
//

#include <json/value.h>
#include "SensorModelParameters.h"
#include "SolutionMethod.h"
#include "../tree_models.h"

namespace mgodpl::declarative {

	Json::Value toJson(const SensorScalarParameters &sensorParameters) {
		Json::Value json;
		json["maxViewDistance"] = sensorParameters.maxViewDistance;
		json["minViewDistance"] = sensorParameters.minViewDistance;
		json["fieldOfViewAngle"] = sensorParameters.fieldOfViewAngle;
		json["maxScanAngle"] = sensorParameters.maxScanAngle;
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

	Json::Value toJson(const OrbitFacingTree &orbit) {
		Json::Value json;
		json["type"] = "orbit";
		json["parameters"] = toJson(orbit.params);
		return json;
	}

	Json::Value toJson(const ProbingMotionsMethod &_method) {
		Json::Value json;
		json["type"] = "probing";
		return json;
	}

	Json::Value toJson(const SolutionMethod &method) {
		return std::visit([](const auto &m) { return toJson(m); }, method);
	}

}
