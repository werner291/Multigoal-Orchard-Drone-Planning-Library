// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/12/24.
//

#include <json/value.h>
#include "SensorModelParameters.h"
#include "../tree_models.h"
#include "local_optimization.h"

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

	Json::Value to_json(const RandomSpanShortcutting& config) {
		Json::Value json;
		json["max_radius"] = config.max_radius;
		json["n_attempts"] = config.n_attempts;
		return json;
	}

	Json::Value to_json(const MidpointPull& config) {
		Json::Value json;
		json["max_pull_factor"] = config.max_pull_factor;
		json["n_attempts"] = config.n_attempts;
		return json;
	}

	Json::Value to_json(const WaypointDeletion& config) {
		Json::Value json;
		return json;
	}

	Json::Value to_json(const LocalOptimizationConfig& config) {
		Json::Value json = std::visit([](const auto &c) { return to_json(c); }, config);

		if (std::holds_alternative<RandomSpanShortcutting>(config)) {
			json["type"] = "random_span_shortcutting";
		} else if (std::holds_alternative<MidpointPull>(config)) {
			json["type"] = "midpoint_pull";
		} else if (std::holds_alternative<WaypointDeletion>(config)) {
			json["type"] = "waypoint_deletion";
		} else {
			throw std::runtime_error("Unknown local optimization config type");
		}

		return json;
	}

}
