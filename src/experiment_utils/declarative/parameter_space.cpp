// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#include "parameter_space.h"

Json::Value mgodpl::declarative::toJson(const mgodpl::declarative::StaticPointScanMetaParameters &params) {
	Json::Value json;
	json["n_repeat"] = params.n_repeat;
	return json;
}

std::vector<mgodpl::declarative::PointScanEvalParameters>
mgodpl::declarative::gen_eval_params(const mgodpl::declarative::StaticPointScanMetaParameters &meta_params) {

	random_numbers::RandomNumberGenerator rng(meta_params.seed);

	// Define the sensor parameters
	SensorScalarParameters sensor_params {
			.maxViewDistance = INFINITY,
			.minViewDistance = 0.0,
			.fieldOfViewAngle = M_PI / 3.0,
			.maxScanAngle = M_PI / 3.0,
	};

	// Create a set of trees with different leaf scaling factors
	std::vector<TreeModelParameters> tree_params;

	for (const double leaf_scale: {0.0, 0.5, 1.0, 1.5, 2.0}) {
		tree_params.push_back(TreeModelParameters {
				.name = "appletree",
				.leaf_scale = leaf_scale,
				.fruit_subset = Unchanged {},
				.seed = rng.uniformInteger(0, std::numeric_limits<int>::max())
		});
	}

	// Take the cartesian product of the tree and sensor parameters
	std::vector<PointScanEvalParameters> eval_params;

	for (const auto &tree_param: tree_params) {
		eval_params.push_back({
									  .tree_params = tree_param,
									  .sensor_params = sensor_params
							  });
	}

	// Return the generated parameters
	return eval_params;
}
