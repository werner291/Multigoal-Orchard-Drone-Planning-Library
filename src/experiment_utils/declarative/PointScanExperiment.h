// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/12/24.
//

#ifndef MGODPL_POINTSCANEXPERIMENT_H
#define MGODPL_POINTSCANEXPERIMENT_H

#include <cstddef>
#include <json/value.h>
#include "../tree_models.h"
#include "SensorModelParameters.h"

namespace mgodpl::declarative {

	/**
	 * The environmental parameters for a given point scanning experiment, assuming a static environment.
	 *
	 * This serves as a POD specification for the environment and objective of the experiment,
	 * and us meant to be stored alongside the results in order to reproduce them and understand
	 * the impact of environmental parameters on the results.
	 */
	struct PointScanEvalParameters {

		/// Parameters about the tree model, including which model to use and the number of fruits to scan.
		TreeModelParameters tree_params;

		/// Parameters about the sensor, including the field of view and the maximum view distance.
		SensorScalarParameters sensor_params;

		/// How many points to use per fruit. (TODO: this is more an implementation detail, does it belong here?)
		size_t n_scannable_points_per_fruit = 200;

	};

	Json::Value toJson(const PointScanEvalParameters &params);
}

#endif //MGODPL_POINTSCANEXPERIMENT_H
