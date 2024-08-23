// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3/11/24.
//

#ifndef MGODPL_PARAMETER_SPACE_H
#define MGODPL_PARAMETER_SPACE_H

#include <cstddef>
#include <json/value.h>
#include "declarative_environment.h"

namespace mgodpl::declarative {

	/**
	 * Parameters for the point scanning experiment for declaring
	 * the problem parameter space over which to evaluate the planners.
	 */
	struct StaticPointScanMetaParameters {
		/// The number of scenarios to evaluate.
		size_t n_repeat = 1;
		std::vector<double> leaf_scales = {1.0};
		int seed = 42; //< The seed to use for random number generation.
		const std::vector<double> leaf_sizes; //< The leaf sizes to use for the trees.
	};

	Json::Value toJson(const StaticPointScanMetaParameters &params);

	/**
	 * Translate a StaticPointScanMetaParameters into a set of parameter combinations.
	 */
	std::vector<PointScanEvalParameters> gen_eval_params(const StaticPointScanMetaParameters &meta_params);
}

#endif //MGODPL_PARAMETER_SPACE_H
