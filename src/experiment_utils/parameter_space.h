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

	struct MetaParameters {
		/// The number of scenarios to evaluate.
		size_t n_repeat = 1;
		int seed = 42; //< The seed to use for random number generation.
	};

	Json::Value toJson(const MetaParameters &params);

	/**
	 * Generate a set of evaluation parameters to use for the point scanning experiment.
	 *
	 * TODO: maybe we can add some meta-parameters to control which set of parameters to generate?
	 */
	std::vector<PointScanEvalParameters> gen_eval_params(const MetaParameters &meta_params);
}

#endif //MGODPL_PARAMETER_SPACE_H
