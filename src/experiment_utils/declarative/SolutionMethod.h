// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/8/24.
//

#ifndef MGODPL_SOLUTIONMETHOD_H
#define MGODPL_SOLUTIONMETHOD_H

#include <json/value.h>
#include <variant>
#include "../parametric_paths.h"

namespace mgodpl::declarative {

	struct OrbitFacingTree {
		OrbitPathParameters params;
	};

	struct ProbingMotionsMethod {

	};

	using SolutionMethod = std::variant<OrbitFacingTree, ProbingMotionsMethod>;

	Json::Value toJson(const OrbitFacingTree &orbit);

	Json::Value toJson(const ProbingMotionsMethod &_method);

	Json::Value toJson(const SolutionMethod &method);
}

#endif //MGODPL_SOLUTIONMETHOD_H
