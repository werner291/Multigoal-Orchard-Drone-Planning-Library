// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/10/24.
//

#ifndef MGODPL_LOCAL_OPTIMIZATION_H
#define MGODPL_LOCAL_OPTIMIZATION_H

#include <json/value.h>
#include <variant>

namespace mgodpl::declarative {

	struct RandomSpanShortcutting {
		double max_radius = 2.0;
		unsigned int n_attempts = 0;
	};

	struct MidpointPull {
		double max_pull_factor = 1.0;
		unsigned int n_attempts = 0;
	};

	struct WaypointDeletion {
	};

	using LocalOptimizationConfig = std::variant<RandomSpanShortcutting, MidpointPull, WaypointDeletion>;

	Json::Value to_json(const RandomSpanShortcutting& config);

	Json::Value to_json(const MidpointPull& config);

	Json::Value to_json(const WaypointDeletion& config);

	Json::Value to_json(const LocalOptimizationConfig& config);

}

#endif //MGODPL_LOCAL_OPTIMIZATION_H
