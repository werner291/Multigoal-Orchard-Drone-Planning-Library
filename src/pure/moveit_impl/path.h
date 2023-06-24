// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#ifndef MGODPL_PATH_H
#define MGODPL_PATH_H

#include "../../RobotPath.h"

#include "../path.h"

template<>
RobotPath mgodpl::path_reverse(RobotPath path) {
	std::reverse(path.waypoints.begin(), path.waypoints.end());
	return path;
}

template<>
RobotPath mgodpl::path_concatenate(RobotPath paths...) {
	RobotPath result;

	for (const RobotPath& path : paths) {
		result.waypoints.insert(result.waypoints.end(), path.waypoints.begin(), path.waypoints.end());
	}

	return result;
}

template<>
struct mgodpl::path_configuration_t<RobotPath> {
	using type = moveit::core::RobotState;
};

namespace mgodpl {
	using RobotPath = ::RobotPath;
}

#endif //MGODPL_PATH_H
