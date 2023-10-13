// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#ifndef MGODPL_PATH_H
#define MGODPL_PATH_H

#include <ompl/geometric/PathGeometric.h>
#include "../path.h"

template<>
ompl::geometric::PathGeometric mgodpl::path_reverse(ompl::geometric::PathGeometric path) {
	path.reverse();
	return path;
}

template<>
ompl::geometric::PathGeometric mgodpl::path_concatenate(ompl::geometric::PathGeometric paths...) {
	ompl::geometric::PathGeometric result;

	for (const ompl::geometric::PathGeometric& path : paths) {
		result.append(path);
	}

	return result;
}

#endif //MGODPL_PATH_H
