// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#include "grid_utils.h"

std::array<Eigen::Vector3i, 6> mgodpl::grid_utils::neighbors(const Eigen::Vector3i &matrix) {

	return {
			matrix + Eigen::Vector3i::UnitX(),
			matrix - Eigen::Vector3i::UnitX(),
			matrix + Eigen::Vector3i::UnitY(),
			matrix - Eigen::Vector3i::UnitY(),
			matrix + Eigen::Vector3i::UnitZ(),
			matrix - Eigen::Vector3i::UnitZ()
	};

}
