// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/10/23.
//

#include "grid_utils.h"

namespace mgodpl::grid_utils {

	using namespace math;

	std::array<Vec3i, 6> neighbors(const Vec3i &matrix) {

		return {
			matrix + Vec3i::UnitX(),
			matrix - Vec3i::UnitX(),
			matrix + Vec3i::UnitY(),
			matrix - Vec3i::UnitY(),
			matrix + Vec3i::UnitZ(),
			matrix - Vec3i::UnitZ()
		};

	}
}