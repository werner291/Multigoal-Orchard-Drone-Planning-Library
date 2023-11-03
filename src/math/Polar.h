// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#pragma once

#include "Vec3.h"

namespace mgodpl::math {

	struct Polar {
		double r;
		double azimuth;
		double altitude;
	};

	inline Polar pointToPolar(const Vec3d &point) {
		Polar polar{};
		polar.r = point.norm();
		polar.azimuth = std::atan2(point.y(), point.x());
		polar.altitude = std::asin(point.z() / polar.r);
		return polar;
	}
}