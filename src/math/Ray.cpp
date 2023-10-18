// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/18/23.
//

#include "Ray.h"

namespace mgodpl::math {
	Ray Ray::advanced(double d) const {
		return {line.pointAt(d), line.direction()};
	}

	Vec3d Ray::pointAt(double aDouble) const {
		return line.pointAt(aDouble);
	}
}
