// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/20/23.
//

#include "Triangle.h"

namespace mgodpl::math {
	Triangle::Triangle(const Vec3d &a, const Vec3d &b, const Vec3d &c) : a(a), b(b), c(c) {
	}

	Vec3d Triangle::normal() const {
		return (b - a).cross(c - a).normalized();
	}

	double Triangle::area() const {
		return 0.5 * (b - a).cross(c - a).norm();
	}
}
