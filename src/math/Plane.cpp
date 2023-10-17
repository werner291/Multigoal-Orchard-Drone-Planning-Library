// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/16/23.
//

#include "Plane.h"

mgodpl::math::Plane
mgodpl::math::Plane::from_point_and_normal(const mgodpl::math::Vec3d &point, const mgodpl::math::Vec3d &normal) {
	return {normal, -normal.dot(point)};
}

mgodpl::math::Plane::Plane(const std::array<double, 4> &coefficients) : _normal(coefficients[0], coefficients[1], coefficients[2]), _d(coefficients[3]) {}

double mgodpl::math::Plane::signed_distance(const mgodpl::math::Vec3d &point) const {
	return _normal.dot(point) + _d;
}
