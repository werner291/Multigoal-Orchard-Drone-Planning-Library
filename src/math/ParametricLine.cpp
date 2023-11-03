// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#include "ParametricLine.h"

mgodpl::math::ParametricLine::ParametricLine(const mgodpl::math::Vec3d &origin, const mgodpl::math::Vec3d &direction) : _origin(origin), _direction(direction) {}

mgodpl::math::Vec3d mgodpl::math::ParametricLine::closest_point(const mgodpl::math::Vec3d &p) const {
	return _origin + _direction * (p - _origin).dot(_direction);
}

mgodpl::math::Vec3d mgodpl::math::ParametricLine::pointAt(double d) const {
	return _origin + _direction * d;
}

mgodpl::math::ParametricLine
mgodpl::math::ParametricLine::through_points(const mgodpl::math::Vec3d &vec3, mgodpl::math::Vec3d vec31) {
	return ParametricLine(vec3, vec31 - vec3);
}
