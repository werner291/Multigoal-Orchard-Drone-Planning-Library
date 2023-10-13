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

const mgodpl::math::Vec3d &mgodpl::math::ParametricLine::direction() const {
	return _direction;
}

const mgodpl::math::Vec3d &mgodpl::math::ParametricLine::origin() const {
	return _origin;
}
