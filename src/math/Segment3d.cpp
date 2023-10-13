// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/13/23.
//

#include "Segment3d.h"

mgodpl::math::Segment3d::Segment3d(const mgodpl::math::Vec3d &a, const mgodpl::math::Vec3d &b) : a(a), b(b) {}

mgodpl::math::Vec3d mgodpl::math::Segment3d::closest_point(const mgodpl::math::Vec3d &p) const {
	return a + (b - a) * (p - a).dot(b - a) / (b - a).dot(b - a);
}

mgodpl::math::ParametricLine mgodpl::math::Segment3d::extend_to_line() const {
	return ParametricLine(a, b - a);
}
