// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 11/23/23.
//

#include <Eigen/Geometry>
#include "Quaternion.h"

mgodpl::math::Quaterniond mgodpl::math::slerp(const mgodpl::math::Quaterniond &a, const mgodpl::math::Quaterniond &b, double t) {

	Eigen::Quaterniond ea(a.w, a.x, a.y, a.z);
	Eigen::Quaterniond eb(b.w, b.x, b.y, b.z);

	Eigen::Quaterniond ec = ea.slerp(t, eb);

	return {.x=ec.x(), .y=ec.y(), .z=ec.z(), .w=ec.w()};

}
