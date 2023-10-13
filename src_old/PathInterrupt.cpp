// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-3-23.
//

#include <ostream>
#include "PathInterrupt.h"

std::ostream &PathInterrupt::operator<<(std::ostream &os) const {
	os << "PathInterrupt{segment_index=" << segment_index << ", to_next_waypoint_interpolation="
	   << to_next_waypoint_interpolation << "}";
	return os;
}