// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_PATHINTERRUPT_H
#define NEW_PLANNERS_PATHINTERRUPT_H

#include <cstddef>

struct PathInterrupt {
	size_t segment_index;
	double to_next_waypoint_interpolation; // 0.0 means at the first waypoint, 1.0 means at the second waypoint
};

#endif //NEW_PLANNERS_PATHINTERRUPT_H
