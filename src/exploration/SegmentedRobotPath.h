// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_SEGMENTEDROBOTPATH_H
#define NEW_PLANNERS_SEGMENTEDROBOTPATH_H

#include <cstddef>
#include <moveit/robot_state/robot_state.h>
#include "../RobotPath.h"

/**
 * A variant of RobotPath that stores the path as a vector of segments, each of which is a RobotPath
 * typically because there is something special about the start/end of the segment that frequently
 * requires working in a segment-aware manner.
 */
class SegmentedRobotPath {

public:
	struct Index {
		size_t segment_index;
		size_t waypoint_index;

		bool operator==(const Index &other) const;

		bool operator!=(const Index &other) const;

		bool operator<(const Index &other) const;

		bool operator>(const Index &other) const;

		bool operator<=(const Index &other) const;

		bool operator>=(const Index &other) const;
	};

	Index next_waypoint_index(Index idx);

	Index prev_waypoint_index(Index idx);

	Index first_waypoint_index();

	Index last_waypoint_index();

	bool is_at_target(Index idx);

	moveit::core::RobotState &waypoint(Index idx);

	moveit::core::RobotState &first_waypoint() {
		return waypoint(first_waypoint_index());
	}

	void pop_first();

	bool empty() const;

	std::vector<RobotPath> segments;
};

#endif //NEW_PLANNERS_SEGMENTEDROBOTPATH_H
