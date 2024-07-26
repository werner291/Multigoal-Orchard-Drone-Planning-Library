// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/9/24.
//

#include "local_optimization.h"

namespace mgodpl {
	/**
	 * Try to shorten a path by deleting the waypoint at the given index. This will result in moving directly from
	 * the waypoint before the index to the waypoint after the index.
	 *
	 * The waypoint to be deleted should not be the first or the last waypoint. (Checked by assertion in debug builds.)
	 *
	 * @param path						The path to shorten.
	 * @param waypointIndex				The index of the waypoint to delete.
	 * @param check_motion_collides		A function that checks if a motion between two states collides.
	 * @return							True if the path was successfully shortened, false otherwise.
	 */
	bool tryShortcutByDeletingWaypoint(
		RobotPath &path,
		size_t waypointIndex,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	) {
		// The waypoint to be deleted should not be the first or the last waypoint.
		assert(waypointIndex > 0 && waypointIndex + 1 < path.states.size());

		// Get the three states affected:
		const auto &prev = path.states[waypointIndex - 1];
		const auto &current = path.states[waypointIndex];
		const auto &next = path.states[waypointIndex + 1];

		if (check_motion_collides(prev, next)) {
			return false;
		}

		// If the motion is collision-free, remove the waypoint:
		path.states.erase(path.states.begin() + static_cast<long>(waypointIndex));

		return true;
	}

	bool tryMidpointPull(
		RobotPath &path,
		size_t waypointIndex,
		double pull_factor,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	) {
		assert(waypointIndex > 0 && waypointIndex + 1 < path.states.size());

		// Get the three states affected:
		const auto &prev = path.states[waypointIndex - 1];
		const auto &current = path.states[waypointIndex];
		const auto &next = path.states[waypointIndex + 1];

		// Compute the midpoint:
		auto midpoint = interpolate(prev, next, 0.5);

		// Pull the waypoint towards the midpoint:
		auto new_state = interpolate(current, midpoint, pull_factor);

		// Check if the motion between prev and next is collision-free. If not, abort.
		if (check_motion_collides(current, new_state) || check_motion_collides(new_state, next)) {
			return false;
		}

		// If the motion is collision-free, update the waypoint:
		path.states[waypointIndex] = new_state;

		return true;
	}

	bool tryShortcutBetweenPathPoints(
		RobotPath &path,
		const PathPoint &start,
		const PathPoint &end,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	) {
		// Start should be before end.
		assert(start < end);

		RobotState st1 = interpolate(start, path);
		RobotState st2 = interpolate(end, path);

		bool collides = check_motion_collides(st1, st2);

		if (collides) {
			return false;
		}

		std::vector<RobotState> new_states;

		// Every state before the start state
		new_states.insert(new_states.end(), path.states.begin(), path.states.begin() + start.segment_i);

		// Insert the start state and the end state
		new_states.push_back(st1);
		new_states.push_back(st2);

		// Every state after the end state
		new_states.insert(new_states.end(), path.states.begin() + end.segment_i + 1, path.states.end());

		path.states = std::move(new_states);

		return true; // Shortcut successful.
	}

	PathPoint generateRandomPathPoint(const RobotPath &path, random_numbers::RandomNumberGenerator &rng) {
		size_t segment_i = rng.uniformInteger(0, path.states.size() - 2);
		double segment_t = rng.uniformReal(0.0, 1.0);
		return PathPoint{segment_i, segment_t};
	}

	bool tryDeletingEveryWaypoint(
		RobotPath &path,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	) {
		size_t cursor = 1;

		bool shortened = false;

		while (cursor + 1 < path.states.size()) {
			if (tryShortcutByDeletingWaypoint(path, cursor, check_motion_collides)) {
				shortened = true;
			} else {
				cursor++;
			}
		}

		return shortened;
	}

	bool tryShortcuttingRandomlyLocally(
		RobotPath &path,
		random_numbers::RandomNumberGenerator &rng,
		const std::function<bool(const RobotState &, const RobotState &)> &check_motion_collides
	) {
		// Generate a random path point.
		PathPoint middle = generateRandomPathPoint(path, rng);

		// Radius less than 0.5 might not span a waypoint; no use trying to shortcut a straight line.
		double radius = rng.uniformReal(0.5, 2.0);

		// Generate two points on opposite sides of the middle point, distance radius.
		PathPoint start = middle.adjustByScalar(-radius, path);
		PathPoint end = middle.adjustByScalar(radius, path);

		// Try to shortcut between the two points.
		return tryShortcutBetweenPathPoints(path, start, end, check_motion_collides);
	}

	bool tryShortcuttingRandomlyGlobally(RobotPath &path,
	                                     const std::function<bool(const RobotState &, const RobotState &)> &
	                                     check_motion,
	                                     random_numbers::RandomNumberGenerator &rng) {
		// Pick two random path points.
		PathPoint start = generateRandomPathPoint(path, rng);
		PathPoint end = generateRandomPathPoint(path, rng);

		// Swap to make sure start is before end.
		if (start > end) {
			std::swap(start, end);
		}

		// Try shortcutting between the two points.
		return tryShortcutBetweenPathPoints(path, start, end, check_motion);
	}
}
