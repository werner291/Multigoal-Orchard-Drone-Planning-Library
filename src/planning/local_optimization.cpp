// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 4/9/24.
//

#include "local_optimization.h"

namespace mgodpl {

	bool tryShortcutByDeletingWaypoint(const mgodpl::robot_model::RobotModel &robot,
											   mgodpl::RobotPath &path,
											   size_t waypointIndex,
											   const fcl::CollisionObjectd &obstacle) {

		assert(waypointIndex > 0 && waypointIndex + 1 < path.states.size());

		// Get the three states affected:
		const auto &prev = path.states[waypointIndex - 1];
		const auto &current = path.states[waypointIndex];
		const auto &next = path.states[waypointIndex + 1];

		double toi;

		// Check if the motion between prev and next is collision-free:
		bool collides = mgodpl::check_motion_collides(
				robot,
				obstacle,
				prev,
				next,
				toi
		);

		if (collides) {
			return false;
		}

		// If the motion is collision-free, remove the waypoint:
		path.states.erase(path.states.begin() + waypointIndex);

		return true;

	}

	bool tryMidpointPull(const mgodpl::robot_model::RobotModel &robot,
								 mgodpl::RobotPath &path,
								 size_t waypointIndex,
								 double pull_factor,
								 const fcl::CollisionObjectd &obstacle) {

		assert(waypointIndex > 0 && waypointIndex + 1 < path.states.size());

		// Get the three states affected:
		const auto &prev = path.states[waypointIndex - 1];
		const auto &current = path.states[waypointIndex];
		const auto &next = path.states[waypointIndex + 1];

		// Compute the midpoint:
		auto midpoint = interpolate(prev, next, 0.5);

		// Pull the waypoint towards the midpoint:
		auto new_state = interpolate(current, midpoint, pull_factor);

		// Check collision-freeness on the new two motions that go by the pulled waypoint:
		double toi1, toi2;

		bool collides1 = mgodpl::check_motion_collides(
				robot,
				obstacle,
				prev,
				new_state,
				toi1
		);

		bool collides2 = mgodpl::check_motion_collides(
				robot,
				obstacle,
				new_state,
				next,
				toi2
		);

		if (collides1 || collides2) {
			return false;
		}

		// If the motion is collision-free, update the waypoint:
		path.states[waypointIndex] = new_state;

		return true;
	}

	bool tryShortcutBetweenPathPoints(const mgodpl::robot_model::RobotModel &robot,
											  mgodpl::RobotPath &path,
											  const mgodpl::PathPoint &start,
											  const mgodpl::PathPoint &end,
											  const fcl::CollisionObjectd &obstacle) {

		RobotState st1 = interpolate(start, path);
		RobotState st2 = interpolate(end, path);

		double toi;
		bool collides = check_motion_collides(robot, obstacle, st1, st2, toi);

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

	bool tryDeletingEveryWaypoint(const mgodpl::robot_model::RobotModel &robot,
										  mgodpl::RobotPath &path,
										  const fcl::CollisionObjectd &obstacle) {

		size_t cursor = 1;

		bool shortened = false;

		while (cursor + 1 < path.states.size()) {
			if (tryShortcutByDeletingWaypoint(robot, path, cursor, obstacle)) {
				shortened = true;
			} else {
				cursor++;
			}
		}

		return shortened;

	}

	bool tryShortcuttingRandomly(const mgodpl::robot_model::RobotModel &robot,
										 mgodpl::RobotPath &path,
										 const fcl::CollisionObjectd &obstacle,
										 random_numbers::RandomNumberGenerator &rng) {

		// Generate a random path point.
		PathPoint middle = generateRandomPathPoint(path, rng);

		// Radius less than 0.5 might not span a waypoint; no use trying to shortcut a straight line.
		double radius = rng.uniformReal(0.5, 2.0);

		// Generate two points on opposite sides of the middle point, distance radius.
		PathPoint start = middle.adjustByScalar(-radius, path);
		PathPoint end = middle.adjustByScalar(radius, path);

		// Try to shortcut between the two points.
		return tryShortcutBetweenPathPoints(robot, path, start, end, obstacle);
	}

}
