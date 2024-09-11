// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10-9-24.
//

module;

#include "../experiment_utils/surface_points.h"
#include "local_optimization.h"

export module scan_aware_local_optimization;

export namespace mgodpl {

	/**
	 * \struct OptimizeScanpathHooks
	 * \brief A structure to hold various hooks for the optimize_scanpath function.
	 *
	 * This structure contains several std::function members that can be used as hooks
	 * to monitor and interact with the scan path optimization process.
	 */
	struct OptimizeScanpathHooks {
		/**
		 * \brief Hook called when AABBs for clusters are computed.
		 * \param aabbs A vector of computed AABBs.
		 */
		std::function<void(const std::vector<math::AABBd> &)> computed_aabbs;

		/**
		 * \brief Hook called at the beginning of mapping states to scan points.
		 */
		std::function<void()> begin_mapping_states_to_scan_points;

		/**
		 * \brief Hook called at the beginning of mapping a specific state to scan points.
		 * \param state The current robot state being mapped.
		 */
		std::function<void(const RobotState &)> begin_mapping_state_to_scan_points;

		/**
		 * \brief Hook called at the beginning of mapping a state to a specific scan point cluster.
		 * \param cluster_index The index of the scan point cluster.
		 */
		std::function<void(size_t)> begin_mapping_state_to_scan_point_cluster;

		/**
		 * \brief Hook called when a state scans a specific point.
		 * \param state_index The index of the state.
		 * \param point_index The index of the scan point.
		 */
		std::function<void(size_t, size_t)> state_scans_point;

		/**
		 * \brief Hook called when a state is outside the AABB.
		 * \param state_index The index of the state.
		 */
		std::function<void(size_t)> state_outside_aabb;

		/**
		 * \brief Hook called at the end of mapping a state to a specific scan point cluster.
		 * \param cluster_index The index of the scan point cluster.
		 */
		std::function<void(size_t)> end_mapping_state_to_scan_point_cluster;

		/**
		 * \brief Hook called at the end of mapping states to scan points.
		 */
		std::function<void(std::vector<bool>)> end_mapping_states_to_scan_points;

		/**
		 * \brief Hook called when a waypoint will be deleted.
		 * \param waypoint_index The index of the waypoint.
		 * \param prev The previous state.
		 * \param current The current state.
		 * \param next The next state.
		 */
		std::function<void(size_t, const RobotState &, const RobotState &, const RobotState &)> will_delete_waypoint;

		/**
		 * \brief Hook called at the beginning of deleting unassociated waypoints.
		 */
		std::function<void()> begin_deleting_unassociated_waypoints;

		/**
		 * \brief Hook called at the end of deleting unassociated waypoints.
		 */
		std::function<void(const RobotPath &)> end_deleting_unassociated_waypoints;

		/**
		 * \brief Hook called at the beginning of deleting associated waypoints.
		 */
		std::function<void()> begin_deleting_associated_waypoints;

		/**
		 * \brief Hook called at the end of deleting associated waypoints.
		 */
		std::function<void(const RobotPath &)> end_deleting_associated_waypoints;

		/**
		 * \brief Hook called when the shortcutting process ends.
		 */
		std::function<void(const RobotPath &)> end_shortcutting;
	};
	bool is_any_point_visible(const mgodpl::ScannablePoints &scannable_points, const mgodpl::math::Vec3d &ee_pos) {
		for (size_t point_index = 0; point_index < scannable_points.surface_points.size(); point_index++) {
			if (is_visible(scannable_points, point_index, ee_pos)) {
				return true;
			}
		}
		return false;
	}

	bool is_any_point_visible(const std::vector<ScannablePoints> &scan_points, const math::Vec3d &ee_pos) {
		for (const auto &scannable_points: scan_points) {
			if (is_any_point_visible(scannable_points, ee_pos)) {
				return true;
			}
		}
		return false;
	}

	bool is_any_new_point_visible(const std::vector<ScannablePoints> &scan_points,
								  const math::Vec3d &ee_pos,
								  std::vector<SeenPoints> &ever_seen) {
		size_t newly_seen = 0;
		for (size_t i = 0; i < scan_points.size(); i++) {
			const auto &scannable_points = scan_points[i];
			newly_seen += update_visibility(scannable_points, ee_pos, ever_seen[i]);
		}
		return newly_seen;
	}

	RobotPath optimize_scanpath(RobotPath path,
								const std::vector<ScannablePoints> &scan_points,
								const fcl::CollisionObjectd &tree_collision,
								const robot_model::RobotModel &robot_model,
								const std::optional<OptimizeScanpathHooks> &hooks) {

		// The path is trivial and cannot be optimized:
		if (path.n_waypoints() < 3) {
			return path;
		}

		const auto &aabbs = computeAABBsForClusters(scan_points);

		if (hooks) hooks->computed_aabbs(aabbs);

		if (hooks) hooks->begin_mapping_states_to_scan_points();

		// Step 1: for every waypoint in the path, check a pool that says whether it scans a point or not:
		std::vector<bool> scans_point(path.n_waypoints(), false);

		// TODO: update_visibility offers an interface to see if NEW points are visible, as opposed ti just any points.
		std::vector<bool> scans_new_point(path.n_waypoints(), false);

		std::vector<SeenPoints> ever_seen;
		ever_seen.reserve(scan_points.size());
		for (const auto &scannable_points: scan_points) {
			ever_seen.push_back(SeenPoints::create_all_unseen(scannable_points));
		}

		// Step two: for every waypoint in the path, check if it is associated with any point in the scan points:
		for (size_t i = 0; i < path.n_waypoints(); i++) {

			const auto &state = path.waypoint(i);
			const auto &fk = forwardKinematics(robot_model, state);
			const auto &ee_pos = fk.forLink(robot_model.findLinkByName("end_effector")).translation;

			if (hooks) hooks->begin_mapping_state_to_scan_points(state);

			scans_point[i] = is_any_point_visible(scan_points, ee_pos);
			scans_new_point[i] = is_any_new_point_visible(scan_points, ee_pos, ever_seen);
		}

		if (hooks) hooks->end_mapping_states_to_scan_points(scans_point);

		const std::function motion_collides = [&](const RobotState &a, const RobotState &b) {
			return check_motion_collides(robot_model, tree_collision, a, b);
		};

		if (hooks) hooks->begin_deleting_unassociated_waypoints();

		// Step 3: for every waypoint that doesn't scan a point, try to delete it:
		for (size_t wp_i = 1; wp_i + 1 < path.n_waypoints(); wp_i++) {
			if (!scans_point[wp_i]) {

				// Get the three states affected:
				const auto &prev = path.states[wp_i - 1];
				const auto &current = path.states[wp_i];
				const auto &next = path.states[wp_i + 1];

				if (!motion_collides(prev, next)) {

					if (hooks) hooks->will_delete_waypoint(wp_i, prev, current, next);

					// If the motion is collision-free, remove the waypoint:
					path.states.erase(path.states.begin() + static_cast<long>(wp_i));
					// Delete the entry in scans_point:
					scans_point.erase(scans_point.begin() + static_cast<long>(wp_i));

					wp_i -= 1; // Since we just deleted the waypoint, we need to recheck the current index.
				}
			}
		}

		if (hooks) hooks->end_deleting_unassociated_waypoints(path);

		// Step 4: for every waypoint that doesn't scan a new point, try to delete it:
		if (hooks) hooks->begin_deleting_associated_waypoints();

		for (size_t wp_i = 1; wp_i + 1 < path.n_waypoints(); wp_i++) {
			if (!scans_new_point[wp_i]) {

				// Get the three states affected:
				const auto &prev = path.states[wp_i - 1];
				const auto &current = path.states[wp_i];
				const auto &next = path.states[wp_i + 1];

				if (!motion_collides(prev, next)) {

					if (hooks) hooks->will_delete_waypoint(wp_i, prev, current, next);

					// If the motion is collision-free, remove the waypoint:
					path.states.erase(path.states.begin() + static_cast<long>(wp_i));
					// Delete the entry in scans_point:
					scans_point.erase(scans_point.begin() + static_cast<long>(wp_i));
					// Delete the entry in scans_new_point:
					scans_new_point.erase(scans_new_point.begin() + static_cast<long>(wp_i));

					wp_i -= 1; // Since we just deleted the waypoint, we need to recheck the current index.
				}
			}
		}

		if (hooks) hooks->end_deleting_associated_waypoints(path);

		// Step 5: try shortcutting between the midpoints of every segment:
		for (size_t wp_i = path.n_waypoints() - 1; wp_i > 0; wp_i--) {
			// ^ Working backwards so we don't invalidate indices.

			// Skip points where we already scan a point:
			if (scans_point[wp_i]) {
				continue;
			}

			tryShortcutBetweenPathPoints(
					path,
					{.segment_i=wp_i - 1, .segment_t=0.5},
					{.segment_i=wp_i, .segment_t=0.5},
					motion_collides
			);
		}

		if (hooks) hooks->end_shortcutting(path);

		return path;
	}

}