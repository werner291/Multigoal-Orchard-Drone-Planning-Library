// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#ifndef MGODPL_SCAN_AWARE_LOCAL_OPTIMIZATION_H
#define MGODPL_SCAN_AWARE_LOCAL_OPTIMIZATION_H

#include <functional>
#include <optional>
#include <vector>

#include "../math/AABB.h"
#include "../math/Vec3.h"
#include "../planning/RobotPath.h"
#include "../planning/RobotState.h"
#include "../planning/scannable_points.h"
#include "fcl_forward_declarations.h"
#include "RobotModel.h"
#include "collision_detection.h"

namespace mgodpl {

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

	bool is_any_point_visible(const ScannablePoints &scannable_points, const math::Vec3d &ee_pos);

	bool is_any_point_visible(const std::vector<ScannablePoints> &scan_points, const math::Vec3d &ee_pos);

	bool
	is_any_new_point_visible(const std::vector<ScannablePoints> &scan_points,
							 const math::Vec3d &ee_pos,
							 std::vector<SeenPoints> &ever_seen);

	RobotPath optimize_scanpath(RobotPath path,
								const std::vector<ScannablePoints> &scan_points,
								const fcl::CollisionObjectd &tree_collision,
								const robot_model::RobotModel &robot_model,
								const std::optional<OptimizeScanpathHooks> &hooks = {});
};

#endif //MGODPL_SCAN_AWARE_LOCAL_OPTIMIZATION_H
