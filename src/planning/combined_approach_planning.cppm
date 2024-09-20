module;
// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <functional>
#include <optional>
#include <vector>

#include "RobotState.h"
#include "RobotPath.h"

export module combined_approach_planning;

namespace mgodpl {

	using CollisionFn = std::function<bool(const RobotState &)>;
	using MotionCollisionFn = std::function<bool(const RobotState &, const RobotState &)>;

	export std::optional<RobotPath> plan_approach_path(
			RobotState nearest_shell_state,
			std::function<RobotState()> sample_goal_state,
			std::function<RobotState(const RobotState&)> project_goal_state,
			const CollisionFn state_collides,
			const MotionCollisionFn& motion_collides,
			const int max_goal_samples = 1000
			) {

		// Step 1: try a straight-in motion:
		{
			RobotState projected_shell_state = project_goal_state(nearest_shell_state);
			if (!motion_collides(nearest_shell_state, projected_shell_state)) {
				return RobotPath {
					.states = {nearest_shell_state, projected_shell_state}
				};
			}
		}

		for (int sample_i = 0; sample_i < max_goal_samples; ++sample_i) {

			RobotState goal_state = sample_goal_state();
			if (state_collides(goal_state)) {
				continue;
			}

			RobotMotion probing_motion = straightout_motion(goal_state);

			if (!motion_collides(probing_out.probing_motion.start, probing_out.probing_motion.end)) {
				return RobotPath {
					.states = {nearest_shell_state, probing_out.probing_motion.start, probing_out.probing_motion.end}
				};
			}

			// Now try RRT:
			rrt(
					sample,
					sample_state,
					collides,
					motion_collides,
					equal_weights_distance,
					1000,
					[&](const std::vector<RRTNode> &nodes) {
						auto side = inside(cgal::to_cgal_point(last_position));
						bool has_escaped = side == CGAL::ON_UNBOUNDED_SIDE;
						if (has_escaped) {
							std::cout << "Found a way out for apple " << apple << "!" << std::endl;
							path = retrace(nodes);
							return true;
						} else {
							return false;
						}
					}
				);

		}

	}

}