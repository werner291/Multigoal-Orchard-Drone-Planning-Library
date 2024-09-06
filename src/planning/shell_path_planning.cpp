// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 9/5/24.
//

#include "shell_path_planning.h"

#include <fcl/narrowphase/collision.h>
#include "RobotState.h"
#include "RobotPath.h"
#include "RobotModel.h"
#include "ApproachPath.h"
#include "goal_sampling.h"
#include "collision_detection.h"
#include "probing_motions.h"
#include "approach_path_planning.h"
#include "shell_path.h"
#include "shell_path_assembly.h"
#include "visitation_order.h"
#include "nearest_neighbours/approach_by_pullout.h"
#include "shell_path_planning.h"

namespace mgodpl::shell_path_planning {

	mgodpl::RobotPath plan_multigoal_path(const RobotState &initial_state,
										  const fcl::CollisionObjectd &obstacle,
										  const cgal::CgalMeshData &chull_shell,
										  const std::vector<math::Vec3d> &target_points,
										  double distance_from_target,
										  const robot_model::RobotModel &robot,
										  random_numbers::RandomNumberGenerator &rng,
										  const PlanMultigoalPathMethods &methods,
										  const std::optional<std::function<RobotPath(const math::Vec3d target_point,
																					  const RobotState &state)>> &at_goal_hook,
										  const std::optional<PlanMultigoalPathHooks> &hooks) {

		// Plan a path from the initial state to the shell:
		ApproachPath initial_approach_path = approach_planning::plan_initial_approach_path(robot,
																						   initial_state,
																						   robot.findLinkByName(
																								   "flying_base"),
																						   chull_shell);

		// Plan an approach path for every target:
		std::vector<ApproachPath> approach_paths;
		std::vector<size_t> target_indices;

		for (size_t target_i = 0; target_i < target_points.size(); ++target_i) {
			auto approach_path = approach_planning::plan_approach_by_pullout(
					obstacle,
					chull_shell,
					target_points[target_i],
					distance_from_target,
					robot,
					rng,
					1000 // TODO: don't put a magic number here.
			);

			if (approach_path.has_value()) {
				approach_paths.push_back(approach_path.value());
				target_indices.push_back(target_i);
			}
		}

		// Build a distance matrix between the approach paths:

		// compute the shell distances:
		const auto &initial_state_distances = shell_distances(initial_approach_path.shell_point,
															  approach_paths,
															  chull_shell.convex_hull);
		const auto &target_to_target_distances = shell_distances(approach_paths, chull_shell);

		std::vector<size_t> order = methods.compute_visitation_order(target_to_target_distances,
																	 initial_state_distances);

		RobotPath finalPath;

		const ApproachPath *lastPath = &initial_approach_path;

		// We're going to do retreat-move-probe paths: backing away from one path, moving to the start of next, and then probing.
		for (size_t i = 0; i < approach_paths.size(); ++i) {
			const ApproachPath *nextPath = &approach_paths[order[i]];

			auto goalToGoal = retreat_move_probe(robot, chull_shell.convex_hull, *lastPath, *nextPath);

			finalPath.append(goalToGoal);

			// Check if the user wants to do something at this goal:
			if (at_goal_hook.has_value()) {
				auto hook = at_goal_hook.value();
				auto path = hook(target_points[target_indices[order[i]]], finalPath.states.back());
				finalPath.append(path);
			}

			lastPath = nextPath;
		}

		return finalPath;
	}

}