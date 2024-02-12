// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 2/12/24.
//

#include "shell_path_assembly.h"
#include "shell_path.h"

using namespace mgodpl;
using namespace mgodpl::shell_path_planning;

mgodpl::RobotPath mgodpl::shell_path_planning::retreat_move_probe(const mgodpl::robot_model::RobotModel &robot,
																  const mgodpl::cgal::Surface_mesh &convex_hull,
																  const mgodpl::ApproachPath &retreat_path,
																  const mgodpl::ApproachPath &probe_path) {

	RobotPath final_path;

	final_path.states.insert(final_path.states.end(),
							 retreat_path.path.states.rbegin(),
							 retreat_path.path.states.rend());

	auto move_path = shell_path(retreat_path.shell_point, probe_path.shell_point, convex_hull, robot);

	final_path.states.insert(final_path.states.end(), move_path.states.begin(), move_path.states.end());

	final_path.states.insert(final_path.states.end(),
							 probe_path.path.states.begin(),
							 probe_path.path.states.end());

	return final_path;

}

mgodpl::RobotPath mgodpl::shell_path_planning::assemble_final_path(const mgodpl::robot_model::RobotModel &robot,
																   const mgodpl::cgal::Surface_mesh &convex_hull,
																   const std::vector<ApproachPath> &approach_paths,
																   mgodpl::ApproachPath &initial_approach_path,
																   const std::vector<size_t> &order) {// Finally, assemble the final path.
	RobotPath final_path;

	{
		const ApproachPath *last_path = &initial_approach_path;

		// We're going to do retreat-move-probe paths: backing away from one path, moving to the start of next, and then probing.
		for (size_t i = 0; i < approach_paths.size(); ++i) {

			const ApproachPath *next_path = &approach_paths[order[i]];

			auto goal_to_goal = retreat_move_probe(robot, convex_hull, *last_path, *next_path);

			final_path.states.insert(final_path.states.end(),
									 goal_to_goal.states.begin(),
									 goal_to_goal.states.end());

			last_path = next_path;

		}
	}
	return final_path;
}
