
#ifndef NEW_PLANNERS_RUN_EXPERIMENT_H
#define NEW_PLANNERS_RUN_EXPERIMENT_H

#include <functional>
#include <range/v3/view/iota.hpp>
#include "planners/MultiGoalPlanner.h"
#include "planning_scene_diff_message.h"
#include "ompl_custom.h"
#include "planners/ShellPathPlanner.h"

using NewMultiGoalPlannerAllocatorFn = std::function<std::shared_ptr<MultiGoalPlanner> (const AppleTreePlanningScene &, const ompl::base::SpaceInformationPtr &)>;

void
run_planner_experiment(const std::vector<NewMultiGoalPlannerAllocatorFn> &allocators,
					   const std::string &results_path,
					   const int num_runs,
					   const std::vector<size_t> &napples,
					   const std::vector<std::string> &scene_names,
					   const unsigned int nworkers,
					   bool groundPlane);

std::vector<NewMultiGoalPlannerAllocatorFn> make_tsp_over_prm_allocators(
		const std::vector<size_t>& samples_per_goal = {2,3,4,5,6,7,8,9},
		const std::vector<double>& plan_times_seconds = {1.0, 2.0, 5.0, 10.0, 15.0, 20.0},
		const std::vector<bool>& optimize_segments_options = {false, true}
		);

#endif //NEW_PLANNERS_RUN_EXPERIMENT_H
