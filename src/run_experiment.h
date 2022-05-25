//
// Created by werner on 18-5-22.
//

#ifndef NEW_PLANNERS_RUN_EXPERIMENT_H
#define NEW_PLANNERS_RUN_EXPERIMENT_H

#include <functional>
#include "NewMultiGoalPlanner.h"
#include "planning_scene_diff_message.h"
#include "ompl_custom.h"

typedef std::function<std::shared_ptr<NewMultiGoalPlanner>(
        const AppleTreePlanningScene& scene_info,
        const ompl::base::SpaceInformationPtr&)>
        NewMultiGoalPlannerAllocatorFn;

void run_planner_experiment(const std::vector <NewMultiGoalPlannerAllocatorFn> &allocators,
                            const std::string &results_path,
                            const int num_runs);

#endif //NEW_PLANNERS_RUN_EXPERIMENT_H
