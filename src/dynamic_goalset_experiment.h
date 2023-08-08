// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#ifndef NEW_PLANNERS_DYNAMIC_GOALSET_EXPERIMENT_H
#define NEW_PLANNERS_DYNAMIC_GOALSET_EXPERIMENT_H

#include <functional>
#include <memory>
#include "DynamicMultiGoalPlanner.h"
#include "DynamicGoalsetPlanningProblem.h"

using DynamicPlannerAllocatorFn = std::function<std::shared_ptr<DynamicMultiGoalPlanner>(const ompl::base::SpaceInformationPtr &)>;

struct Experiment {
	std::pair<std::string, DynamicPlannerAllocatorFn> *planner;
	std::pair<Json::Value, DynamicGoalsetPlanningProblem> *problem;
};

Json::Value toJSON(const Experiment &experiment);

Json::Value
runDynamicPlannerExperiment(const moveit::core::RobotModelPtr &robot, const Experiment &experiment, bool saveSegments);

#endif //NEW_PLANNERS_DYNAMIC_GOALSET_EXPERIMENT_H
