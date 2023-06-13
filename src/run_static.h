// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 15-5-23.
//

#ifndef NEW_PLANNERS_RUN_STATIC_H
#define NEW_PLANNERS_RUN_STATIC_H

#include <json/value.h>
#include "planner_allocators.h"
#include "static_problem_generation.h"

Json::Value runPlannerOnStaticProblem(const StaticPlannerAllocatorFn &planner,
									  const Problem &problem);

#endif //NEW_PLANNERS_RUN_STATIC_H
