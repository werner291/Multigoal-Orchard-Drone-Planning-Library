// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-3-23.
//

#ifndef NEW_PLANNERS_PLANNER_ALLOCATORS_H
#define NEW_PLANNERS_PLANNER_ALLOCATORS_H

#include "DynamicMultiGoalPlanner.h"
#include "shell_space/OmplShellSpace.h"

using DMGPlannerPtr = std::shared_ptr<DynamicMultiGoalPlanner>;
using DMGPlannerAllocatorFn = std::function<DMGPlannerPtr(const ompl::base::SpaceInformationPtr &)>;

std::shared_ptr<OmplShellSpace<Eigen::Vector3d>>
paddedOmplSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_fre(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr static_planner(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_initial_orbit(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr batch_replanner(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_lci(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_LIFO(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_FIFO(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_FISO(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_random(const ompl::base::SpaceInformationPtr &si);

#endif //NEW_PLANNERS_PLANNER_ALLOCATORS_H
