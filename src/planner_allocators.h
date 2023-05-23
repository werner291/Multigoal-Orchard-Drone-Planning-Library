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
#include "planners/shell_path_planner/Construction.h"
#include "planners/ShellPathPlanner.h"
#include "planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "SimpleIncrementalTSPMethods.h"

using StaticPlannerPtr = std::shared_ptr<MultiGoalPlanner>;
using StaticPlannerAllocatorFn = std::function<StaticPlannerPtr(const ompl::base::SpaceInformationPtr &)>;

template <typename ShellPoint>
StaticPlannerAllocatorFn makeShellBasedPlanner(MkOmplShellFn<ShellPoint> shellBuilder) {
	return [shellBuilder](const ompl::base::SpaceInformationPtr& si) -> StaticPlannerPtr {
		auto planner = std::make_shared<ShellPathPlanner<ShellPoint>>(shellBuilder, std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si), true);
		return std::dynamic_pointer_cast<MultiGoalPlanner>(planner);
	};
}

using DMGPlannerPtr = std::shared_ptr<DynamicMultiGoalPlanner>;
using DMGPlannerAllocatorFn = std::function<DMGPlannerPtr(const ompl::base::SpaceInformationPtr &)>;

std::shared_ptr<OmplShellSpace<Eigen::Vector3d>> paddedOmplSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si);

std::shared_ptr<OmplShellSpace<Eigen::Vector3d>> omplSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_fre(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr static_planner(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr dynamic_planner_initial_orbit(const ompl::base::SpaceInformationPtr &si);

DMGPlannerPtr batch_replanner(const ompl::base::SpaceInformationPtr &si);

/**
 * Generates a dynamic planner allocator using the specified TSP strategy and a spherical shell.
 *
 * @param strategy The TSP strategy to be used.
 * @return A shared pointer to a DMGPlanner using the dynamic planner with the specified TSP strategy and a spherical shell.
 */
DMGPlannerAllocatorFn dynamic_planner_simple_reorder_sphere(SimpleIncrementalTSPMethods::Strategy strategy);

#endif //NEW_PLANNERS_PLANNER_ALLOCATORS_H
