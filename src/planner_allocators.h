// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7-3-23.
//

#ifndef NEW_PLANNERS_PLANNER_ALLOCATORS_H
#define NEW_PLANNERS_PLANNER_ALLOCATORS_H


#include "DynamicMultiGoalPlanner.h"
#include "ORToolsTSPMethods.h"
#include "SimpleIncrementalTSPMethods.h"
#include "planner_allocators.h"
#include "planners/CachingDynamicPlanner.h"
#include "planners/ChangeAccumulatingPlannerAdapter.h"
#include "planners/ChangeIgnoringReplannerAdapter.h"
#include "planners/InitialOrbitPlanner.h"
#include "planners/ShellPathPlanner.h"
#include "planners/shell_path_planner/Construction.h"
#include "planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "shell_space/OmplShellSpace.h"
#include "shell_space/SphereShell.h"

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

/**
 * @brief Dynamic planner with forward reachability estimation.
 *
 * @tparam ShellPoint Data type of the shell points.
 * @param paddedOmplSphereShell Function for constructing a OmplShellSpace.
 * @return DMGPlannerAllocatorFn
 */
template <typename ShellPoint>
DMGPlannerAllocatorFn dynamic_planner_fre(MkOmplShellFn<ShellPoint> paddedOmplSphereShell) {
	return [paddedOmplSphereShell](const ompl::base::SpaceInformationPtr &si) -> DMGPlannerPtr {
		return std::make_shared<CachingDynamicPlanner<ShellPoint>>(
				std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si),
				std::make_shared<ORToolsTSPMethods>(),
				paddedOmplSphereShell
		);
	};
};

/**
 * @brief Static planner.
 *
 * @tparam ShellPoint Data type of the shell points.
 * @param paddedOmplSphereShell Function for constructing a OmplShellSpace.
 * @return DMGPlannerAllocatorFn
 */
template <typename ShellPoint>
DMGPlannerAllocatorFn static_planner(MkOmplShellFn<ShellPoint> paddedOmplSphereShell) {
	return [paddedOmplSphereShell](const ompl::base::SpaceInformationPtr &si) -> DMGPlannerPtr {
		return std::make_shared<ChangeIgnoringReplannerAdapter>(
				std::make_shared<ShellPathPlanner<ShellPoint>>(
						paddedOmplSphereShell,
						std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint >>(si),
						true
				)
		);
	};
};

/**
 * @brief Dynamic planner with initial orbit planning.
 *
 * @tparam ShellPoint Data type of the shell points.
 * @param paddedOmplSphereShell Function for constructing a OmplShellSpace.
 * @return DMGPlannerAllocatorFn
 */
template <typename ShellPoint>
DMGPlannerAllocatorFn dynamic_planner_initial_orbit(MkOmplShellFn<ShellPoint> paddedOmplSphereShell) {
	return [paddedOmplSphereShell](const ompl::base::SpaceInformationPtr &si) -> DMGPlannerPtr {
		return std::make_shared<InitialOrbitPlanner>(dynamic_planner_fre(paddedOmplSphereShell)(si));
	};
};

/**
 * @brief Batch replanner.
 *
 * @tparam ShellPoint Data type of the shell points.
 * @param paddedOmplSphereShell Function for constructing a OmplShellSpace.
 * @return DMGPlannerAllocatorFn
 */
template <typename ShellPoint>
DMGPlannerAllocatorFn batch_replanner(MkOmplShellFn<ShellPoint> paddedOmplSphereShell) {
	return [paddedOmplSphereShell](const ompl::base::SpaceInformationPtr &si) -> DMGPlannerPtr {
		return std::make_shared<ChangeAccumulatingPlannerAdapter>(
				std::make_shared<ShellPathPlanner<ShellPoint>>(
						paddedOmplSphereShell,
						std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint >>(si),
						true
				)
		);
	};
};

/**
 * @brief Dynamic planner with simple reordering on a sphere.
 *
 * @tparam ShellPoint Data type of the shell points.
 * @param strategy The reordering strategy to use.
 * @param paddedOmplSphereShell Function for constructing a OmplShellSpace.
 * @param shellBuilder The function to build shell.
 * @return DMGPlannerAllocatorFn
 */
template <typename ShellPoint>
DMGPlannerAllocatorFn dynamic_planner_simple_reorder_sphere(SimpleIncrementalTSPMethods::Strategy strategy, MkOmplShellFn<ShellPoint> shellBuilder) {
	return [strategy, shellBuilder](const ompl::base::SpaceInformationPtr &si) -> DMGPlannerPtr {
		return std::make_shared<CachingDynamicPlanner<ShellPoint>>(
				std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si),
				std::make_shared<SimpleIncrementalTSPMethods>(strategy),
				shellBuilder
		);
	};
}


#endif //NEW_PLANNERS_PLANNER_ALLOCATORS_H
