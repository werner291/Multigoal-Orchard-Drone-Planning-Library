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

/// Alias for a shared pointer to a MultiGoalPlanner.
using StaticPlannerPtr = std::shared_ptr<MultiGoalPlanner>;

/// Alias for a function that creates a MultiGoalPlanner given a SpaceInformation.
using StaticPlannerAllocatorFn = std::function<StaticPlannerPtr(const ompl::base::SpaceInformationPtr &)>;

/**
 * @brief Factory function to create a StaticPlannerAllocatorFn that uses a shell-based planner.
 *
 * This function returns another function that, when called, creates a StaticPlannerPtr. The created planner
 * uses the MakeshiftPrmApproachPlanningMethods for path planning, and uses the provided shell builder to assist
 * in the planning process.
 *
 * @tparam ShellPoint The type of points used to define the shell around the goal region.
 * @param shellBuilder A function to build the shell around a goal region.
 * @param approach_max_t The maximum time allowed for each planning attempt.
 *
 * @return A StaticPlannerAllocatorFn that creates a shell-based planner.
 */
template <typename ShellPoint>
StaticPlannerAllocatorFn makeShellBasedPlanner(MkOmplShellFn<ShellPoint> shellBuilder, double approach_max_t) {
	return [shellBuilder,approach_max_t](const ompl::base::SpaceInformationPtr& si) -> StaticPlannerPtr {
		auto planner = std::make_shared<ShellPathPlanner<ShellPoint>>(
				shellBuilder,
				std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si, approach_max_t),
				true);
		return std::dynamic_pointer_cast<MultiGoalPlanner>(planner);
	};
}

/// Alias for a shared pointer to a DynamicMultiGoalPlanner.
using DMGPlannerPtr = std::shared_ptr<DynamicMultiGoalPlanner>;

/// Alias for a function that creates a DynamicMultiGoalPlanner given a SpaceInformation.
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
				mgodpl::tsp_utils::incremental_tsp_order_ortools_always_reorder(),
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
DMGPlannerAllocatorFn static_planner(MkOmplShellFn<ShellPoint> paddedOmplSphereShell, const double max_approach_time = 1.0) {
	return [paddedOmplSphereShell, max_approach_time=max_approach_time](const ompl::base::SpaceInformationPtr &si) -> DMGPlannerPtr {
		return std::make_shared<ChangeIgnoringReplannerAdapter>(
				std::make_shared<ShellPathPlanner<ShellPoint>>(
						paddedOmplSphereShell,
						std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint >>(si, max_approach_time),
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
						std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint >>(si, 1.0),
						true
				)
		);
	};
};

///**
// * @brief Dynamic planner with simple reordering on a sphere.
// *
// * @tparam ShellPoint Data type of the shell points.
// * @param strategy The reordering strategy to use.
// * @param paddedOmplSphereShell Function for constructing a OmplShellSpace.
// * @param shellBuilder The function to build shell.
// * @return DMGPlannerAllocatorFn
// */
//template <typename ShellPoint>
//DMGPlannerAllocatorFn dynamic_planner_simple_reorder_sphere(SimpleIncrementalTSPMethods::Strategy strategy, MkOmplShellFn<ShellPoint> shellBuilder) {
//	return [strategy, shellBuilder](const ompl::base::SpaceInformationPtr &si) -> DMGPlannerPtr {
//		return std::make_shared<CachingDynamicPlanner<ShellPoint>>(
//				std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si, 1.0),
//				std::make_shared<SimpleIncrementalTSPMethods>(strategy),
//				shellBuilder
//		);
//	};
//}


#endif //NEW_PLANNERS_PLANNER_ALLOCATORS_H
