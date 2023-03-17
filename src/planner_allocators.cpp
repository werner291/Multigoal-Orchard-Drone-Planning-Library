// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "planner_allocators.h"
#include "ORToolsTSPMethods.h"
#include "planners/ChangeIgnoringReplannerAdapter.h"
#include "planners/ChangeAccumulatingPlannerAdapter.h"
#include "planners/InitialOrbitPlanner.h"
#include "planners/CachingDynamicPlanner.h"
#include "planners/ShellPathPlanner.h"
#include "planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "shell_space/SphereShell.h"
#include "SimpleIncrementalTSPMethods.h"

std::shared_ptr<OmplShellSpace<Eigen::Vector3d>>
paddedOmplSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));
	return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);
};

DMGPlannerPtr dynamic_planner_fre(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
																			si),
																	std::make_shared<ORToolsTSPMethods>(),
																	paddedOmplSphereShell);
};

DMGPlannerPtr static_planner(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<ChangeIgnoringReplannerAdapter>(std::make_shared<ShellPathPlanner<Eigen::Vector3d >>(
			paddedOmplSphereShell,
			std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d >>(si),
			true));
};

DMGPlannerPtr dynamic_planner_initial_orbit(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<InitialOrbitPlanner>(dynamic_planner_fre(si));
};

DMGPlannerPtr batch_replanner(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<ChangeAccumulatingPlannerAdapter>(std::make_shared<ShellPathPlanner<Eigen::Vector3d >>(
			paddedOmplSphereShell,
			std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d >>(si), true));
};

DMGPlannerPtr dynamic_planner_lci(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
																			si),
																	std::make_shared<SimpleIncrementalTSPMethods>(
																			SimpleIncrementalTSPMethods::Strategy::LeastCostlyInsertion),
																	paddedOmplSphereShell);
}

DMGPlannerPtr dynamic_planner_LIFO(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
																			si),
																	std::make_shared<SimpleIncrementalTSPMethods>(
																			SimpleIncrementalTSPMethods::Strategy::LastInFirstOut),
																	paddedOmplSphereShell);
}

DMGPlannerPtr dynamic_planner_FIFO(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
																			si),
																	std::make_shared<SimpleIncrementalTSPMethods>(
																			SimpleIncrementalTSPMethods::Strategy::FirstInFirstOut),
																	paddedOmplSphereShell);
}

DMGPlannerPtr dynamic_planner_FISO(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
																			si),
																	std::make_shared<SimpleIncrementalTSPMethods>(
																			SimpleIncrementalTSPMethods::Strategy::FirstInSecondOut),
																	paddedOmplSphereShell);
}

DMGPlannerPtr dynamic_planner_random(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
																			si),
																	std::make_shared<SimpleIncrementalTSPMethods>(
																			SimpleIncrementalTSPMethods::Strategy::Random),
																	paddedOmplSphereShell);
}
