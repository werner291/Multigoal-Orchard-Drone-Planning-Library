// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 3-3-23.
//

#include <gtest/gtest.h>
#include "../src/TreeMeshes.h"
#include "../src/utilities/experiment_utils.h"
#include "../src/planning_scene_diff_message.h"
#include "../src/shell_space/SphereShell.h"
#include "../src/shell_space/OmplShellSpace.h"
#include "../src/planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include <range/v3/all.hpp>

TEST(ApproachPlanningAccuracy, test) {


	// Load the apple tree meshes.
	auto drone = loadRobotModel();

	auto meshes = loadTreeMeshes("appletree");

	const auto scene = AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(
			treeMeshesToMoveitSceneMsg(meshes))), .apples = meshes.fruit_meshes |
															ranges::views::transform(appleFromMesh) |
															ranges::to_vector};


	// Convert the meshes to a planning scene message.
	//	const auto scene = createMeshBasedAppleTreePlanningSceneMessage("appletree", true);

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(drone);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, scene);

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));
	auto shell = OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

	auto approach_planner = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);

	auto goals = constructNewAppleGoals(si, scene.apples);

	std::vector<bool> approach_successes;

	for (const auto &goal: goals) {
		auto result = approach_planner->approach_path(goal, *shell);
		approach_successes.push_back(result.has_value());
	}

	double proportion_success = (double) ranges::count(approach_successes, true) / (double) approach_successes.size();

	std::cout << "Proportion of successful approaches: " << proportion_success << std::endl;

	EXPECT_GT(proportion_success, 0.95);

}