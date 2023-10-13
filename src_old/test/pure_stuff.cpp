// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../src/pure/shell_path_planning.h"
#include "../src/utilities/experiment_utils.h"
#include "../src/pure/shell_space_impl/MinimumEnclosingSphereShell.h"
#include "../src/shell_space/CGALMeshShell.h"
#include "../src/pure/moveit_impl/path.h"

template<>
std::string mgodpl::path_reverse(std::string path) {
	return {path.rbegin(), path.rend()};
}

template<>
std::string mgodpl::path_concatenate(std::string paths...) {
	std::string result;
	for (const auto& path : paths) {
		result += path;
	}
	return result;
}


template<>
struct mgodpl::PathTraits<std::string> {

	static std::string concatenate(const std::vector<std::string>& paths) {
		std::string result;
		for (const auto& path : paths) {
			result += path;
		}
		return result;
	}
};

template<>
struct mgodpl::ShellSpaceTraits<std::string> {
	using ShellPoint = char;

	static std::string shell_path(ShellPoint sp1, ShellPoint sp2) {
		return std::string(1, sp1) + std::string(1, sp2);
	}
};


TEST(PureStuff, Test1) {

	using namespace mgodpl;

	ApproachPath<std::string, char> approachPath1 {	"abc", 'A' };

	ApproachToGoal<std::string, char, std::size_t> approachToGoal2 {{"def",	'D'}, 42};

	auto composedPath = composeRetreatShellApproachPath<std::string, std::string>(
			approachPath1,
			approachToGoal2,
			std::string(),
			[](const std::string& path) { return path; }
	);

	EXPECT_EQ(composedPath, "cbaADdef");

}

TEST(PureMoveit, TestBasic) {

	auto drone = loadRobotModel();

	auto state_outside_tree = randomStateOutsideTree(drone, 42);

	auto tree_meshes = loadTreeMeshes("appletree");

	CGALMeshShell shell(convexHull(tree_meshes.leaves_mesh.vertices), 1.0, 0.0);

	mgodpl::PlanShellRetractionPathFn<RobotPath, mgodpl::SphereShell, Eigen::Vector3d> fn =
			[](const mgodpl::SphereShell& shell, const Eigen::Vector3d& goal) -> std::optional<RobotPath> { return std::nullopt; };

	mgodpl::ApproachPlanningFns<RobotPath, mgodpl::SphereShell, Eigen::Vector3d> approachPlanningFns {
			[](const mgodpl::SphereShell& shell, const Eigen::Vector3d& goal) -> std::optional<RobotPath> { return {}; },
			[](const mgodpl::SphereShell& shell, const Eigen::Vector3d& goal) -> std::optional<RobotPath> { return {}; }
	};

//	mgodpl::plan(
//			drone,
//			state_outside_tree,
//			approachPlanningFns,
//			[](const auto& path) { return path; },
//
//			)

}