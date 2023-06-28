// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "../utilities/fcl_util.h"
#include "../utilities/experiment_utils.h"

#include "../pure/shell_path_planning.h"
#include "../utilities/mesh_utils.h"
#include "../pure/shell_space_impl/CGALMeshShell.h"
#include "../pure/ShellConfigurationSpace.h"

#include "../pure/moveit_impl/ShellConfigurationFromShellSurface.h"
#include "../pure/approach_planning.h"
#include "../utilities/convex_hull.h"
#include "../pure/moveit_impl/EndEffectorOnPointGoalRegion.h"

#include <fcl/fcl.h>
#include <range/v3/view/transform.hpp>
#include <range/v3/to_container.hpp>

int main(int argc, char **argv) {

	using namespace mgodpl;
	using namespace fcl_util;
	using namespace moveit_impl;
	using namespace shell_path_planning;
	using namespace mesh_shell;

	using ShellSpace = ShellConfigurationFromShellSurface<CGALMeshShell>;

	auto drone = loadRobotModel();

	auto tree_model = loadTreeMeshes("appletree");

	auto trunk = tree_model.trunk_mesh;

	auto trunk_collision = rosMeshMsgToCollisionObject(trunk);

	auto start_state = randomStateOutsideTree(drone, 42);

	const std::vector<Eigen::Vector3d> goal_points = tree_model.fruit_meshes | ranges::views::transform([](const auto& mesh) -> Eigen::Vector3d {
		return mesh_aabb(mesh).center();
	}) | ranges::to_vector;

	std::vector<chull_tools::Kernel::Point_3> points = tree_model.leaves_mesh.vertices | ranges::views::transform([](const auto& vertex) -> chull_tools::Kernel::Point_3 {
		return chull_tools::Kernel::Point_3(vertex.x, vertex.y, vertex.z);
	}) | ranges::to_vector;

	ShellSpace shell_configuration_space {
		.robot_model = drone,
		.end_effector = drone->getLinkModel("end_effector"),
		.shell = std::move(CGALMeshShell(chull_tools::computeConvexHullAsMesh(points), 0.0, 0.0))
	};

	// Create an instance of ApproachPlanningFns
	ApproachPlanningFns<RobotPath, ShellSpace, EndEffectorOnPointGoalRegion> approachPlanningFnsInstance;

	// Link the lambdas
	approachPlanningFnsInstance.plan_shell_retraction_path = [](const ShellSpace& shell, const EndEffectorOnPointGoalRegion& goal_point) -> ApproachPath<RobotPath, CGALMeshShell::ShellPoint> {

		auto closest_pair = approach_planning::find_closest_configurations(shell, goal_point);

		moveit::core::RobotState shell_conf = configuration_at_internal_point(shell, closest_pair.first);

		return {
			.path = straight_path_from_to<RobotPath>(shell_conf,closest_pair.second),
			.shell_point = closest_pair.first
		};

	};

	approachPlanningFnsInstance.plan_shell_retraction_path_from_configuration = [](const ShellSpace& shell, const moveit::core::RobotState& goal_configuration) -> ApproachPath<RobotPath, CGALMeshShell::ShellPoint> {

		Eigen::Vector3d ee_pos = goal_configuration.getGlobalLinkTransform(shell.end_effector).translation();

		auto shell_point = project_euclidean_to_shell(shell.shell, ee_pos);

		auto shell_state = configuration_at_internal_point(shell, shell_point);

		return {
			.path = straight_path_from_to<RobotPath>(shell_state, goal_configuration),
			.shell_point = shell_point
		};

	};

	LocalOptFn<RobotPath> identity_opt = [](const RobotPath& path) -> RobotPath {
		return path;
	};

	std::vector<EndEffectorOnPointGoalRegion> goal_regions = goal_points | ranges::views::transform([=](const auto& point) -> EndEffectorOnPointGoalRegion {
		return EndEffectorOnPointGoalRegion {
			.target_point = point,
			.robot_model = drone,
			.end_effector = drone->getLinkModel("end_effector"),
			.max_distance = 0.05
		};
	}) | ranges::to_vector;

	plan(
		start_state,
		goal_regions,
		approachPlanningFnsInstance,
		identity_opt,
		shell_configuration_space
			);

}