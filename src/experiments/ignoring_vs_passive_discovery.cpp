#include <range/v3/all.hpp>
#include <boost/asio.hpp>

#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"
#include "../planners/ShellPathPlanner.h"
#include "../vtk/Viewer.h"
#include "../CurrentPathState.h"
#include "../vtk/SimpleVtkViewer.h"

struct PlanningProblem {
	moveit::core::RobotState start_state;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
};

void quickVisualizePath(const TreeMeshes &meshes, const RobotPath &path);

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh) {
	return Apple{mesh_aabb(mesh).center(), {0.0, 0.0, 0.0}};
}

int main(int argc, char **argv) {

	TreeMeshes meshes = loadTreeMeshes("appletree");

	meshes.fruit_meshes.resize(10);

	const auto scene_msg = treeMeshesToMoveitSceneMsg(meshes);

	const auto scene = AppleTreePlanningScene{scene_msg, meshes.fruit_meshes | ranges::views::transform(appleFromMesh) |
														 ranges::to_vector};

	const auto robot = loadRobotModel();
	const int N_INITIAL_STATES = 100;

	using namespace ranges;

	boost::asio::thread_pool pool(4);

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));
	auto shell = std::make_shared < MoveItShellSpace < Eigen::Vector3d >> (robot, workspaceShell);

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto threadLocalStateSpace = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this every time.
	auto si = loadSpaceInformation(threadLocalStateSpace, scene);

	const auto start_state = randomStateOutsideTree(robot, 0);
	auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size());

	auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si);

	auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {

		auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

		return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

	};

	auto static_planner = std::make_shared<ShellPathPlanner<Eigen::Vector3d>>(shellBuilder,
																			  std::move(approach_methods),
																			  true);

	ompl::base::ScopedState start(si);
	threadLocalStateSpace->copyToOMPLState(start.get(), start_state);

	std::vector<ompl::base::GoalPtr> goals =
			views::ints(0, (int) scene.apples.size()) | views::filter([&](int goal_id) {
				return apple_discoverability[goal_id];
			}) | views::transform([&](int goal_id) {
				auto goal = std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, scene.apples[goal_id].center);
				return std::static_pointer_cast<ompl::base::Goal>(goal);
			}) | to_vector;

	auto ptc = ompl::base::plannerNonTerminatingCondition();

	auto result = static_planner->plan(si, start.get(), goals, scene, ptc);

	RobotPath path = omplPathToRobotPath(result.combined());

	quickVisualizePath(meshes, path);

	return EXIT_SUCCESS;

}

