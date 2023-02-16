#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../planning_scene_diff_message.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"
#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "../planners/ShellPathPlanner.h"
#include "../vtk/Viewer.h"
#include "../CurrentPathState.h"

#include "../vtk/SimpleVtkViewer.h"
#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/goal_events.h"
#include "../DynamicGoalVisitationEvaluation.h"
#include "../vtk/visualize_dynamic.h"

#include <vtkProperty.h>

struct PlanningProblem {
	moveit::core::RobotState start_state;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
};

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

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));
	auto shell = std::make_shared<MoveItShellSpace<Eigen::Vector3d >>(robot, workspaceShell);

	const auto start_state = randomStateOutsideTree(robot, 0);
	auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size(), 1.0);

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, scene);

	using DynamicPlannerAllocatorFn = std::function<std::shared_ptr<DynamicMultiGoalPlanner>(const ompl::base::SpaceInformationPtr&)>;

	DynamicPlannerAllocatorFn planner_allocator = [](const ompl::base::SpaceInformationPtr& si) -> std::shared_ptr<DynamicMultiGoalPlanner> {

		auto approach_methods = std::make_unique < MakeshiftPrmApproachPlanningMethods < Eigen::Vector3d >> (si);

		auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {

			auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

			return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

		};

		auto static_planner = std::make_shared < ShellPathPlanner <
							  Eigen::Vector3d >> (shellBuilder, std::move(approach_methods), true);

		return std::make_shared<ChangeIgnoringReplannerAdapter>(static_planner);
	};

	boost::asio::thread_pool pool(4);

	const size_t REPETITIONS = 10;

	std::mutex mutex;
	std::vector<int> n_visited_all;

	boost::asio::post(pool, [&] {
		auto change_ignoring = planner_allocator(si);

		DynamicGoalVisitationEvaluation eval(change_ignoring, start_state, scene, apple_discoverability, si);

		eval.computeNextTrajectory();

		while (eval.getRe()) {
			eval.computeNextTrajectory();
		}

		int n_visited = ranges::count_if(eval.getDiscoveryStatus(), [](const auto &status) { return status == utilities::VISITED; });

		std::lock_guard<std::mutex> lock(mutex);
		n_visited_all.push_back(n_visited);

	});

	pool.join();

	Json::Value result;

	for (size_t i = 0; i < n_visited_all.size(); i++) {
		result["change_ignoring"][(int) i] = n_visited_all[i];
	}

	std::ofstream out("analysis/dynamic.json");
	out << result;

//	return visualizeEvaluation(meshes, scene, robot, start_state, apple_discoverability, eval);

}

