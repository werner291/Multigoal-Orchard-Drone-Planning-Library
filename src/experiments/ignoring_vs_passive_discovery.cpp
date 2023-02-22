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
#include "../utilities/run_experiments.h"
#include "../vtk/visualize_dynamic.h"
#include "../planners/CachingDynamicPlanner.h"

#include <vtkProperty.h>
#include <range/v3/view/iota.hpp>

struct PlanningProblem {
	moveit::core::RobotState start_state;
	std::vector<AppleDiscoverabilityType> apple_discoverability;
};

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh) {
	return Apple{mesh_aabb(mesh).center(), {0.0, 0.0, 0.0}};
}

using DynamicPlannerAllocatorFn = std::function<std::shared_ptr<DynamicMultiGoalPlanner>(const ompl::base::SpaceInformationPtr&)>;

struct Experiment {
	DynamicPlannerAllocatorFn *allocator;
	std::string planner_name;
	PlanningProblem problem;
};

Json::Value toJSON(const Experiment &experiment) {
	Json::Value result;
	result["planner"] = experiment.planner_name;
	return result;
}

std::shared_ptr<DynamicMultiGoalPlanner> static_baseline_planner(const ompl::base::SpaceInformationPtr &si) {

	auto approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d >>(si);

	auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {

		auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

		return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

	};

	auto static_planner = std::make_shared<ShellPathPlanner<Eigen::Vector3d >>(shellBuilder,
																			   std::move(approach_methods),
																			   true);

	return std::make_shared<ChangeIgnoringReplannerAdapter>(static_planner);
};

class ORToolsTSPMethods : public IncrementalTSPMethods {

public:
	std::vector<size_t> initial_ordering(size_t n,
										 std::function<double(size_t, size_t)> distance,
										 std::function<double(size_t)> first_distance) override {

		return tsp_open_end(first_distance, distance, n);

	}

	std::vector<size_t> update_ordering(const std::vector<size_t> &current_ordering,
										size_t new_goal,
										std::function<double(size_t, size_t)> distance,
										std::function<double(size_t)> first_distance) override {
		throw std::runtime_error("Not implemented");
	}
};

std::shared_ptr<DynamicMultiGoalPlanner> dynamic_planner(const ompl::base::SpaceInformationPtr &si) {

	std::shared_ptr<ApproachPlanningMethods<Eigen::Vector3d>> approach_methods = std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
			si);

	auto shellBuilder = [](const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {

		auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

		return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

	};

	auto tsp = std::make_shared<ORToolsTSPMethods>();

	auto dynamic_planner = std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(approach_methods,
																					tsp,
																					shellBuilder);

	return dynamic_planner;
};

std::vector<PlanningProblem>
genDynamicGoalsetPlanningProblems(const AppleTreePlanningScene &scene, const moveit::core::RobotModelPtr &robot) {
	const int REPETITIONS = 10;

	std::vector<PlanningProblem> problems = ranges::views::iota(0, REPETITIONS) | ranges::views::transform([&](int i) {
		return PlanningProblem{randomStateOutsideTree(robot, i),
							   generateAppleDiscoverability((int) scene.apples.size(), 1.0, i)};
	}) | ranges::to_vector;

	return problems;
}

std::vector<Experiment> genPlannerProblemMatrix(const std::vector<PlanningProblem> &problems,
												const std::vector<std::pair<std::string, DynamicPlannerAllocatorFn *>> &planners) {
	auto experiments =
			ranges::views::cartesian_product(planners, problems) | ranges::views::transform([&](const auto &pair) {
				const auto &[planner, problem] = pair;
				const auto &[planner_name, allocator] = planner;
				return Experiment{allocator, planner_name, problem};
			}) | ranges::to_vector;
	return experiments;
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
	auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size(), 0.5, 42);

	std::cout << "Starting planning with " << apple_discoverability.size() << " apples in total, of which "
			  << ranges::count(apple_discoverability, DISCOVERABLE) << " are discoverable." << std::endl;

	//	DynamicPlannerAllocatorFn planner_allocator = static_baseline_planner;
	DynamicPlannerAllocatorFn planner_allocator = dynamic_planner;

	//	std::vector<PlanningProblem> problems = genDynamicGoalsetPlanningProblems(scene, robot);
	//
	//	const std::vector<std::pair<std::string,DynamicPlannerAllocatorFn*>> planners = {
	//			{ "change_ignoring", &planner_allocator }
	//	};
	//
	//	auto experiments = genPlannerProblemMatrix(problems, planners);
	//
	//	runExperimentsParallelRecoverable<Experiment>(experiments, [&](const Experiment& experiment) {
	//
	//		// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	//		// So, we just re-create the state space every time just to be safe.
	//		auto ss = omplStateSpaceForDrone(robot);
	//
	//		// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	//		// we'll need to copy this for every thread
	//		auto si = loadSpaceInformation(ss, scene);
	//
	//		auto change_ignoring = (*experiment.allocator)(si);
	//
	//		DynamicGoalVisitationEvaluation eval(change_ignoring, start_state, scene, apple_discoverability, si);
	//
	//		eval.computeNextTrajectory();
	//
	//		while (eval.getUpcomingGoalEvent().has_value()) {
	//			eval.computeNextTrajectory();
	//		}
	//
	//		int n_visited = (int) ranges::count_if(eval.getDiscoveryStatus(), [](const auto &status) { return status == utilities::VISITED; });
	//
	//		Json::Value result;
	//
	//		result["n_visited"] = n_visited;
	//
	//		return result;
	//
	//	}, "analysis/data/dynamic_log.json", 16, 4, 42);

	{
		// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
		// So, we just re-create the state space every time just to be safe.
		auto ss = omplStateSpaceForDrone(robot);

		// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
		// we'll need to copy this for every thread
		auto si = loadSpaceInformation(ss, scene);

		auto planner = planner_allocator(si);

		auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(planner, si, ss);

		DynamicGoalVisitationEvaluation eval(adapter, start_state, scene, apple_discoverability);

		visualizeEvaluation(meshes, scene, robot, start_state, apple_discoverability, eval);
	}

}

