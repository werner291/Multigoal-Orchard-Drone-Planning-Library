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
#include "../planners/CachingDynamicPlanner.h"
#include "../ORToolsTSPMethods.h"
#include "../utilities/run_experiments.h"

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
	result["n_given"] = ranges::count(experiment.problem.apple_discoverability, AppleDiscoverabilityType::GIVEN);
	result["n_discoverable"] = ranges::count(experiment.problem.apple_discoverability, AppleDiscoverabilityType::DISCOVERABLE);
	return result;
}

std::shared_ptr<OmplShellSpace<Eigen::Vector3d>> paddedOmplSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));

	return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);

};

std::shared_ptr<DynamicMultiGoalPlanner> static_baseline_planner(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<ChangeIgnoringReplannerAdapter>(
			std::make_shared<ShellPathPlanner<Eigen::Vector3d >>(
					paddedOmplSphereShell,
					std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d >>(si),
					true));
};

std::shared_ptr<DynamicMultiGoalPlanner> dynamic_planner(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(
			std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si),
			std::make_shared<ORToolsTSPMethods>(),
			paddedOmplSphereShell
			);
};

std::vector<PlanningProblem> genDynamicGoalsetPlanningProblems(const AppleTreePlanningScene &scene, const moveit::core::RobotModelPtr &robot) {

	const int REPETITIONS = 2;

	// Generate a sequence of IDs.
	auto rep_ids = ranges::views::iota(0, REPETITIONS);

	// Generate a range of probabilities, ranging from 0.0 to 1.0.

	const int N_PROBABILITIES = 2;

	auto probs = ranges::views::iota(0, N_PROBABILITIES) | ranges::views::transform([](int i) { return i / (double) (N_PROBABILITIES-1); });

	auto combinations = ranges::views::cartesian_product(rep_ids, probs);

	return combinations | ranges::views::transform([&](const auto &pair) {
		const auto &[rep_id, prob] = pair;
		return PlanningProblem{randomStateOutsideTree(robot, rep_id),
							   generateAppleDiscoverability((int) scene.apples.size(), prob, rep_id)};
	}) | ranges::to_vector;

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

	const auto scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(treeMeshesToMoveitSceneMsg(meshes)));

	const auto scene = AppleTreePlanningScene {
		scene_msg,
		meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector
	};

	const auto robot = loadRobotModel();

	using namespace ranges;

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));
	auto shell = std::make_shared<MoveItShellSpace<Eigen::Vector3d >>(robot, workspaceShell);

	{
		ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

		DynamicPlannerAllocatorFn planner_allocator_1 = static_baseline_planner;
		DynamicPlannerAllocatorFn planner_allocator_2 = dynamic_planner;

		std::vector<PlanningProblem> problems = genDynamicGoalsetPlanningProblems(scene, robot);

		const std::vector<std::pair<std::string, DynamicPlannerAllocatorFn *>> planners =
				{{"change_ignoring", &planner_allocator_1},
				 {"dynamic_planner", &planner_allocator_2}};

		auto experiments = genPlannerProblemMatrix(problems, planners);

		runExperimentsParallelRecoverable<Experiment>(experiments, [&](const Experiment &experiment) {

			// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
			// So, we just re-create the state space every time just to be safe.
			auto ss = omplStateSpaceForDrone(robot);

			// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
			// we'll need to copy this for every thread
			auto si = loadSpaceInformation(ss, scene);

			auto change_ignoring = (*experiment.allocator)(si);

			auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(change_ignoring, si, ss);

			DynamicGoalVisitationEvaluation eval(adapter, experiment.problem.start_state, scene, experiment.problem.apple_discoverability);

			auto start_time = std::chrono::high_resolution_clock::now();

			eval.computeNextTrajectory();

			while (eval.getUpcomingGoalEvent().has_value()) {
				if (!eval.computeNextTrajectory().has_value()) {
					break;
				}
			}

			auto end_time = std::chrono::high_resolution_clock::now();

			int n_visited = (int) ranges::count_if(eval.getDiscoveryStatus(),
												   [](const auto &status) { return status == utilities::VISITED; });

			double total_path_length = ranges::accumulate(
					eval.getSolutionPathSegments() | ranges::views::transform([](const auto &segment) {
						return segment.first.length();
					}), 0.0);

			Json::Value result;

			result["n_visited"] = n_visited;
			result["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

			return result;

		}, "analysis/data/dynamic_log.json", 16, 1, 42);
	}

//	if (false) {
//
//		const auto start_state = randomStateOutsideTree(robot, 0);
//		auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size(), 0.3, 42);
//
//		std::cout << "Starting planning with " << apple_discoverability.size() << " apples in total, of which "
//				  << ranges::count(apple_discoverability, DISCOVERABLE) << " are discoverable." << std::endl;
//
//		// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
//		// So, we just re-create the state space every time just to be safe.
//		auto ss = omplStateSpaceForDrone(robot);
//
//		// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
//		// we'll need to copy this for every thread
//		auto si = loadSpaceInformation(ss, scene);
//
//		auto planner = dynamic_planner(si);
//
//		auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(planner, si, ss);
//
//		DynamicGoalVisitationEvaluation eval(adapter, start_state, scene, apple_discoverability);
//
//		visualizeEvaluation(meshes, scene, robot, start_state, apple_discoverability, eval);
//	}

}

