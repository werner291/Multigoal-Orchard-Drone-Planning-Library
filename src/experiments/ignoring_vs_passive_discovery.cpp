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

#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/goal_events.h"
#include "../DynamicGoalVisitationEvaluation.h"
#include "../planners/CachingDynamicPlanner.h"
#include "../ORToolsTSPMethods.h"
#include "../utilities/run_experiments.h"

#include "../DynamicGoalsetPlanningProblem.h"
#include "../vtk/visualize_dynamic.h"

#include <vtkProperty.h>
#include <range/v3/view/iota.hpp>

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh) {
	return Apple{mesh_aabb(mesh).center(), {0.0, 0.0, 0.0}};
}

using DynamicPlannerAllocatorFn = std::function<std::shared_ptr<DynamicMultiGoalPlanner>(const ompl::base::SpaceInformationPtr &)>;

struct Experiment {
	DynamicPlannerAllocatorFn *allocator;
	std::string planner_name;
	DynamicGoalsetPlanningProblem problem;
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

std::vector<Experiment> genPlannerProblemMatrix(const std::vector<DynamicGoalsetPlanningProblem> &problems,
												std::vector<std::pair<std::string, DynamicPlannerAllocatorFn>> &planners) {
	auto experiments =
			ranges::views::cartesian_product(planners, problems) | ranges::views::transform([&](const auto &pair) {
				auto &[planner, problem] = pair;
				auto &[planner_name, allocator] = planner;
				return Experiment{&allocator, planner_name, problem};
			}) | ranges::to_vector;
	return experiments;
}

int main(int argc, char **argv) {

	TreeMeshes meshes = loadTreeMeshes("appletree");

	const auto scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(treeMeshesToMoveitSceneMsg(meshes)));

	const auto scene = AppleTreePlanningScene {
			scene_msg, meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector};

	const auto robot = loadRobotModel();

	using namespace ranges;

	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene, 0.1));
	auto shell = std::make_shared<MoveItShellSpace<Eigen::Vector3d >>(robot, workspaceShell);

	DynamicPlannerAllocatorFn dynamic_planner_fre = [](const ompl::base::SpaceInformationPtr &si) -> std::shared_ptr<DynamicMultiGoalPlanner> {
		return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
																				si),
																		std::make_shared<ORToolsTSPMethods>(
																				ORToolsTSPMethods::UpdateStrategy::FULL_REORDER),
																		paddedOmplSphereShell);
	};

	//#define STATISTICS

#ifdef STATISTICS
	ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

	auto problems = DynamicGoalsetPlanningProblem::genDynamicGoalsetPlanningProblems(scene, robot, 10);

	DynamicPlannerAllocatorFn static_planner = [](const ompl::base::SpaceInformationPtr &si) -> std::shared_ptr<DynamicMultiGoalPlanner> {
		return std::make_shared<ChangeIgnoringReplannerAdapter>(
				std::make_shared<ShellPathPlanner<Eigen::Vector3d >>(
						paddedOmplSphereShell,
						std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d >>(si),
						true));
	};

	DynamicPlannerAllocatorFn dynamic_planner_lci = [](const ompl::base::SpaceInformationPtr &si) -> std::shared_ptr<DynamicMultiGoalPlanner> {
		return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(
				std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(si),
				std::make_shared<ORToolsTSPMethods>(ORToolsTSPMethods::UpdateStrategy::LEAST_COSTLY_INSERT),
				paddedOmplSphereShell
		);
	};

	std::vector<std::pair<std::string, DynamicPlannerAllocatorFn>> planners =
			{
			{"change_ignoring", static_planner},
			{"dynamic_planner_lci", dynamic_planner_lci},
			{"dynamic_planner_fre", dynamic_planner_fre}
			};

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
		result["total_path_length"] = total_path_length;

		return result;

	}, "analysis/data/dynamic_log.json", 16, 8, 42);
#else

	const auto start_state = randomStateOutsideTree(robot, 0);
	auto apple_discoverability = generateAppleDiscoverability((int) scene.apples.size(), 1.0, 42, 1);

	std::cout << "Starting planning with " << apple_discoverability.size() << " apples in total, of which "
			  << ranges::count(apple_discoverability, DISCOVERABLE) << " are discoverable." << std::endl;

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, scene);

	auto planner = dynamic_planner_fre(si);

	auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(planner, si, ss);

	DynamicGoalVisitationEvaluation eval(adapter, start_state, scene, apple_discoverability);

	visualizeEvaluation(meshes, scene, robot, start_state, apple_discoverability, eval);

#endif
}
