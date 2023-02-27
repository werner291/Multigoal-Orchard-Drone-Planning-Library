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
#include "../utilities/MeshOcclusionModel.h"
#include "../utilities/alpha_shape.h"

#include <vtkProperty.h>
#include <range/v3/view/iota.hpp>

Apple appleFromMesh(const shape_msgs::msg::Mesh &mesh) {
	return Apple{mesh_aabb(mesh).center(), {0.0, 0.0, 0.0}};
}

using DynamicPlannerAllocatorFn = std::function<std::shared_ptr<DynamicMultiGoalPlanner>(const ompl::base::SpaceInformationPtr &)>;

std::shared_ptr<OmplShellSpace<Eigen::Vector3d>> paddedOmplSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));
	return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);
};

struct Experiment {
	std::pair<std::string, DynamicPlannerAllocatorFn> *planner;
	std::pair<Json::Value, DynamicGoalsetPlanningProblem> *problem;
};

Json::Value toJSON(const Experiment &experiment) {
	Json::Value result;
	result["planner"] = experiment.planner->first;
	result["problem"] = experiment.problem->first;
	return result;
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

	utilities::CanSeeAppleFn distance_occlusion = [](const moveit::core::RobotState &state, const Apple &apple) {

		Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

		const double discovery_max_distance = 1.0;

		return (ee_pos - apple.center).squaredNorm() < discovery_max_distance * discovery_max_distance;

	};

	auto alphashape = alphaShape(meshes.leaves_mesh.vertices | ranges::views::transform([](const auto &v) {
		return Eigen::Vector3d{v.x, v.y, v.z};
	}) | ranges::to_vector, LEAVES_ALPHA_SQRTRADIUS);

	MeshOcclusionModel occlusion_model(alphashape);

	utilities::CanSeeAppleFn leaf_alpha_occlusion = [&](const moveit::core::RobotState &state, const Apple &apple) {

		Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

		return occlusion_model.checkOcclusion(apple.center, ee_pos);

	};

	#define STATISTICS

#ifdef STATISTICS
	ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

	auto repIds = ranges::views::iota(0, 5);

	// Numbers of apples to throw at the planner.
	const auto nApples = {10, 20};

	// Generate a range of probabilities, ranging from 0.0 to 1.0.
	const int nProbabilities = 4;
	auto probs = ranges::views::iota(0, nProbabilities) | views::transform([](int i) { return i / (double) (nProbabilities - 1); });

	std::vector<std::pair<std::string, utilities::CanSeeAppleFn>> can_see_apple_fns = {
		std::make_pair("distance", distance_occlusion),
		std::make_pair("alpha_shape", leaf_alpha_occlusion)
	};

	auto problems =
			views::cartesian_product(repIds, probs, nApples, can_see_apple_fns) |
			views::transform([&](const auto &pair) -> std::pair<Json::Value,DynamicGoalsetPlanningProblem> {

				const auto &[repId, prob, n_total, can_see_apple] = pair;

				const auto discoverability = generateAppleDiscoverability((int) scene.apples.size(), prob, repId, n_total);

				Json::Value problem_params;
				problem_params["n_given"] = ranges::count(discoverability, AppleDiscoverabilityType::GIVEN);
				problem_params["n_discoverable"] = ranges::count(discoverability, AppleDiscoverabilityType::DISCOVERABLE);
				problem_params["n_total"] = n_total;
				problem_params["visibility_model"] = can_see_apple.first;

				DynamicGoalsetPlanningProblem problem {
					.start_state= randomStateOutsideTree(robot, repId),
					.apple_discoverability=	discoverability,
					.can_see_apple=	can_see_apple.second
				};

				return {
					problem_params,
					problem
				};

			}) | to_vector;

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
//			{"dynamic_planner_lci", dynamic_planner_lci},
			{"dynamic_planner_fre", dynamic_planner_fre}
			};

	auto experiments = views::cartesian_product(planners, problems) | views::transform([](const auto &pair) {
		const auto &[planner, problem] = pair;
		return Experiment {
			.planner= &planner,
			.problem= &problem
		};
	}) | to_vector;

	runExperimentsParallelRecoverable<Experiment>(experiments, [&](const Experiment &experiment) {

		// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
		// So, we just re-create the state space every time just to be safe.
		auto ss = omplStateSpaceForDrone(robot);

		// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
		// we'll need to copy this for every thread
		auto si = loadSpaceInformation(ss, scene);

		auto ompl_planner = experiment.planner->second(si);

		auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(ompl_planner, si, ss);

		DynamicGoalVisitationEvaluation eval(
				adapter,
				experiment.problem->second.start_state,
				scene,
				experiment.problem->second.apple_discoverability,
				experiment.problem->second.can_see_apple
				);

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

	DynamicGoalVisitationEvaluation eval(adapter, start_state, scene, apple_discoverability, distance_occlusion);

	visualizeEvaluation(meshes, scene, robot, start_state, apple_discoverability, eval);

#endif
}
