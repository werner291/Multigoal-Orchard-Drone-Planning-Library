#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../AppleTreePlanningScene.h"
#include "../utilities/experiment_utils.h"
#include "../vtk/Viewer.h"

#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/goal_events.h"
#include "../DynamicGoalVisitationEvaluation.h"
#include "../planners/CachingDynamicPlanner.h"
#include "../utilities/run_experiments.h"

#include "../DynamicGoalsetPlanningProblem.h"
#include "../utilities/MeshOcclusionModel.h"
#include "../utilities/alpha_shape.h"
#include "../planner_allocators.h"

#include <vtkProperty.h>
#include <range/v3/view/iota.hpp>

using DynamicPlannerAllocatorFn = std::function<std::shared_ptr<DynamicMultiGoalPlanner>(const ompl::base::SpaceInformationPtr &)>;

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

class StraightlineApproachMethods : public ApproachPlanningMethods<Eigen::Vector3d> {

	//	ompl::

public:
	std::optional<OmplApproachPath<Eigen::Vector3d>>
	approach_path(const ompl::base::State *start, const OmplShellSpace<Eigen::Vector3d> &shell) const override {

		Eigen::Vector3d shell_pt = shell.pointNearState(start);

		ompl::geometric::PathGeometric path(shell.getSpaceInformation());
		path.append(start);

		ompl::geometric::PathGeometric approach_path(shell.getSpaceInformation());
		shell.stateFromPoint()

		OmplApproachPath<> result{.shell_point = shell.pointNearState(start),}


	}

	std::optional<OmplApproachPath<Eigen::Vector3d>>
	approach_path(const ompl::base::GoalPtr &goal, const OmplShellSpace<Eigen::Vector3d> &shell) const override {
		return std::optional<OmplApproachPath<Eigen::Vector3d>>();
	}
};

DMGPlannerPtr dynamic_planner_fre(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<Eigen::Vector3d>>(std::make_unique<
			MakeshiftPrmApproachPlanningMethods < Eigen::Vector3d>>
	(si), std::make_shared<ORToolsTSPMethods>(ORToolsTSPMethods::UpdateStrategy::FULL_REORDER), paddedOmplSphereShell);
};

DMGPlannerPtr static_planner(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<ChangeIgnoringReplannerAdapter>(std::make_shared<ShellPathPlanner < Eigen::Vector3d >>
	(paddedOmplSphereShell, std::make_unique<MakeshiftPrmApproachPlanningMethods < Eigen::Vector3d >>
	(si), true));
};

DMGPlannerPtr dynamic_planner_initial_orbit(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<InitialOrbitPlanner>(dynamic_planner_fre(si));
};

DMGPlannerPtr batch_replanner(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<ChangeAccumulatingPlannerAdapter>(std::make_shared<ShellPathPlanner < Eigen::Vector3d >>
	(paddedOmplSphereShell, std::make_unique<MakeshiftPrmApproachPlanningMethods < Eigen::Vector3d >>
	(si), true));
};

Json::Value
runDynamicPlannerExperiment(const moveit::core::RobotModelPtr &robot, const Experiment &experiment, bool saveSegments) {

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(robot);

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, experiment.problem->second.scene);

	// Allocate the planner.
	auto ompl_planner = experiment.planner->second(si);

	// Wrap it into the adapter that lets us use it with MoveIt types.
	auto adapter = std::make_shared<DynamicMultiGoalPlannerOmplToMoveitAdapter>(ompl_planner, si, ss);

	// Create the evaluation object.
	DynamicGoalVisitationEvaluation eval(adapter,
										 experiment.problem->second.start_state,
										 experiment.problem->second.scene,
										 experiment.problem->second.apple_discoverability,
										 *experiment.problem->second.can_see_apple);

	// Record the start time.
	auto start_time = std::chrono::high_resolution_clock::now();

	auto stats_at_start = getDiscoveryStatusStats(eval.getDiscoveryStatus());

	// Run the planner for the initial set of goals.
	eval.runTillCompletion();

	// Record the end time.
	auto end_time = std::chrono::high_resolution_clock::now();

	int n_visited = (int) ranges::count_if(eval.getDiscoveryStatus(),
										   [](const auto &status) { return status == utilities::VISITED; });

	double total_path_length = ranges::accumulate(
			eval.getSolutionPathSegments() | ranges::views::transform([](const auto &segment) {
				return segment.path.length();
			}), 0.0);

	std::chrono::nanoseconds total_time = ranges::accumulate(
			eval.getSolutionPathSegments() | ranges::views::transform([](const auto &segment) {
				return segment.time;
			}), std::chrono::nanoseconds(0));

	Json::Value result;
	result["initial_knowledge"] = toJSON(stats_at_start);
	result["n_visited"] = n_visited;
	result["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(total_time).count();
	result["total_path_length"] = total_path_length;

	const auto solution = eval.getSolutionPathSegments();

	if (!solution.empty()) {

		for (const auto &segment: solution) {

			Json::Value segment_json;
			segment_json["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(segment.time).count();
			segment_json["path_length"] = segment.path.length();

			for (const auto &waypoint: segment.path.waypoints) {

				moveit::core::RobotState waypoint_copy(waypoint);
				waypoint_copy.update(true);

				Json::Value waypoint_json;
				waypoint_json["base_pos"] = toJSON(Eigen::Vector3d(waypoint_copy.getGlobalLinkTransform("base_link")
																		   .translation()));
				waypoint_json["ee_pos"] = toJSON(Eigen::Vector3d(waypoint_copy.getGlobalLinkTransform("end_effector")
																		 .translation()));

				segment_json["waypoints"].append(waypoint_json);

			}

			segment_json["end_event"] = toJSON(segment.goal_event);

			result["solution_segments"].append(segment_json);

		}

	}

	return result;
}

int main(int argc, char **argv) {

	// Load the apple tree meshes.
	TreeMeshes meshes = loadTreeMeshes("appletree");

	// Convert the meshes to a planning scene message.
	const auto scene = AppleTreePlanningScene{.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(
			treeMeshesToMoveitSceneMsg(meshes))), .apples = meshes.fruit_meshes |
															ranges::views::transform(appleFromMesh) |
															ranges::to_vector};

	// Load the robot model.
	const auto robot = loadRobotModel();

	using namespace ranges;

	auto alphashape = alphaShape(meshes.leaves_mesh.vertices | ranges::views::transform([](const auto &v) {
		return Eigen::Vector3d{v.x, v.y, v.z};
	}) | ranges::to_vector, LEAVES_ALPHA_SQRTRADIUS);

	MeshOcclusionModel occlusion_model(alphashape, 0);

	CanSeeAppleFn leaf_alpha_occlusion = [&](const moveit::core::RobotState &state, const Apple &apple) {

		Eigen::Vector3d ee_pos = state.getGlobalLinkTransform("end_effector").translation();

		return !occlusion_model.checkOcclusion(apple.center, ee_pos);

	};

	ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

	auto repIds = ranges::views::iota(0, 50);

	// Numbers of apples to throw at the planner.
	const auto nApples = {
			//			10,
			//			50,
			100};

	std::vector<Proportions> probs = {
			Proportions{.fraction_true_given = 0.0, .fraction_false_given = 0.0, .fraction_discoverable = 1.0},};

	// The different occlusion functions.
	auto can_see_apple_fns = {std::make_pair("alpha_shape", leaf_alpha_occlusion)};

	// Generate a set of problems based on the carthesian product of the above ranges.
	auto problems = views::cartesian_product(repIds, probs, nApples, can_see_apple_fns) |
					views::transform([&](const auto &pair) -> std::pair<Json::Value, DynamicGoalsetPlanningProblem> {

						// Generate a problem for every unique combination of repetition ID,
						// discoverability degree, total number of apples and occlusion model.
						const auto &[repId, prob, n_total, can_see_apple] = pair;

						AppleTreePlanningScene censored_scene = scene;
						// Shuffle the apples.
						std::shuffle(censored_scene.apples.begin(), censored_scene.apples.end(), std::mt19937(repId));
						// Delete any over n.
						censored_scene.apples.resize(n_total);

						// Translate the discoverability degree into a vector of discoverability types/
						const auto discoverability = generateAppleDiscoverability(prob, repId, n_total);

						// Create a JSON object containing the parameters of the problem for later reference.
						Json::Value problem_params;
						problem_params["n_given"] = ranges::count(discoverability, AppleDiscoverabilityType::GIVEN);
						problem_params["n_discoverable"] = ranges::count(discoverability,
																		 AppleDiscoverabilityType::DISCOVERABLE);
						problem_params["n_false"] = ranges::count(discoverability, AppleDiscoverabilityType::FALSE);
						problem_params["n_total"] = n_total;
						problem_params["visibility_model"] = can_see_apple.first;

						// Create the problem, to be solved by the planners.
						DynamicGoalsetPlanningProblem problem{.start_state= randomStateOutsideTree(robot,
																								   repId), .scene=        censored_scene, .apple_discoverability=    discoverability, .can_see_apple=    &can_see_apple
								.second};

						// Return the problem parameters and the problem itself.
						return {problem_params, problem};

					}) | to_vector;

	// The different planners to test.
	std::vector<std::pair<std::string, DynamicPlannerAllocatorFn>> planners = {
			// A planner that ignores the dynamic goalset, only planning to the initially-given apples.
			// It will obviously not ve very optimal, but will serve as a baseline.
			{"change_ignoring",               static_planner},
			// A planner that adds new goals to a "batch" to be replanned to after the current
			// path has been completed.
			{"batch_replanner",               batch_replanner},
			// A planner that uses the dynamic goalset, but uses a greedy approach to insert new goals in the visitation order.
			{"dynamic_planner_lci",           dynamic_planner_lci},
			// A planner that uses the dynamic goalset, and completely reorders the visitation order from scratch every time a goal is added.
			{"dynamic_planner_fre",           dynamic_planner_fre},
			// // Same as dynamic_planner_fre, but with an initial orbit around the tree to discover some of the dynamic goals
			{"dynamic_planner_initial_orbit", dynamic_planner_initial_orbit}

	};

	// Take the carthesian product of the different planners and problems,
	// making it so that every planner is tested on every problem.
	auto experiments = views::cartesian_product(planners, problems) | views::transform([](const auto &pair) {
		const auto &[planner, problem] = pair;
		return Experiment{.planner= &planner, .problem= &problem};
	}) | to_vector;

	// Run the experiments in parallel.
	runExperimentsParallelRecoverable<Experiment>(experiments, [&](const Experiment &experiment) {
		return runDynamicPlannerExperiment(robot, experiment, true);
	}, "analysis/data/dynamic_log_advanced.json", 32, std::thread::hardware_concurrency(), 42);

}


