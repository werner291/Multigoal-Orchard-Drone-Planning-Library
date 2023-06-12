#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../AppleTreePlanningScene.h"
#include "../utilities/experiment_utils.h"
#include "../shell_space/MoveItShellSpace.h"
#include "../shell_space/SphereShell.h"
#include "../planners/shell_path_planner/ApproachPlanning.h"
#include "../planners/shell_path_planner/MakeshiftPrmApproachPlanningMethods.h"
#include "../planners/ShellPathPlanner.h"
#include "../vtk/Viewer.h"

#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../exploration/ColorEncoding.h"
#include "../utilities/goal_events.h"
#include "../planners/CachingDynamicPlanner.h"
#include "../utilities/run_experiments.h"

#include "../planner_allocators.h"
#include "../shell_space/CylinderShell.h"

#include <vtkProperty.h>


struct Problem {
	moveit::core::RobotState start;
	AppleTreePlanningScene scene;
};

using StaticPlannerAllocatorFn = std::function<std::shared_ptr<MultiGoalPlanner>(const ompl::base::SpaceInformationPtr &)>;

struct Experiment {
	std::pair<Json::Value, StaticPlannerAllocatorFn> *planner;
	const std::pair<Json::Value, Problem> *problem;
};

Json::Value toJSON(const Experiment &experiment) {
	Json::Value result;
	result["planner"] = experiment.planner->first;
	result["problem"] = experiment.problem->first;
	return result;
}

/**
 * @brief Generates a vector of problems containing pairs of JSON values and Problem objects.
 * @param robot A shared pointer to a RobotModelConst object.
 * @param numRepetitions The number of repetitions to be performed.
 * @param modelNames A vector of model names for which the problems will be generated.
 * @param applesCounts A vector of integers representing the number of apples for each problem.
 * @return A vector of pairs containing a JSON value and a Problem object.
 */
std::vector<std::pair<Json::Value, Problem>> generateProblems(moveit::core::RobotModelConstPtr robot,
															  int numRepetitions,
															  const std::vector<std::string> &modelNames,
															  const std::vector<int> &applesCounts) {

	// Get the required scenes for each model name.
	const auto scenes = scenes_for_trees(modelNames, 500);

	using namespace ranges;

	// Generate a range of repetition indices.
	auto repIds = ranges::views::iota(0, numRepetitions);

	// Generate a vector of problems by calculating the cartesian product of repIds, applesCounts, and scenes.
	std::vector<std::pair<Json::Value, Problem>> problems =
			ranges::views::cartesian_product(repIds, applesCounts, scenes) |
			ranges::views::transform([&](const auto &pair) {

				const auto &[repId, nApples, scene] = pair;

				// Get a random sample of apples.
				AppleTreePlanningScene reduced_scene{.scene_msg = scene.scene_msg, .apples = scene.apples};

				assert(reduced_scene.apples.size() >= nApples);

				// Shuffle the apples to create a random sample.
				std::shuffle(reduced_scene.apples.begin(), reduced_scene.apples.end(), std::mt19937(repId));

				// Resize the apples vector to keep only the required number of apples.
				reduced_scene.apples.resize(nApples);

				// Get a random start state.
				auto start = randomStateOutsideTree(robot, repId + 991 * nApples);

				// Create a JSON value to represent the problem.
				Json::Value problem;
				problem["repId"] = repId;
				problem["nApples"] = nApples;
				problem["scene"] = scene.scene_msg->name;

				// Return the pair containing the JSON value and the Problem object.
				return std::make_pair(problem, Problem{.start = std::move(start), .scene = std::move(reduced_scene)});
			}) | ranges::to_vector;

	return problems;
}

template<typename SP>
std::unique_ptr<ApproachPlanningMethods<SP>> makeApproachPlanningMethods(const ompl::base::SpaceInformationPtr &si) {
	return std::make_unique<MakeshiftPrmApproachPlanningMethods<SP>>(si);
}

//template<typename SP>
//using WorkspaceShellFn = std::function<std::shared_ptr<WorkspaceShell<SP>>(const AppleTreePlanningScene &)>;
//
//std::shared_ptr<OmplShellSpace<ConvexHullPoint>>
//paddedOmplChullShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
//	auto workspaceShell = horizontalAdapter<ConvexHullPoint>(cuttingPlaneConvexHullAroundLeaves(scene_info, 0.1, 1.0));
//	return OmplShellSpace<ConvexHullPoint>::fromWorkspaceShell(workspaceShell, si);
//};
//
//template<typename SP>
//StaticPlannerAllocatorFn mkPlannerAllocator(WorkspaceShellFn<SP> mkShell) {
//	return [mkShell](const ompl::base::SpaceInformationPtr &si) {
//		return std::make_shared<ShellPathPlanner<SP>>(mkShell,makeApproachPlanningMethods<SP>(si),true);
//	};
//}

template<typename SP> using WorkspaceShellFn = std::function<std::shared_ptr<WorkspaceShell<SP>>(const AppleTreePlanningScene &)>;

template<typename T>
StaticPlannerAllocatorFn
createPlannerPair(const WorkspaceShellFn<T> &workspaceFn, DistancePredictionStrategy strategy) {
	return [strategy, workspaceFn](const ompl::base::SpaceInformationPtr &si) -> std::shared_ptr<MultiGoalPlanner> {
		MkOmplShellFn<T> mkShell = [&](const AppleTreePlanningScene &scene, const ompl::base::SpaceInformationPtr &si) {
			auto workspaceShell = horizontalAdapter<T>(workspaceFn(scene));
			return OmplShellSpace<T>::fromWorkspaceShell(workspaceShell, si);
		};

		auto approach_methods = makeApproachPlanningMethods<T>(si);

		return std::make_shared<ShellPathPlanner<T>>(mkShell, std::move(approach_methods), true, strategy);
	};
}

int main(int argc, char **argv) {

	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

	// Load the robot model.
	const auto robot = loadRobotModel();

	const size_t N_RUNS = 50;

	const auto problems = generateProblems(robot, N_RUNS, {"appletree", "lemontree2", "orangetree4"}, {10, 50, 100});

	using namespace ranges;

	std::vector<std::pair<Json::Value, StaticPlannerAllocatorFn>> planners;

	for (const auto strategy: {DistancePredictionStrategy::SHELL_PATH_LENGTH,
							   DistancePredictionStrategy::UNOPTIMIZED_GTG_PATH_LENGTH}) {

		const std::string &strategy_name =
				strategy == DistancePredictionStrategy::SHELL_PATH_LENGTH ? "shell_path_length" : "gtg_path_length";

		Json::Value params;
		params["strategy"] = strategy_name;

		params["shell"] = "Sphere";
		planners.emplace_back(params, createPlannerPair<Eigen::Vector3d>([&](const AppleTreePlanningScene &scene) {
			return paddedSphericalShellAroundLeaves(scene, 0.1);
		}, strategy));

		params["shell"] = "Cutting Plane Chull";
		planners.emplace_back(params, createPlannerPair<ConvexHullPoint>([&](const AppleTreePlanningScene &scene) {
			return cuttingPlaneConvexHullAroundLeaves(scene, 0.1, 1.0);
		}, strategy));

		params["shell"] = "CGAL Chull";
		planners.emplace_back(params, createPlannerPair<CGALMeshShellPoint>([&](const AppleTreePlanningScene &scene) {
			return convexHullAroundLeavesCGAL(scene, 1.0, 0.1);
		}, strategy));

		params["shell"] = "Cylinder";
		planners.emplace_back(params, createPlannerPair<CylinderShellPoint>([&](const AppleTreePlanningScene &scene) {
			return paddedCylindricalShellAroundLeaves(scene, 0.1);
		}, strategy));

	}


	// Take the carthesian product of the different planners and problems,
	// making it so that every planner is tested on every problem.
	std::vector<Experiment> experiments;

	for (auto &problem: problems) {
		for (auto &planner: planners) {
			experiments.push_back(Experiment{.planner= &planner, .problem= &problem});
		}
	}

	// Run the experiments in parallel.
	runExperimentsParallelRecoverable<Experiment>(experiments, [&](const Experiment &experiment) {

		// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
		// So, we just re-create the state space every time just to be safe.
		auto ss = omplStateSpaceForDrone(robot, TRANSLATION_BOUND);

		// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
		// we'll need to copy this for every thread
		auto si = loadSpaceInformation(ss, experiment.problem->second.scene);

		// Allocate the planner.
		auto ompl_planner = experiment.planner->second(si);

		ompl::base::ScopedState<> start_state(ss);
		ss->copyToOMPLState(start_state.get(), experiment.problem->second.start);

		auto goals = experiment.problem->second.scene.apples |
					 ranges::views::transform([&](const auto &apple) -> ompl::base::GoalPtr {
						 return std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center);
					 }) | ranges::to_vector;

		// use OMPL non-terminating condition
		auto ptc = ompl::base::plannerNonTerminatingCondition();

		// Record the start time.
		auto start_time = std::chrono::high_resolution_clock::now();
		auto eval = ompl_planner->plan(si, start_state.get(), goals, experiment.problem->second.scene, ptc);
		auto end_time = std::chrono::high_resolution_clock::now();

		// Check whether the path segments actually connect to each other.
		for (int i = 0; i + 1 < eval.segments.size(); ++i) {
			const auto &segment = eval.segments[i];
			const auto &next_segment = eval.segments[i + 1];
			assert(segment.path_.getStateCount() > 0);
			assert(si->distance(segment.path_.getState(segment.path_.getStateCount() - 1),
								next_segment.path_.getState(0)) < 1e-3);
		}

		Json::Value result;
		result["n_visited"] = eval.segments.size();
		result["time"] = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		result["total_path_length"] = eval.length();

		return result;

	}, "analysis/data/static_shapes_techniques.json", 8, std::thread::hardware_concurrency(), 42);

	return 0;
}

