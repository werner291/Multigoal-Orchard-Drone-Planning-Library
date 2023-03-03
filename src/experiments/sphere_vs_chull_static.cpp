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

using DynamicPlannerAllocatorFn = std::function<std::shared_ptr<DynamicMultiGoalPlanner>(const ompl::base::SpaceInformationPtr &)>;

std::shared_ptr<OmplShellSpace<Eigen::Vector3d>>
paddedOmplSphereShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<Eigen::Vector3d>(paddedSphericalShellAroundLeaves(scene_info, 0.1));
	return OmplShellSpace<Eigen::Vector3d>::fromWorkspaceShell(workspaceShell, si);
};

std::shared_ptr<OmplShellSpace<ConvexHullPoint>>
paddedOmplChullShell(const AppleTreePlanningScene &scene_info, const ompl::base::SpaceInformationPtr &si) {
	auto workspaceShell = horizontalAdapter<ConvexHullPoint>(convexHullAroundLeaves(scene_info, 0.1, 1.0));
	return OmplShellSpace<ConvexHullPoint>::fromWorkspaceShell(workspaceShell, si);
};

struct Problem {
	moveit::core::RobotState start;
	std::vector<Apple> apples;
};

using StaticPlannerAllocatorFn = std::function<std::shared_ptr<MultiGoalPlanner>(const ompl::base::SpaceInformationPtr &)>;

struct Experiment {
	std::pair<std::string, StaticPlannerAllocatorFn> *planner;
	std::pair<Json::Value, Problem> *problem;
};

Json::Value toJSON(const Experiment &experiment) {
	Json::Value result;
	result["planner"] = experiment.planner->first;
	result["problem"] = experiment.problem->first;
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

#define STATISTICS

#ifdef STATISTICS
	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

	auto repIds = ranges::views::iota(0, 5);

	// Numbers of apples to throw at the planner.
	const auto nApples = {100};//{10, 50, 100};

	std::vector<std::pair<Json::Value, Problem>> problems =
			ranges::views::cartesian_product(repIds, nApples) | ranges::views::transform([&](const auto &pair) {
				const auto &[repId, nApples] = pair;

				// Get a random sample of apples.
				auto apples = scene.apples;
				std::shuffle(apples.begin(), apples.end(), std::mt19937(repId));
				apples.resize(nApples);

				// Get a random start state.
				auto start = randomStateOutsideTree(robot, repId + 991 * nApples);

				Json::Value problem;
				problem["repId"] = repId;
				problem["nApples"] = nApples;

				return std::make_pair(problem, Problem{.start = std::move(start), .apples = std::move(apples)});
			}) | ranges::to_vector;


	StaticPlannerAllocatorFn make_sphere_planner = [](const ompl::base::SpaceInformationPtr &si) {
		return std::make_shared<ShellPathPlanner<Eigen::Vector3d >>(paddedOmplSphereShell,
																	std::make_unique<MakeshiftPrmApproachPlanningMethods<Eigen::Vector3d>>(
																			si),
																	true);
	};

	StaticPlannerAllocatorFn make_chull_planner = [](const ompl::base::SpaceInformationPtr &si) {
		return std::make_shared<ShellPathPlanner<ConvexHullPoint>>(paddedOmplChullShell,
																   std::make_unique<MakeshiftPrmApproachPlanningMethods<ConvexHullPoint>>(
																		   si),
																   true);
	};

	// The different planners to test.
	std::vector<std::pair<std::string, StaticPlannerAllocatorFn>> planners = {
			{"sphere", make_sphere_planner},
			//			{"chull",  make_chull_planner}
	};

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
		auto ss = omplStateSpaceForDrone(robot);

		// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
		// we'll need to copy this for every thread
		auto si = loadSpaceInformation(ss, scene);

		// Allocate the planner.
		auto ompl_planner = experiment.planner->second(si);

		ompl::base::ScopedState<> start_state(ss);
		ss->copyToOMPLState(start_state.get(), experiment.problem->second.start);

		auto goals = experiment.problem->second.apples |
					 ranges::views::transform([&](const auto &apple) -> ompl::base::GoalPtr {
						 return std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center);
					 }) | ranges::to_vector;

		// use OMPL non-terminating condition
		auto ptc = ompl::base::plannerNonTerminatingCondition();

		// Record the start time.
		auto start_time = std::chrono::high_resolution_clock::now();
		auto eval = ompl_planner->plan(si, start_state.get(), goals, scene, ptc);
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

	}, "analysis/data/static_sphere_vs_chull.json", 8, 4, 42);
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

	return 0;
}
