#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../DynamicGoalsetPlanningProblem.h"
#include "../exploration/ColorEncoding.h"
#include "../planner_allocators.h"
#include "../planners/CachingDynamicPlanner.h"
#include "../utilities/run_experiments.h"

#include <vtkProperty.h>

#include "../static_problem_generation.h"

Json::Value runExperiment(const StaticPlannerAllocatorFn &planner, const Problem &problem) {

	// *Somewhere* in the state space is something that isn't thread-safe despite const-ness.
	// So, we just re-create the state space every time just to be safe.
	auto ss = omplStateSpaceForDrone(problem.start.getRobotModel());

	// Collision-space is "thread-safe" by using locking. So, if we want to get any speedup at all,
	// we'll need to copy this for every thread
	auto si = loadSpaceInformation(ss, problem.scene);

	// Allocate the planner.
	auto ompl_planner = planner(si);

	ompl::base::ScopedState<> start_state(ss);
	ss->copyToOMPLState(start_state.get(), problem.start);

	auto goals = problem.scene.apples |
				 ranges::views::transform([&](const auto &apple) -> ompl::base::GoalPtr {
					 return std::make_shared<DroneEndEffectorNearTarget>(si, 0.05, apple.center);
				 }) | ranges::to_vector;

	// use OMPL non-terminating condition
	auto ptc = ompl::base::plannerNonTerminatingCondition();

	// Record the start time.
	auto start_time = std::chrono::high_resolution_clock::now();
	auto eval = ompl_planner->plan(si, start_state.get(), goals, problem.scene, ptc);
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

}

int main(int argc, char **argv) {

	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

	// Load the robot model.
	const auto robot = loadRobotModel();

	auto model_names = getTreeModelNames();

	// Truncate to 5 trees for testing.
	model_names.resize(10);

	const auto problems = generateStaticPlanningProblems(robot, 10, model_names);

	using namespace ranges;

	// The different planners to test.
	std::vector<std::pair<Json::Value, StaticPlannerAllocatorFn>> planners = {
			{Json::Value("sphere"), makeShellBasedPlanner<Eigen::Vector3d>(omplSphereShell)},
			{Json::Value("cuttingplane"), makeShellBasedPlanner<ConvexHullPoint>(cuttingPlaneChullShell)},
			{Json::Value("cgal"), makeShellBasedPlanner<CGALMeshShellPoint>(cgalPlaneChullShell)},
			{Json::Value("cylinder"), makeShellBasedPlanner<CylinderShellPoint>(cylinderShell)}
	};


	// Run the experiments in parallel.
	runPlannersOnProblemsParallelRecoverable<StaticPlannerAllocatorFn,Problem>(
			planners,
			problems,
			runExperiment,
			"analysis/data/shell_comparison_RAL.json",
			8,
			4,
			42);

	return 0;
}