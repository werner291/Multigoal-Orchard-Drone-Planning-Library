#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../DynamicGoalsetPlanningProblem.h"
#include "../planner_allocators.h"
#include "../utilities/run_experiments.h"

#include "../static_problem_generation.h"
#include "../run_static.h"
#include "../planners/MultigoalPrmStar.h"

StaticPlannerAllocatorFn makeTspOverPRMStarPlanner(double prmBuildTime, size_t samplesPerGoal, bool optimizeSegments) {
	return [=](const ompl::base::SpaceInformationPtr &) -> StaticPlannerPtr {
		return std::make_shared<MultigoalPrmStar>(prmBuildTime, samplesPerGoal, optimizeSegments);
	};
}

int main(int argc, char **argv) {

	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

	// Load the robot model.
	const auto robot = loadRobotModel();

	auto model_names = getTreeModelNames();

	const auto problems = generateStaticPlanningProblems(robot, 10, model_names);

	std::cout << "Generated " << problems.size() << " static planning problems." << std::endl;

	using namespace ranges;

	std::vector<std::pair<Json::Value, StaticPlannerAllocatorFn>> planners;

	// Parameters for shell based planners
	std::vector<double> shell_approach_max_t_values = {0.1, 0.25, 0.5, 1.0};

	for (auto approach_max_t : shell_approach_max_t_values) {
		Json::Value planner_params(Json::objectValue);

		planner_params["name"] = "shell_based";

		planner_params["approach_max_t"] = approach_max_t;

		planner_params["shell_type"] = "minimum_enclosing_sphere";
		planners.emplace_back(planner_params, makeShellBasedPlanner<Eigen::Vector3d>(minimumEnclosingSphereShell, approach_max_t));

		planner_params["shell_type"] = "cgal_convex_hull";
		planners.emplace_back(planner_params, makeShellBasedPlanner<CGALMeshShellPoint>(cgalChullShell, approach_max_t));
	}

	// Parameters for tsp over prm planners
	std::vector<double> prm_build_time_values = {1,2,5,10,15};//,20};
	std::vector<size_t> samples_per_goal_values = {1, 2, 3, 4, 6, 8, 10};//, 6, 7, 8, 9, 10};

	for (auto prm_build_time : prm_build_time_values) {
		for (auto samples_per_goal : samples_per_goal_values) {
			Json::Value planner_params(Json::objectValue);
			planner_params["name"] = "tsp_over_prm*";
			planner_params["prm_build_time"] = prm_build_time;
			planner_params["samples_per_goal"] = samples_per_goal;

			planners.emplace_back(planner_params, makeTspOverPRMStarPlanner(prm_build_time, samples_per_goal, true));
		}
	}

	std::cout << "Running with " << planners.size() << " planners and " << problems.size() << " problems." << std::endl;

	// Run the experiments in parallel.
	runPlannersOnProblemsParallelRecoverable<StaticPlannerAllocatorFn, Problem>(planners,
																				problems,
																				runPlannerOnStaticProblem,
																				"analysis/data/shell_comparison_RAL_with_prm3.json",
																				8,
																				std::thread::hardware_concurrency(),
																				42);

	return 0;
}