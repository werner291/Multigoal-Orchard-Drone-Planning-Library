#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../DynamicGoalsetPlanningProblem.h"
#include "../planner_allocators.h"
#include "../utilities/run_experiments.h"

#include "../static_problem_generation.h"
#include "../run_static.h"
#include "../planners/MultigoalPrmStar.h"

StaticPlannerAllocatorFn
makeTspOverPRMStarPlanner(double prmBuildTime, size_t samplesPerGoal, bool optimizeSegments, double aabb_padding) {
	return [=](const ompl::base::SpaceInformationPtr &) -> StaticPlannerPtr {
		return std::make_shared<MultigoalPrmStar>(prmBuildTime, samplesPerGoal, optimizeSegments, aabb_padding);
	};
}

int main(int argc, char **argv) {

	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

	// Load the robot model.
	const auto robot = loadRobotModel();

	std::vector<std::string> model_names = {"appletree2"};//getTreeModelNames();

	const auto problems = generateStaticPlanningProblems(robot, 5, model_names);

	std::cout << "Generated " << problems.size() << " static planning problems." << std::endl;

	using namespace ranges;

	std::vector<std::pair<Json::Value, StaticPlannerAllocatorFn>> planners;

	// Parameters for tsp over prm planners
	std::vector<double> prm_build_time_values = {2,5,10};//{1,2,5,10,15};//,20};
	std::vector<size_t> samples_per_goal_values = {5};//{1, 2, 3, 4, 6, 8, 10};//, 6, 7, 8, 9, 10};
	std::vector<double> outside_tree_margins = {1.0, 2.0, 5.0, 10.0};

	for (auto prm_build_time : prm_build_time_values) {
		for (auto samples_per_goal : samples_per_goal_values) {
			for (auto margin: outside_tree_margins) {
				Json::Value planner_params(Json::objectValue);
				planner_params["name"] = "tsp_over_prm*";
				planner_params["prm_build_time"] = prm_build_time;
				planner_params["samples_per_goal"] = samples_per_goal;
				planner_params["optimize_segments"] = true;
				planner_params["outside_tree_margin"] = margin;

				planners.emplace_back(planner_params,
									  makeTspOverPRMStarPlanner(prm_build_time, samples_per_goal, true, margin));
			}
		}
	}

	std::cout << "Running with " << planners.size() << " planners and " << problems.size() << " problems." << std::endl;

	// Run the experiments in parallel.
	runPlannersOnProblemsParallelRecoverable<StaticPlannerAllocatorFn, Problem>(planners,
																				problems,
																				runPlannerOnStaticProblem,
																				"analysis/data/tsp_over_prm_margin.json",
																				8,
																				std::thread::hardware_concurrency(),
																				42);

	return 0;
}