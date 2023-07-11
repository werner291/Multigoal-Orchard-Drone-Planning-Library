#include <array>
#include <string>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/algorithm/count.hpp>
#include <range/v3/range/conversion.hpp>

#include "../CanSeeApple.h"
#include "../utilities/discoverability_specifications.h"
#include "../utilities/experiment_utils.h"
#include "../utilities/cgal_utils.h"
#include "../planner_allocators.h"
#include "utilities/default_occlusion_models.h"
#include "../DynamicGoalsetPlanningProblem.h"
#include "../dynamic_goalset_experiment.h"
#include "../utilities/run_experiments.h"

const std::array<Proportions,5> probs = {
		Proportions {
				.fraction_true_given = 1.0,
				.fraction_false_given = 0.0,
				.fraction_discoverable = 0.0
		},
		Proportions {
				.fraction_true_given = 0.5,
				.fraction_false_given = 0.0,
				.fraction_discoverable = 0.5
		},
//		Proportions {
//				.fraction_true_given = 0.0,
//				.fraction_false_given = 0.0,
//				.fraction_discoverable = 1.0
//		},
//		Proportions {
//				.fraction_true_given = 0.5,
//				.fraction_false_given = 0.5,
//				.fraction_discoverable = 0.0
//		},
//		Proportions {
//				.fraction_true_given = 0.0,
//				.fraction_false_given = 0.5,
//				.fraction_discoverable = 0.5
//		},
};

int main(int argc, char **argv) {

	int N_TREES = 1;
	const int REPETITIONS = 1;

	// Load the apple tree meshes.
	auto models = loadAllTreeModels(INT_MAX, 600);

	// Load the robot model.
	const auto robot = loadRobotModel();

	ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

	auto repIds = ranges::views::iota(0, REPETITIONS);

	using ShellPoint = mgodpl::cgal_utils::CGALMeshPointAndNormal;

	DMGPlannerAllocatorFn sp = static_planner<ShellPoint>(cgalChullShell);

	std::array<std::pair<std::string, DMGPlannerAllocatorFn>, 9> PLANNERS_TO_TEST = {
			// A planner that ignores the dynamic goalset, only planning to the initially-given apples.
			// It will obviously not ve very optimal, but will serve as a baseline.
			std::make_pair("change_ignoring", sp),
			// A planner that adds new goals to a "batch" to be replanned to after the current
			// path has been completed.
			{"batch_replanner", batch_replanner<ShellPoint>(cgalChullShell)},
			// A planner that uses the dynamic goalset, but uses a greedy approach to insert new goals in the visitation order.
			{"dynamic_planner_lci", dynamic_planner_simple_reorder_sphere<ShellPoint>(SimpleIncrementalTSPMethods::Strategy::LeastCostlyInsertion,cgalChullShell)},
			// A planner that inserts new goals simply at the end of the tour
			{"dynamic_planner_LIFO", dynamic_planner_simple_reorder_sphere<ShellPoint>(SimpleIncrementalTSPMethods::Strategy::LastInFirstOut,cgalChullShell)},
			// A planner that inserts new goals simply at the beginning of the tour
			{"dynamic_planner_FIFO", dynamic_planner_simple_reorder_sphere<ShellPoint>(SimpleIncrementalTSPMethods::Strategy::FirstInFirstOut,cgalChullShell)},
			// A planner that puts goals at the second place in the order.
			{"dynamic_planner_FISO", dynamic_planner_simple_reorder_sphere<ShellPoint>(SimpleIncrementalTSPMethods::Strategy::FirstInSecondOut,cgalChullShell)},
			// A planner that randomizes the order of the goals.
			{"dynamic_planner_random", dynamic_planner_simple_reorder_sphere<ShellPoint>(SimpleIncrementalTSPMethods::Strategy::Random,cgalChullShell)},
			// Same as dynamic_planner_fre, but with an initial orbit around the tree to discover some of the dynamic goals
			{"dynamic_planner_initial_orbit", dynamic_planner_initial_orbit<ShellPoint>(cgalChullShell)},
			// A planner that uses the dynamic goalset, and completely reorders the visitation order from scratch every time a goal is added.
			{"dynamic_planner_fre", dynamic_planner_fre<ShellPoint>(cgalChullShell)},
	};

	// Generate a set of problems based on the carthesian product of the above ranges.
	auto problems =
			ranges::views::cartesian_product(models, repIds, probs, OCCLUSION_MODELS) |
			ranges::views::transform([&](const auto &pair) -> std::pair<Json::Value,DynamicGoalsetPlanningProblem> {

				// Generate a problem for every unique combination of repetition ID,
				// discoverability degree, total number of apples and occlusion model.
				auto &[scene, repId, prob, occlusion_model] = pair;

				// Translate the discoverability degree into a vector of discoverability types/
				const auto discoverability = generateAppleDiscoverability(prob,
																		  repId,
																		  scene.fruit_meshes.size());

				// Create a JSON object containing the parameters of the problem for later reference.
				Json::Value problem_params;
				problem_params["n_given"] = (int) ranges::count(discoverability, AppleDiscoverabilityType::GIVEN);
				problem_params["n_discoverable"] = (int) ranges::count(discoverability, AppleDiscoverabilityType::DISCOVERABLE);
				problem_params["n_false"] = (int) ranges::count(discoverability, AppleDiscoverabilityType::FALSE);
				problem_params["n_total"] = (int) scene.fruit_meshes.size();
				problem_params["visibility_model"] = occlusion_model.first;

				// Create the problem, to be solved by the planners.
				DynamicGoalsetPlanningProblem problem {
					.start_state = randomStateOutsideTree(robot,repId),
					.tree_meshes = &scene,
					.apple_discoverability= discoverability,
					.can_see_apple = &occlusion_model.second
				};

				// Return the problem parameters and the problem itself.
				return {problem_params, problem};

			}) | ranges::to_vector;



	// Take the carthesian product of the different planners and problems,
	// making it so that every planner is tested on every problem.
	auto experiments = ranges::views::cartesian_product(PLANNERS_TO_TEST, problems) | ranges::views::transform([](const auto &pair) {
		const auto &[planner, problem] = pair;
		return Experiment {
			.planner= &planner,
			.problem= &problem
		};
	}) | ranges::to_vector;

	// Run the experiments in parallel.
	runExperimentsParallelRecoverable<Experiment>(experiments, [&](const Experiment &experiment) {
		return runDynamicPlannerExperiment(robot, experiment);
	}, "analysis/data/dynamic_log_ral.json", 32, std::thread::hardware_concurrency(), 42);

}