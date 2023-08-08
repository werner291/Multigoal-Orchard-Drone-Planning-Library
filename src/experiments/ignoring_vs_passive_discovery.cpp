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

const std::array<Proportions,3> probs = {
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
		Proportions {
				.fraction_true_given = 0.0,
				.fraction_false_given = 0.0,
				.fraction_discoverable = 1.0
		},
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

void runDynamicPlannerExperiments(const bool KEEP_SEGMENTS,
								  std::vector<TreeMeshes> &models,
								  const moveit::core::RobotModelPtr &robot,
								  const int REPETITIONS,
								  std::vector<std::pair<std::string, DMGPlannerAllocatorFn>> &PLANNERS_TO_TEST,
								  const std::array<std::pair<std::string, CanSeeAppleFnFactory>, 1> &OCCLUSION_MODELS_TO_TEST,
								  const std::string &resultsFile);

using ShellPoint = mgodpl::cgal_utils::CGALMeshPointAndNormal;

DMGPlannerPtr dynamicPlannerInitialOrbitFunction(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<InitialOrbitPlanner>(
			std::make_shared<CachingDynamicPlanner<ShellPoint>>(
					std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si),
					mgodpl::tsp_utils::IncrementalTSPMethods {
							.initial_ordering = mgodpl::tsp_utils::determine_tsp_order_ortools,
							.update_methods = {
									.update_ordering_with_insertion = mgodpl::tsp_utils::insert_least_costly,
									.update_ordering_with_removal = mgodpl::tsp_utils::removalBySimpleDeletion
							}
					},
					cgalChullShell
			)
	);
}

DMGPlannerPtr dynamicPlannerLCIFunction(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<ShellPoint>>(
			std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si),
			mgodpl::tsp_utils::IncrementalTSPMethods {
					.initial_ordering = mgodpl::tsp_utils::determine_tsp_order_ortools,
					.update_methods = {
							.update_ordering_with_insertion = mgodpl::tsp_utils::insert_least_costly,
							.update_ordering_with_removal = mgodpl::tsp_utils::removalBySimpleDeletion
					}
			},
			cgalChullShell
	);
}

DMGPlannerPtr dynamicPlannerFREFunction(const ompl::base::SpaceInformationPtr &si) {
	return std::make_shared<CachingDynamicPlanner<ShellPoint>>(
			std::make_unique<MakeshiftPrmApproachPlanningMethods<ShellPoint>>(si),
			mgodpl::tsp_utils::incremental_tsp_order_ortools_always_reorder(),
			cgalChullShell
	);
}

int main(int argc, char **argv) {

	enum ExperimentMode {
		THREE_TREE_MODE,
		THOUSAND_TREE_MODE
	};

	const ExperimentMode mode = THOUSAND_TREE_MODE; // Set this to either THREE_TREE_MODE or THOUSAND_TREE_MODE

	// Common Constants
	const int REPETITIONS = 3;
	const bool KEEP_SEGMENTS = false;

	// Load the robot model.
	const auto robot = loadRobotModel();
	ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
	DMGPlannerAllocatorFn sp = static_planner<ShellPoint>(cgalChullShell, 0.5);

	// OCCLUSION MODELS (Common for both modes)
	const std::array<std::pair<std::string, CanSeeAppleFnFactory>, 1> OCCLUSION_MODELS_TO_TEST = {
			std::make_pair("mesh_occlusion_end_effector", [](const TreeMeshes &meshes) {
				return mesh_occludes_vision(meshes.leaves_mesh, "end_effector");
			})
	};

	if (mode == THREE_TREE_MODE) {

		const int N_TREES = 3;
		const int MAX_FRUIT = 200;
		auto models = loadAllTreeModels(N_TREES, MAX_FRUIT);

		std::vector<std::pair<std::string, DMGPlannerAllocatorFn>> PLANNERS_TO_TEST = {
				{"dynamic_planner_lci", dynamicPlannerLCIFunction},
				{"dynamic_planner_fre", dynamicPlannerFREFunction}
		};

		runDynamicPlannerExperiments(KEEP_SEGMENTS,
									 models,
									 robot,
									 REPETITIONS,
									 PLANNERS_TO_TEST,
									 OCCLUSION_MODELS_TO_TEST,
									 "analysis/data/dynamic_log_icra2024_3trees.json");

	} else if (mode == THOUSAND_TREE_MODE) {

		const int N_TREES = 1000;
		const int MAX_FRUIT = 600;
		auto models = loadAllTreeModels(N_TREES, MAX_FRUIT);

		std::vector<std::pair<std::string, DMGPlannerAllocatorFn>> PLANNERS_TO_TEST = {
				{"dynamic_planner_lci", dynamicPlannerLCIFunction},
				{"dynamic_planner_initial_orbit", dynamicPlannerInitialOrbitFunction}
		};

		runDynamicPlannerExperiments(KEEP_SEGMENTS,
									 models,
									 robot,
									 REPETITIONS,
									 PLANNERS_TO_TEST,
									 OCCLUSION_MODELS_TO_TEST,
									 "analysis/data/dynamic_log_icra2024_1000trees.json");

	}


}

void runDynamicPlannerExperiments(const bool KEEP_SEGMENTS,
								  std::vector<TreeMeshes> &models,
								  const moveit::core::RobotModelPtr &robot,
								  const int REPETITIONS,
								  std::vector<std::pair<std::string, DMGPlannerAllocatorFn>> &PLANNERS_TO_TEST,
								  const std::array<std::pair<std::string, CanSeeAppleFnFactory>, 1> &OCCLUSION_MODELS_TO_TEST,
								  const std::string &resultsFile) {// Generate a set of problems based on the carthesian product of the above ranges.

	auto repIds = ranges::views::iota(0, REPETITIONS);

	auto problems =
			ranges::views::cartesian_product(models, repIds, probs, OCCLUSION_MODELS_TO_TEST) |
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
				problem_params["n_given"] = (int) ranges::count(discoverability, GIVEN);
				problem_params["n_discoverable"] = (int) ranges::count(discoverability, DISCOVERABLE);
				problem_params["n_false"] = (int) ranges::count(discoverability, FALSE);
				problem_params["n_total"] = (int) scene.fruit_meshes.size();
				problem_params["visibility_model"] = occlusion_model.first;
				problem_params["tree_model"] = scene.tree_name;

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
		return runDynamicPlannerExperiment(robot, experiment, KEEP_SEGMENTS);
	}, resultsFile, 32, std::thread::hardware_concurrency(), 42);
}
