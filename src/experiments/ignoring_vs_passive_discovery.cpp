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

void runDynamicPlannerExperiments(const bool KEEP_SEGMENTS,
								  std::vector<TreeMeshes> &models,
								  const moveit::core::RobotModelPtr &robot,
								  const int REPETITIONS,
								  std::vector<std::pair<std::string, DMGPlannerAllocatorFn>> &PLANNERS_TO_TEST,
								  const std::array<std::pair<std::string, CanSeeAppleFnFactory>, 1> &OCCLUSION_MODELS_TO_TEST,
								  const std::string &resultsFile,
								  const std::vector<Proportions> &discoverabilityProportions);

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

enum ExperimentMode {
	THREE_TREE_MODE,
	ACTIVE_EXPLORATION_CLI_MODE,
	IMPACT_OF_DELETION_MODE,
};

const std::string THREE_TREE_STR = "lci_vs_fre";
const std::string THOUSAND_TREE_STR = "cheaper_planners";
const std::string IMPACT_OF_DELETION_STR = "impact_of_deletion";

std::optional<ExperimentMode> getExperimentMode(const std::string &mode_arg) {
	if (mode_arg == THREE_TREE_STR) {
		return THREE_TREE_MODE;
	} else if (mode_arg == THOUSAND_TREE_STR) {
		return ACTIVE_EXPLORATION_CLI_MODE;
	} else if (mode_arg == IMPACT_OF_DELETION_STR) {
		return IMPACT_OF_DELETION_MODE;
	}
	return {};
}

void printUsage(char *const *argv) {
	std::cerr << "Usage: " << argv[0] << " <mode>" << std::endl;
	std::cerr << "Where <mode> can be: " << std::endl;
	std::cerr << "  " << THREE_TREE_STR << ": Compare LCI and FRE on 3 trees." << std::endl;
	std::cerr << "  " << THOUSAND_TREE_STR << ": Compare LCI and Initial Orbit on 1000 trees." << std::endl;
	std::cerr << "  " << IMPACT_OF_DELETION_STR << ": Compare LCI and FRE on 3 trees, but with deletion." << std::endl;
}

int main(int argc, char **argv) {

	if (argc < 2) {
		printUsage(argv);
		return 1;
	}

	auto mode = getExperimentMode(argv[1]);

	if (!mode.has_value()) {
		std::cerr << "Invalid mode specified." << std::endl;
		printUsage(argv);
		return 1;
	}


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

	if (*mode == THREE_TREE_MODE) {

		const int REPETITIONS = 10;
		// Common Constants
		const bool KEEP_SEGMENTS = false;

		const auto props = mgodpl::utilities::linspace(
				mgodpl::dynamic_goals::ALL_GIVEN,
				mgodpl::dynamic_goals::ALL_DISCOVERABLE,
				4
		);

		const int N_TREES = 3;
		const int MAX_FRUIT = 100;
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
									 "analysis/data/dynamic_log_icra2024_3trees.json",
									 props);

	} else if (mode == ACTIVE_EXPLORATION_CLI_MODE) {

		const auto props = mgodpl::utilities::linspace(
				mgodpl::dynamic_goals::ALL_GIVEN,
				mgodpl::dynamic_goals::ALL_DISCOVERABLE,
				4
		);
		const int REPETITIONS = 10;

		const int N_TREES = 1000;
		const int MAX_FRUIT = 600;
		// Common Constants
		const bool KEEP_SEGMENTS = false;
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
									 "analysis/data/dynamic_log_icra2024_1000trees.json",
									 props);

	} else if (mode == IMPACT_OF_DELETION_MODE) {

		// I want to essentially construct a triangle of proportions.
		// The three vertices are:
		// 1. All apples are given (ALL_GIVEN)
		// 2. All apples are discoverable (ALL_DISCOVERABLE)
		// 3. All apples are false (ALL_FALSE)

		// Then, we generate a set of proportions that are linearly interpolated between these three vertices.

		using namespace mgodpl::utilities;
		using namespace mgodpl::dynamic_goals;

		std::set<Proportions> props {};

		// A set of proportions featuring no false entries.
		for (const double discoverable : linspace(0.0, 1.0, 5)) {
			props.insert(Proportions {
				.fraction_true_given = 1.0 - discoverable,
				.fraction_false_given = 0.0,
				.fraction_discoverable = discoverable
			});
		}

		// For 50% and 0% true-given, different values of false-given.
		for (const double true_given : linspace(0.0, 0.5, 2)) {
			for (const double false_in_given: linspace(0.0, 1.0, 6)) {

				double false_given = false_in_given * (1.0 - true_given);
				double discoverable = 1.0 - true_given - false_given;

				props.insert(Proportions {
					.fraction_true_given = true_given,
					.fraction_false_given = false_given,
					.fraction_discoverable = discoverable
				});
			}
		}

		std::vector<Proportions> props_vec {props.begin(), props.end()};

		for (const auto &prop : props_vec) {
			std::cout << "True-given: " << prop.fraction_true_given << ", False-given: " << prop.fraction_false_given << ", Discoverable: " << prop.fraction_discoverable << std::endl;
		}

		const int REPETITIONS = 5;
		const int N_TREES = 3;
		const int MAX_FRUIT = 100;
		// Common Constants
		const bool KEEP_SEGMENTS = true;
		auto models = loadAllTreeModels(N_TREES, MAX_FRUIT);

		std::vector<std::pair<std::string, DMGPlannerAllocatorFn>> PLANNERS_TO_TEST = {
				{"dynamic_planner_fre", dynamicPlannerFREFunction},
				{"dynamic_planner_lci", dynamicPlannerLCIFunction},
				{"dynamic_planner_initial_orbit", dynamicPlannerInitialOrbitFunction}
		};

		runDynamicPlannerExperiments(KEEP_SEGMENTS,
									 models,
									 robot,
									 REPETITIONS,
									 PLANNERS_TO_TEST,
									 OCCLUSION_MODELS_TO_TEST,
									 "analysis/data/dynamic_log_icra2024_3trees_deletion_smalltrees.json",
									 props_vec);

	}


}

void runDynamicPlannerExperiments(const bool KEEP_SEGMENTS,
								  std::vector<TreeMeshes> &models,
								  const moveit::core::RobotModelPtr &robot,
								  const int REPETITIONS,
								  std::vector<std::pair<std::string, DMGPlannerAllocatorFn>> &PLANNERS_TO_TEST,
								  const std::array<std::pair<std::string, CanSeeAppleFnFactory>, 1> &OCCLUSION_MODELS_TO_TEST,
								  const std::string &resultsFile,
								  const std::vector<Proportions> &discoverabilityProportions) {// Generate a set of problems based on the carthesian product of the above ranges.

	auto repIds = ranges::views::iota(0, REPETITIONS);

	auto problems =
			ranges::views::cartesian_product(models, repIds, discoverabilityProportions, OCCLUSION_MODELS_TO_TEST) |
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
		return runDynamicPlannerExperiment(robot, experiment, KEEP_SEGMENTS, std::chrono::minutes(10));
	}, resultsFile, 32, std::thread::hardware_concurrency(), 42);
}
