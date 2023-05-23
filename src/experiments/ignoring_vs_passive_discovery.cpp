#include <range/v3/all.hpp>
#include <boost/asio.hpp>
#include <utility>

#include "../AppleTreePlanningScene.h"
#include "../utilities/experiment_utils.h"

#include "../planners/ChangeIgnoringReplannerAdapter.h"
#include "../utilities/goal_events.h"
#include "../DynamicGoalVisitationEvaluation.h"
#include "../planners/CachingDynamicPlanner.h"
#include "../utilities/run_experiments.h"

#include "../DynamicGoalsetPlanningProblem.h"
#include "../utilities/MeshOcclusionModel.h"
#include "../utilities/alpha_shape.h"
#include "../planner_allocators.h"
#include "../dynamic_goalset_experiment.h"

#include <vtkProperty.h>
#include <range/v3/view/iota.hpp>

const std::array<std::pair<std::string, CanSeeAppleFnFactory>, 12> OCCLUSION_MODELS = {
			// Basic occlusion functions
			std::make_pair<std::string, CanSeeAppleFnFactory>("omniscient", [](TreeMeshes &meshes) -> CanSeeAppleFn{return omniscient_occlusion;}), // If omniscient_occlusion doesn't need `meshes`, you can return it directly.
			{"distance", [](TreeMeshes &meshes){return distance_occlusion;}}, // Same here.

			// Field of view occlusions
			{"angle_end_effector", [](TreeMeshes &meshes){return in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector");}},
			{"angle_base_link", [](TreeMeshes &meshes){return in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link");}},

			// Mesh occlusions
			{"mesh_occlusion_end_effector", [](TreeMeshes &meshes){return mesh_occludes_vision(meshes.leaves_mesh, "end_effector");}},
			{"mesh_occlusion_base_link", [](TreeMeshes &meshes){return mesh_occludes_vision(meshes.leaves_mesh, "base_link");}},

			// Alpha shape occlusions
			{"alpha_occlusion_end_effector", [](TreeMeshes &meshes){return leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "end_effector");}},
			{"alpha_occlusion_base_link", [](TreeMeshes &meshes){return leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "base_link");}},

			// Combinations of mesh occlusion and angle occlusion
			{"mesh_and_angle_end_effector", [](TreeMeshes &meshes){
				return only_if_both(mesh_occludes_vision(meshes.leaves_mesh, "end_effector"), in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector"));
			}},
			{"mesh_and_angle_base_link", [](TreeMeshes &meshes){
				return only_if_both(mesh_occludes_vision(meshes.leaves_mesh, "base_link"), in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link"));
			}},

			// Combinations of alpha shape occlusion and angle occlusion
			{"alpha_and_angle_end_effector", [](TreeMeshes &meshes){
				return only_if_both(leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "end_effector"), in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector"));
			}},
			{"alpha_and_angle_base_link", [](TreeMeshes &meshes){
				return only_if_both(leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "base_link"), in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link"));
			}}
	};



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
		Proportions {
				.fraction_true_given = 0.0,
				.fraction_false_given = 0.0,
				.fraction_discoverable = 1.0
		},
		Proportions {
				.fraction_true_given = 0.5,
				.fraction_false_given = 0.5,
				.fraction_discoverable = 0.0
		},
		Proportions {
				.fraction_true_given = 0.0,
				.fraction_false_given = 0.5,
				.fraction_discoverable = 0.5
		},
};




int main(int argc, char **argv) {

	// Load the apple tree meshes.
	auto models = loadRandomTreeModels(2, 600);

	// Load the robot model.
	const auto robot = loadRobotModel();

	using namespace ranges;

	ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

	const int REPETITIONS = 10;

	auto repIds = ranges::views::iota(0, 10);

	DMGPlannerAllocatorFn sp = static_planner<CGALMeshShellPoint>(cgalChullShell);

	std::array<std::pair<std::string, DMGPlannerAllocatorFn>, 9> PLANNERS_TO_TEST = {
			// A planner that ignores the dynamic goalset, only planning to the initially-given apples.
			// It will obviously not ve very optimal, but will serve as a baseline.
			std::make_pair("change_ignoring", sp),
			// A planner that adds new goals to a "batch" to be replanned to after the current
			// path has been completed.
			{"batch_replanner", batch_replanner<CGALMeshShellPoint>(cgalChullShell)},
			// A planner that uses the dynamic goalset, but uses a greedy approach to insert new goals in the visitation order.
			{"dynamic_planner_lci", dynamic_planner_simple_reorder_sphere<CGALMeshShellPoint>(SimpleIncrementalTSPMethods::Strategy::LeastCostlyInsertion,cgalChullShell)},
			// A planner that inserts new goals simply at the end of the tour
			{"dynamic_planner_LIFO", dynamic_planner_simple_reorder_sphere<CGALMeshShellPoint>(SimpleIncrementalTSPMethods::Strategy::LastInFirstOut,cgalChullShell)},
			// A planner that inserts new goals simply at the beginning of the tour
			{"dynamic_planner_FIFO", dynamic_planner_simple_reorder_sphere<CGALMeshShellPoint>(SimpleIncrementalTSPMethods::Strategy::FirstInFirstOut,cgalChullShell)},
			// A planner that puts goals at the second place in the order.
			{"dynamic_planner_FISO", dynamic_planner_simple_reorder_sphere<CGALMeshShellPoint>(SimpleIncrementalTSPMethods::Strategy::FirstInSecondOut,cgalChullShell)},
			// A planner that randomizes the order of the goals.
			{"dynamic_planner_random", dynamic_planner_simple_reorder_sphere<CGALMeshShellPoint>(SimpleIncrementalTSPMethods::Strategy::Random,cgalChullShell)},
			// Same as dynamic_planner_fre, but with an initial orbit around the tree to discover some of the dynamic goals
			{"dynamic_planner_initial_orbit", dynamic_planner_initial_orbit<CGALMeshShellPoint>(cgalChullShell)},
			// A planner that uses the dynamic goalset, and completely reorders the visitation order from scratch every time a goal is added.
			{"dynamic_planner_fre", dynamic_planner_fre<CGALMeshShellPoint>(cgalChullShell)},
	};

	// Generate a set of problems based on the carthesian product of the above ranges.
	auto problems =
			views::cartesian_product(models, repIds, probs, OCCLUSION_MODELS) |
			views::transform([&](const auto &pair) -> std::pair<Json::Value,DynamicGoalsetPlanningProblem> {

				// Generate a problem for every unique combination of repetition ID,
				// discoverability degree, total number of apples and occlusion model.
				auto &[scene, repId, prob, occlusion_model] = pair;

				// Translate the discoverability degree into a vector of discoverability types/
				const auto discoverability = generateAppleDiscoverability(prob,
																		  repId,
																		  scene.fruit_meshes.size());

				// Create a JSON object containing the parameters of the problem for later reference.
				Json::Value problem_params;
				problem_params["n_given"] = ranges::count(discoverability, AppleDiscoverabilityType::GIVEN);
				problem_params["n_discoverable"] = ranges::count(discoverability, AppleDiscoverabilityType::DISCOVERABLE);
				problem_params["n_false"] = ranges::count(discoverability, AppleDiscoverabilityType::FALSE);
				problem_params["n_total"] = scene.fruit_meshes.size();
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

			}) | to_vector;



	// Take the carthesian product of the different planners and problems,
	// making it so that every planner is tested on every problem.
	auto experiments = views::cartesian_product(PLANNERS_TO_TEST, problems) | views::transform([](const auto &pair) {
		const auto &[planner, problem] = pair;
		return Experiment {
			.planner= &planner,
			.problem= &problem
		};
	}) | to_vector;

	// Run the experiments in parallel.
	runExperimentsParallelRecoverable<Experiment>(experiments, [&](const Experiment &experiment) {
		return runDynamicPlannerExperiment(robot, experiment);
	}, "analysis/data/dynamic_log_ral.json", 32, std::thread::hardware_concurrency(), 42);

}