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

/**
 * @brief Constructs a set of standard occlusion models.
 *
 * These models provide various ways to determine whether an apple is occluded,
 * based on several different parameters including distance, angle of view, mesh occlusion,
 * and combinations of these factors.
 *
 * @param meshes Tree meshes to be used for occlusion.
 * @return An array of pairs. Each pair consists of a string representing the occlusion model's name,
 *         and a function representing the occlusion model.
 */
std::array<std::pair<std::string, CanSeeAppleFn>, 12> mk_standard_occlusion_models(TreeMeshes &meshes) {

	std::pair<std::string, CanSeeAppleFn> omni_pair = {"omniscient", omniscient_occlusion};

	return {
			// Basic occlusion functions
			omni_pair, // I don't know why this needs to be separated out, but the type inference fails otherwise
			{"distance", distance_occlusion},

			// Field of view occlusions
			{"angle_end_effector", in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector")},
			{"angle_base_link", in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link")},

			// Mesh occlusions
			{"mesh_occlusion_end_effector", mesh_occludes_vision(meshes.leaves_mesh, "end_effector")},
			{"mesh_occlusion_base_link", mesh_occludes_vision(meshes.leaves_mesh, "base_link")},

			// Alpha shape occlusions
			{"alpha_occlusion_end_effector", leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "end_effector")},
			{"alpha_occlusion_base_link", leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "base_link")},

			// Combinations of mesh occlusion and angle occlusion
			{"mesh_and_angle_end_effector", only_if_both(
					mesh_occludes_vision(meshes.leaves_mesh, "end_effector"),
					in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector")
			)},
			{"mesh_and_angle_base_link", only_if_both(
					mesh_occludes_vision(meshes.leaves_mesh, "base_link"),
					in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link")
			)},

			// Combinations of alpha shape occlusion and angle occlusion
			{"alpha_and_angle_end_effector", only_if_both(
					leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "end_effector"),
					in_angle(1.0, Eigen::Vector3d::UnitX(), "end_effector")
			)},
			{"alpha_and_angle_base_link", only_if_both(
					leaves_alpha_shape_occludes_vision(meshes.leaves_mesh, "base_link"),
					in_angle(1.0, Eigen::Vector3d::UnitX(), "base_link")
			)}
	};
}

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
	TreeMeshes meshes = loadTreeMeshes("appletree");

	// Convert the meshes to a planning scene message.
	const auto scene = AppleTreePlanningScene {
		.scene_msg = std::make_shared<moveit_msgs::msg::PlanningScene>(std::move(treeMeshesToMoveitSceneMsg(meshes))),
		.apples = meshes.fruit_meshes | ranges::views::transform(appleFromMesh) | ranges::to_vector
	};

	// Load the robot model.
	const auto robot = loadRobotModel();

	using namespace ranges;

	ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

	auto repIds = ranges::views::iota(0, 50);

	// Numbers of apples to throw at the planner.
	const auto nApples = {10, 50, 100};

	std::array<std::pair<std::string, DynamicPlannerAllocatorFn>, 9> PLANNERS_TO_TEST = {
			// A planner that ignores the dynamic goalset, only planning to the initially-given apples.
			// It will obviously not ve very optimal, but will serve as a baseline.
			std::make_pair("change_ignoring", static_planner),
			// A planner that adds new goals to a "batch" to be replanned to after the current
			// path has been completed.
			{"batch_replanner", batch_replanner},
			// A planner that uses the dynamic goalset, but uses a greedy approach to insert new goals in the visitation order.
			{"dynamic_planner_lci",
			 dynamic_planner_simple_reorder_sphere(SimpleIncrementalTSPMethods::LeastCostlyInsertion)},
			// A planner that inserts new goals simply at the end of the tour
			{"dynamic_planner_LIFO", dynamic_planner_simple_reorder_sphere(SimpleIncrementalTSPMethods::LastInFirstOut)},
			// A planner that inserts new goals simply at the beginning of the tour
			{"dynamic_planner_FIFO", dynamic_planner_simple_reorder_sphere(SimpleIncrementalTSPMethods::FirstInFirstOut)},
			// A planner that puts goals at the second place in the order.
			{"dynamic_planner_FISO",
			 dynamic_planner_simple_reorder_sphere(SimpleIncrementalTSPMethods::FirstInSecondOut)},
			// A planner that randomizes the order of the goals.
			{"dynamic_planner_random", dynamic_planner_simple_reorder_sphere(SimpleIncrementalTSPMethods::Random)},
			// Same as dynamic_planner_fre, but with an initial orbit around the tree to discover some of the dynamic goals
			{"dynamic_planner_initial_orbit", dynamic_planner_initial_orbit},
			// A planner that uses the dynamic goalset, and completely reorders the visitation order from scratch every time a goal is added.
			{"dynamic_planner_fre", dynamic_planner_fre},
	};

	std::array<std::pair<std::string, CanSeeAppleFn>, 12> can_see_apple_fns = mk_standard_occlusion_models(meshes);

	// Generate a set of problems based on the carthesian product of the above ranges.
	auto problems =
			views::cartesian_product(repIds, probs, nApples, can_see_apple_fns) |
			views::transform([&](const auto &pair) -> std::pair<Json::Value,DynamicGoalsetPlanningProblem> {

				// Generate a problem for every unique combination of repetition ID,
				// discoverability degree, total number of apples and occlusion model.
				const auto &[repId, prob, n_total, can_see_apple] = pair;

				AppleTreePlanningScene censored_scene = scene;
				// Shuffle the apples.
				std::shuffle(censored_scene.apples.begin(), censored_scene.apples.end(), std::mt19937(repId));
				// Delete any over n.
				censored_scene.apples.resize(n_total);

				// Translate the discoverability degree into a vector of discoverability types/
				const auto discoverability = generateAppleDiscoverability(prob,
																		  repId,
																		  n_total);

				// Create a JSON object containing the parameters of the problem for later reference.
				Json::Value problem_params;
				problem_params["n_given"] = ranges::count(discoverability, AppleDiscoverabilityType::GIVEN);
				problem_params["n_discoverable"] = ranges::count(discoverability, AppleDiscoverabilityType::DISCOVERABLE);
				problem_params["n_false"] = ranges::count(discoverability, AppleDiscoverabilityType::FALSE);
				problem_params["n_total"] = n_total;
				problem_params["visibility_model"] = can_see_apple.first;

				// Create the problem, to be solved by the planners.
				DynamicGoalsetPlanningProblem problem{
					.start_state= randomStateOutsideTree(robot,repId),
					.scene=        censored_scene,
					.apple_discoverability=    discoverability,
					.can_see_apple=    &can_see_apple.second};

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
	}, "analysis/data/dynamic_log_advanced_4.json", 32, std::thread::hardware_concurrency(), 42);

}